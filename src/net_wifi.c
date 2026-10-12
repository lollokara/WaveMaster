#include "net_wifi.h"

#include <stdio.h>
#include <string.h>

#include "calib.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "net_server.h"
#include "nvs.h"

#define MGR_TASK_STACK 4096
#define MGR_TASK_PRIO  4
#define MGR_PERIOD_MS  500
#define BACKOFF_MIN_MS 1000
#define BACKOFF_MAX_MS 15000
#define NVS_NS         "wmwifi"
#define SSID_MAX       32
#define PASS_MIN       8
#define PASS_MAX       63
#define HOSTNAME       "wavemaster"

static const char *TAG = "net_wifi";

enum { ST_OFF, ST_CONNECTING, ST_UP };

static volatile int s_state = ST_OFF;
static volatile unsigned s_creds_gen = 1;
static char s_ssid[SSID_MAX + 1];
static char s_pass[PASS_MAX + 1];
static esp_netif_t *s_netif;
static bool s_inited;                   /* driver initialised (manager task only) */
static bool s_running;                  /* esp_wifi_start() done (manager task only) */
static volatile bool s_want_connect;
static volatile bool s_ev_lost;         /* set by the event handler, logged by mgr_task */
static volatile bool s_ev_ip;
static int s_init_fails;
#define INIT_FAILS_MAX 3
static volatile int s_rssi;
static char s_ip[16];

/* ------------------------------------------------------ credentials ---- */

static bool creds_load(void)
{
    nvs_handle_t h;
    size_t n;
    bool ok = false;

    s_ssid[0] = s_pass[0] = '\0';
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK)
        return false;
    n = sizeof(s_ssid);
    if (nvs_get_str(h, "ssid", s_ssid, &n) == ESP_OK) {
        n = sizeof(s_pass);
        ok = nvs_get_str(h, "pass", s_pass, &n) == ESP_OK && s_ssid[0];
    }
    nvs_close(h);
    if (!ok)
        s_ssid[0] = s_pass[0] = '\0';
    return ok;
}

static int creds_store(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t e;

    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK)
        return 3;
    if (ssid) {
        e = nvs_set_str(h, "ssid", ssid);
        if (e == ESP_OK)
            e = nvs_set_str(h, "pass", pass);
    } else {
        nvs_erase_all(h);
        e = ESP_OK;
    }
    if (e == ESP_OK)
        e = nvs_commit(h);
    nvs_close(h);
    return e == ESP_OK ? 0 : 3;
}

/* ------------------------------------------------------------ events --- */

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    /* Runs in the default event loop task (core 0, above dac_task's priority):
     * flags only, no logging, no host I/O. mgr_task does the rest. */
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_state == ST_UP)
            s_ev_lost = true;
        s_state = ST_CONNECTING;
        s_ip[0] = '\0';
        s_want_connect = true;
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = data;

        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        s_state = ST_UP;
        s_ev_ip = true;
    }
}

/* -------------------------------------------------- driver lifecycle --- */

/* Manager task only: this is what puts the WiFi interrupt on core 1. */
static bool driver_init(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t e;

    if (s_inited)
        return true;
    e = esp_netif_init();
    if (e == ESP_OK || e == ESP_ERR_INVALID_STATE)
        e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop: %s", esp_err_to_name(e));
        return false;
    }
    /* The "sta ip" line esp_netif logs from the event task would be a host
     * write on core 0. */
    esp_log_level_set("esp_netif_handlers", ESP_LOG_WARN);
    if (!s_netif) {
        s_netif = esp_netif_create_default_wifi_sta();    /* once: a second create asserts */
        if (!s_netif)
            return false;
        esp_netif_set_hostname(s_netif, HOSTNAME);
    }
    e = esp_wifi_init(&cfg);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init: %s", esp_err_to_name(e));
        return false;
    }
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    s_inited = true;
    net_server_start();
    return true;
}

static void apply_config(void)
{
    wifi_config_t wc;

    memset(&wc, 0, sizeof(wc));
    strlcpy((char *)wc.sta.ssid, s_ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, s_pass, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;     /* no open / WEP / WPA1 */
    wc.sta.pmf_cfg.capable = true;
    esp_wifi_set_config(WIFI_IF_STA, &wc);
}

static void driver_stop(void)
{
    net_server_drop_client();           /* no network: the job must not keep marking */
    if (s_running) {
        esp_wifi_disconnect();
        esp_wifi_stop();
        s_running = false;
    }
    s_state = ST_OFF;
    s_ip[0] = '\0';
    s_want_connect = false;
}

static void mgr_task(void *arg)
{
    unsigned seen_gen = 0;
    int backoff = BACKOFF_MIN_MS;
    int wait = 0;

    (void)arg;
    for (;;) {
        bool enabled = calib_get(CAL_WIFI_EN) != 0.0f;
        bool have = s_ssid[0] != '\0';

        if (seen_gen != s_creds_gen) {
            seen_gen = s_creds_gen;
            s_init_fails = 0;
            have = creds_load();
            if (s_running)
                driver_stop();          /* restarts below with the new credentials */
        }
        if (s_ev_lost) {
            s_ev_lost = false;
            ESP_LOGW(TAG, "link lost");
            net_server_drop_client();   /* do not wait for TCP keepalive to notice */
        }
        if (s_ev_ip) {
            s_ev_ip = false;
            ESP_LOGI(TAG, "connected, ip %s", s_ip);
        }
        if (enabled && have) {
            if (!s_running && s_init_fails < INIT_FAILS_MAX) {
                if (driver_init()) {
                    esp_err_t e;

                    apply_config();
                    e = esp_wifi_start();
                    if (e == ESP_OK) {
                        esp_wifi_set_ps(WIFI_PS_NONE);  /* power save adds ~100 ms latency */
                        s_running = true;
                        s_state = ST_CONNECTING;
                        s_want_connect = true;
                        backoff = BACKOFF_MIN_MS;
                        wait = 0;
                    } else {
                        ESP_LOGE(TAG, "esp_wifi_start: %s", esp_err_to_name(e));
                        s_init_fails++;
                    }
                } else {
                    s_init_fails++;
                }
                if (s_init_fails == INIT_FAILS_MAX)
                    ESP_LOGE(TAG, "WiFi start failed %d times, giving up until the credentials change",
                             INIT_FAILS_MAX);
            } else if (s_running && s_want_connect && s_state != ST_UP) {
                if (wait <= 0) {
                    s_want_connect = false;
                    esp_wifi_connect();
                    wait = backoff;
                    backoff = backoff * 2 > BACKOFF_MAX_MS ? BACKOFF_MAX_MS : backoff * 2;
                } else {
                    wait -= MGR_PERIOD_MS;
                }
            }
            if (s_state == ST_UP) {
                wifi_ap_record_t ap;

                backoff = BACKOFF_MIN_MS;
                if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
                    s_rssi = ap.rssi;
            }
        } else if (s_running) {
            driver_stop();
        }
        vTaskDelay(pdMS_TO_TICKS(MGR_PERIOD_MS));
    }
}

bool net_wifi_start(void)
{
    creds_load();
    return xTaskCreatePinnedToCore(mgr_task, "net_mgr", MGR_TASK_STACK, NULL,
                                   MGR_TASK_PRIO, NULL, 1) == pdPASS;
}

bool net_wifi_up(void)
{
    return s_state == ST_UP;
}

/* ------------------------------------------------------------ $WIFI ---- */

static void print_status(void (*out)(const char *))
{
    char line[160];
    const char *st = !s_running ? "off" : (s_state == ST_UP ? "connected" : "connecting");
    int o = snprintf(line, sizeof(line), "[WIFI:enabled=%d creds=%d state=%s",
                     calib_get(CAL_WIFI_EN) != 0.0f, s_ssid[0] != '\0', st);

    if (s_ssid[0] && o < (int)sizeof(line)) {
        char safe[SSID_MAX + 1];
        size_t i;

        for (i = 0; s_ssid[i] && i < SSID_MAX; i++) {
            unsigned char c = (unsigned char)s_ssid[i];

            safe[i] = (c < 0x20 || c == 0x7f || c == ']') ? '?' : (char)c;   /* keep the [WIFI:...] framing */
        }
        safe[i] = '\0';
        o += snprintf(line + o, sizeof(line) - (size_t)o, " ssid=%s", safe);
    }
    if (s_state == ST_UP && o < (int)sizeof(line))
        o += snprintf(line + o, sizeof(line) - (size_t)o, " ip=%s rssi=%d", s_ip, s_rssi);
    if (o < (int)sizeof(line))
        snprintf(line + o, sizeof(line) - (size_t)o, " client=%d port=%d]\r\n",
                 net_server_has_client(), NET_SERVER_PORT);
    out(line);
}

int net_wifi_command(const char *arg, void (*out)(const char *line))
{
    const char *comma;
    char ssid[SSID_MAX + 1];
    size_t sl, pl;

    while (*arg == ' ')
        arg++;
    if (*arg == '\0') {
        print_status(out);
        return 0;
    }
    if (*arg != '=')
        return 3;
    arg++;
    pl = strlen(arg);
    while (pl && arg[pl - 1] == ' ')    /* trailing blanks from a paste are not part of the password */
        pl--;
    if (pl == 5 && strncasecmp(arg, "CLEAR", 5) == 0) {
        if (creds_store(NULL, NULL) != 0)
            return 3;
        s_creds_gen++;
        return 0;
    }
    comma = strchr(arg, ',');
    if (!comma)
        return 3;
    sl = (size_t)(comma - arg);
    pl = (size_t)((arg + pl) - (comma + 1));
    if (sl == 0 || sl > SSID_MAX || pl < PASS_MIN || pl > PASS_MAX)
        return 3;                       /* WPA2 needs 8..63 characters */
    {
        char pass[PASS_MAX + 1];

        memcpy(ssid, arg, sl);
        ssid[sl] = '\0';
        memcpy(pass, comma + 1, pl);
        pass[pl] = '\0';
        if (creds_store(ssid, pass) != 0)
            return 3;
    }
    s_creds_gen++;                      /* the manager reloads and reconnects */
    return 0;
}
