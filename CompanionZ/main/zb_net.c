#include "zb_net.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "mdns.h"
#include "nvs.h"
#include "zb_cal.h"
#include "zb_settings.h"
#include "zbro_proto.h"

#define NVS_NS         "zbwifi"
#define SSID_MAX       32
#define PASS_MIN       8
#define PASS_MAX       63
#define STA_FAIL_LIMIT 8              /* failed attempts before the provisioning AP comes up alongside */
#define BEACON_MS      2000

static const char *TAG = "zb_net";

static esp_netif_t *s_sta_netif, *s_ap_netif;
static char s_ssid[SSID_MAX + 1], s_pass[PASS_MAX + 1];
static bool s_have_creds;
static volatile bool s_sta_up, s_ap_on;
static volatile int s_fail_count;
static char s_sta_ip[16];
static char s_host[33];

static SemaphoreHandle_t s_tx_mtx;
static int s_cfd = -1;                /* client socket, guarded by s_tx_mtx */

/* ------------------------------------------------------------ creds --- */

static void creds_load(void)
{
    nvs_handle_t h;
    size_t n;
    s_ssid[0] = s_pass[0] = '\0';
    s_have_creds = false;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    n = sizeof(s_ssid);
    if (nvs_get_str(h, "ssid", s_ssid, &n) == ESP_OK) {
        n = sizeof(s_pass);
        if (nvs_get_str(h, "pass", s_pass, &n) != ESP_OK) s_pass[0] = '\0';
        s_have_creds = s_ssid[0] != '\0';
    }
    nvs_close(h);
}

int zb_net_wifi_set(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    size_t sl = ssid ? strlen(ssid) : 0, pl = pass ? strlen(pass) : 0;
    if (sl < 1 || sl > SSID_MAX || pl > PASS_MAX || (pl > 0 && pl < PASS_MIN)) return ZBRO_E_ARG;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return ZBRO_E_FAULT;
    esp_err_t e = nvs_set_str(h, "ssid", ssid);
    if (e == ESP_OK) e = nvs_set_str(h, "pass", pass);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e == ESP_OK ? 0 : ZBRO_E_FAULT;
}

void zb_net_wifi_ssid(char *buf, size_t n)
{
    snprintf(buf, n, "%s", s_ssid);
}

static void reboot_cb(void *arg)
{
    (void)arg;
    zb_estop();
    esp_restart();
}

void zb_net_reboot_later(unsigned ms)
{
    static esp_timer_handle_t t;
    if (!t) {
        const esp_timer_create_args_t a = {.callback = reboot_cb, .name = "zb_reboot"};
        if (esp_timer_create(&a, &t) != ESP_OK) return;
    }
    esp_timer_start_once(t, (uint64_t)ms * 1000);
}

/* ------------------------------------------------------------- wifi --- */

bool zb_net_s3_connected(void) { return s_cfd >= 0; }
bool zb_net_ap_active(void) { return s_ap_on; }
bool zb_net_sta_connected(void) { return s_sta_up; }

int zb_net_rssi(void)
{
    wifi_ap_record_t ap;
    if (!s_sta_up || esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return 0;
    return ap.rssi;
}

void zb_net_ip(char *buf, size_t n)
{
    if (s_sta_up) {
        snprintf(buf, n, "%s", s_sta_ip);
    } else if (s_ap_on) {
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(s_ap_netif, &ip) == ESP_OK)
            snprintf(buf, n, IPSTR, IP2STR(&ip.ip));
        else
            snprintf(buf, n, "192.168.4.1");
    } else {
        snprintf(buf, n, "0.0.0.0");
    }
}

static void ap_config(wifi_config_t *c)
{
    uint8_t mac[6];
    memset(c, 0, sizeof(*c));
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    int l = snprintf((char *)c->ap.ssid, sizeof(c->ap.ssid), "Z-Bro-%02X%02X", mac[4], mac[5]);
    c->ap.ssid_len = (uint8_t)l;
    c->ap.channel = 1;
    c->ap.max_connection = 3;
    c->ap.authmode = WIFI_AUTH_OPEN;
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_sta_up = false;
        s_sta_ip[0] = '\0';
        if (s_have_creds) {
            if (++s_fail_count == STA_FAIL_LIMIT && !s_ap_on) {
                wifi_config_t ap;
                ap_config(&ap);
                esp_wifi_set_config(WIFI_IF_AP, &ap);
                esp_wifi_set_mode(WIFI_MODE_APSTA);
                s_ap_on = true;
            }
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = data;
        snprintf(s_sta_ip, sizeof(s_sta_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        s_sta_up = true;
        s_fail_count = 0;
        if (s_ap_on && s_have_creds) {            /* fallback AP no longer needed */
            esp_wifi_set_mode(WIFI_MODE_STA);
            s_ap_on = false;
        }
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();
    esp_netif_set_hostname(s_sta_netif, s_host);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    if (s_have_creds) {
        wifi_config_t sta;
        memset(&sta, 0, sizeof(sta));
        snprintf((char *)sta.sta.ssid, sizeof(sta.sta.ssid), "%s", s_ssid);
        snprintf((char *)sta.sta.password, sizeof(sta.sta.password), "%s", s_pass);
        sta.sta.threshold.authmode = s_pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
        ESP_LOGI(TAG, "joining '%s'", s_ssid);
    } else {
        wifi_config_t ap;
        ap_config(&ap);
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
        s_ap_on = true;
        ESP_LOGW(TAG, "no credentials: provisioning AP '%s'", (char *)ap.ap.ssid);
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);                /* latency over power: motion commands */
}

static void mdns_start(void)
{
    if (mdns_init() != ESP_OK) return;
    mdns_hostname_set(s_host);
    mdns_instance_name_set("Z-Bro");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    mdns_service_add(NULL, "_zbro", "_tcp", ZBRO_PORT_TCP, NULL, 0);
}

/* ------------------------------------------------------- COM link ----- */

/* Non-blocking send of one complete line. Any failure drops the client (the S3 reconnects). */
static void send_line(const char *s, int n)
{
    if (n <= 0 || !s_tx_mtx) return;
    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
    if (s_cfd >= 0) {
        int off = 0;
        while (off < n) {
            int r = send(s_cfd, s + off, n - off, MSG_DONTWAIT);
            if (r <= 0) {
                if (r < 0 && errno == EINTR) continue;
                shutdown(s_cfd, SHUT_RDWR);       /* the RX loop sees EOF and closes */
                break;
            }
            off += r;
        }
    }
    xSemaphoreGive(s_tx_mtx);
}

static void send_err(int code, const char *text)
{
    char b[ZBRO_MAX_LINE];
    send_line(b, zbro_fmt_err(b, sizeof(b), code, text));
}

const char *zb_err_text(int code)
{
    switch (code) {
    case ZBRO_E_BUSY:      return "busy";
    case ZBRO_E_NOT_HOMED: return "not homed";
    case ZBRO_E_RANGE:     return "out of range";
    case ZBRO_E_ARG:       return "bad argument";
    case ZBRO_E_ENDSTOP:   return "endstop";
    case ZBRO_E_HOME_FAIL: return "homing failed";
    case ZBRO_E_FAULT:     return "fault";
    default:               return "unknown command";
    }
}

static void reply_rc(int rc)
{
    char b[ZBRO_MAX_LINE];
    if (rc == 0) send_line(b, zbro_fmt_ok(b, sizeof(b)));
    else send_err(rc, zb_err_text(rc));
}

static void send_pos(void)
{
    struct zb_status s;
    char b[ZBRO_MAX_LINE];
    zb_motion_status(&s);
    send_line(b, zbro_fmt_pos(b, sizeof(b), s.pos, (int)s.state, s.homed, s.endstop));
}

void zb_net_on_event(enum zb_evt_type type, float pos_mm, int err)
{
    char b[ZBRO_MAX_LINE];
    int n;
    switch (type) {
    case ZB_EVT_HOMED: n = zbro_fmt_evt_homed(b, sizeof(b), pos_mm); break;
    case ZB_EVT_DONE:  n = zbro_fmt_evt_done(b, sizeof(b), pos_mm); break;
    default:           n = zbro_fmt_evt_err(b, sizeof(b), err); break;
    }
    send_line(b, n);
}

void zb_net_on_cal(void)
{
    char *json = zb_cal_json();
    if (!json) return;
    for (char *p = json; *p; p++)
        if (*p == '\n' || *p == '\r') *p = ' ';
    size_t sz = strlen(json) + 16;
    char *b = malloc(sz);
    if (b) {
        int n = snprintf(b, sz, "EVT CAL %s\n", json);
        if (n > 0 && (size_t)n < sz) send_line(b, n);
        free(b);
    }
    free(json);
}

static void handle_request(const struct zbro_req *r)
{
    char b[ZBRO_MAX_LINE];
    switch (r->cmd) {
    case ZBRO_C_HELLO: send_line(b, zbro_fmt_hello(b, sizeof(b), ZB_FW_VERSION)); break;
    case ZBRO_C_PING:  send_line(b, zbro_fmt_ok(b, sizeof(b))); break;
    case ZBRO_C_POS:   send_pos(); break;
    case ZBRO_C_HOME:  reply_rc(zb_home()); break;
    case ZBRO_C_MOVE:  reply_rc(zb_move_abs(r->a, r->has_b ? r->b : 0)); break;
    case ZBRO_C_JOG:   reply_rc(zb_move_rel(r->a, r->has_b ? r->b : 0)); break;
    case ZBRO_C_STOP:  reply_rc(zb_stop()); break;
    case ZBRO_C_ESTOP: zb_estop(); reply_rc(0); break;
    case ZBRO_C_GET: {
        float v;
        if (zb_set_get_by_key(r->key, &v)) send_line(b, zbro_fmt_val(b, sizeof(b), r->key, v));
        else send_err(ZBRO_E_ARG, "unknown key");
        break;
    }
    case ZBRO_C_SET:
        if (zb_set_find(r->key) < 0) send_err(ZBRO_E_ARG, "unknown key");
        else if (zb_set_set_by_key(r->key, r->a, false) != 0) send_err(ZBRO_E_ARG, "value rejected");
        else reply_rc(0);
        break;
    case ZBRO_C_SAVE: zb_set_save(); reply_rc(0); break;
    case ZBRO_C_CAL:
        if (r->cal_op == ZBRO_CAL_STATUS) {
            char *json = zb_cal_json();
            if (!json) { send_err(ZBRO_E_FAULT, "no memory"); break; }
            for (char *p = json; *p; p++)
                if (*p == '\n' || *p == '\r') *p = ' ';
            size_t sz = strlen(json) + 16;
            char *o = malloc(sz);
            if (o) {
                int n = zbro_fmt_cal(o, sz, json);
                send_line(o, n);
                free(o);
            }
            free(json);
        } else if (r->cal_op == ZBRO_CAL_ABORT) {
            zb_cal_abort();
            reply_rc(0);
        } else {
            const char *t[ZBRO_CAL_TESTS];
            for (int i = 0; i < r->ntests; i++) t[i] = r->tests[i];
            reply_rc(zb_cal_start(t, r->ntests) ? 0 : ZBRO_E_BUSY);
        }
        break;
    }
}

static void set_client_opts(int fd)
{
    int on = 1, idle = 5, intvl = 2, cnt = 3;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
}

static void drop_client(void)
{
    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
    if (s_cfd >= 0) {
        close(s_cfd);
        s_cfd = -1;
    }
    xSemaphoreGive(s_tx_mtx);
}

static void tcp_task(void *arg)
{
    (void)arg;
    static struct zbro_lines lines;
    int lfd = -1;
    TickType_t last_rpt = 0;
    int prev_state = ZB_IDLE;

    for (;;) {
        if (lfd < 0) {
            lfd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
            struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons(ZBRO_PORT_TCP)};
            a.sin_addr.s_addr = htonl(INADDR_ANY);
            int one = 1;
            if (lfd >= 0) setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
            if (lfd < 0 || bind(lfd, (struct sockaddr *)&a, sizeof(a)) != 0 || listen(lfd, 1) != 0) {
                if (lfd >= 0) close(lfd);
                lfd = -1;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            fcntl(lfd, F_SETFL, fcntl(lfd, F_GETFL, 0) | O_NONBLOCK);
        }

        struct zb_status st;
        zb_motion_status(&st);
        int cfd = s_cfd;
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(lfd, &rd);
        int maxfd = lfd;
        if (cfd >= 0) {
            FD_SET(cfd, &rd);
            if (cfd > maxfd) maxfd = cfd;
        }
        struct timeval tv = {.tv_sec = 0, .tv_usec = (st.state != ZB_IDLE ? 20 : 100) * 1000};
        int n = select(maxfd + 1, &rd, NULL, NULL, &tv);

        if (n > 0 && FD_ISSET(lfd, &rd)) {                 /* new client replaces the old one */
            int nfd = accept(lfd, NULL, NULL);
            if (nfd >= 0) {
                drop_client();
                set_client_opts(nfd);
                zbro_lines_init(&lines);
                xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
                s_cfd = nfd;
                xSemaphoreGive(s_tx_mtx);
                cfd = nfd;
                ESP_LOGI(TAG, "S3 connected");
            }
        } else if (n > 0 && cfd >= 0 && FD_ISSET(cfd, &rd)) {
            char chunk[64];
            int r = recv(cfd, chunk, sizeof(chunk), 0);
            if (r <= 0 && !(r < 0 && (errno == EAGAIN || errno == EINTR))) {
                drop_client();
                ESP_LOGI(TAG, "S3 disconnected");
                cfd = -1;
            }
            for (int i = 0; i < r; i++) {
                const char *line;
                size_t len;
                int f = zbro_lines_feed(&lines, chunk[i], &line, &len);
                if (f == 1) {
                    struct zbro_req req;
                    int rc = zbro_parse_request(line, len, &req);
                    if (rc == ZBRO_E_NONE) handle_request(&req);
                    else send_err(rc, zb_err_text(rc));
                } else if (f < 0) {
                    send_err(ZBRO_E_ARG, "line too long");
                }
            }
        }

        /* POS reports: every rpt_hz while moving, once more when the axis comes to rest. */
        if (cfd >= 0) {
            float hz = zb_set_get(ZS_RPT_HZ);
            TickType_t period = pdMS_TO_TICKS(hz >= 1.0f ? (int)(1000.0f / hz) : 1000);
            TickType_t now = xTaskGetTickCount();
            bool moving = st.state != ZB_IDLE;
            if ((moving && now - last_rpt >= period) || (!moving && prev_state != ZB_IDLE)) {
                send_pos();
                last_rpt = now;
            }
        }
        prev_state = st.state;
    }
}

/* ----------------------------------------------------------- beacon --- */

static void beacon_task(void *arg)
{
    (void)arg;
    int fd = -1;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(BEACON_MS));
        if (!s_sta_up) continue;
        if (fd < 0) {
            fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (fd < 0) continue;
            int one = 1;
            setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
        }
        char b[ZBRO_MAX_LINE];
        int n = zbro_fmt_beacon(b, sizeof(b), s_sta_ip, ZBRO_PORT_TCP, s_host);
        struct sockaddr_in to = {.sin_family = AF_INET, .sin_port = htons(ZBRO_PORT_UDP)};
        to.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        if (n > 0 && sendto(fd, b, n, 0, (struct sockaddr *)&to, sizeof(to)) < 0) {
            close(fd);                                  /* network went away: reopen next round */
            fd = -1;
        }
    }
}

void zb_net_start(void)
{
    s_tx_mtx = xSemaphoreCreateMutex();
    zb_hostname_get(s_host, sizeof(s_host));
    if (!s_host[0]) snprintf(s_host, sizeof(s_host), "zbro");
    creds_load();
    wifi_start();
    mdns_start();
    xTaskCreate(tcp_task, "zb_tcp", 4096, NULL, 5, NULL);
    xTaskCreate(beacon_task, "zb_beacon", 3072, NULL, 3, NULL);
}
