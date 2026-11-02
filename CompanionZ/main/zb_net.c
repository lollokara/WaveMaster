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
#include "zb_fleet.h"
#include "zb_settings.h"
#include "zbro_proto.h"

#define NVS_NS         "zbwifi"
#define SSID_MAX       32
#define PASS_MIN       8
#define PASS_MAX       63
#define STA_FAIL_LIMIT 8              /* failed attempts before the provisioning AP comes up alongside */
#define BEACON_MS      2000
#define AP_WINDOW_MS   (10 * 60 * 1000)   /* fallback AP stays up this long after the STA gave up */
#define PING_TIMEOUT_MS 8000              /* no traffic from the client for this long during its move: STOP. The S3
                                           * pings every second; TCP retransmits after a WiFi loss burst take 5-7 s */
#define TX_STALL_MS    3000               /* a full send buffer is waited out this long before the client is dropped */

static const char *TAG = "zb_net";

static esp_netif_t *s_sta_netif, *s_ap_netif;
static char s_ssid[SSID_MAX + 1], s_pass[PASS_MAX + 1];
static bool s_have_creds;
static volatile bool s_sta_up, s_ap_on;
static volatile int s_fail_count;
static bool s_ap_window_used;         /* the post-failure AP window opens once per boot */
static esp_timer_handle_t s_ap_timer;
static volatile bool s_client_op;     /* a MOVE/JOG/HOME requested over :7700 is in flight */
static volatile int64_t s_last_rx_us;
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

/* WPA2-PSK provisioning AP. Password "zbro" + 8 hex chars of the MAC (bytes 2..5), printed on the USB
 * console at boot, so the owner can always compute it. */
static void ap_config(wifi_config_t *c)
{
    uint8_t mac[6];
    memset(c, 0, sizeof(*c));
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    int l = snprintf((char *)c->ap.ssid, sizeof(c->ap.ssid), "Z-Bro-%02X%02X", mac[4], mac[5]);
    c->ap.ssid_len = (uint8_t)l;
    snprintf((char *)c->ap.password, sizeof(c->ap.password), "zbro%02x%02x%02x%02x", mac[2], mac[3], mac[4], mac[5]);
    c->ap.channel = 1;
    c->ap.max_connection = 3;
    c->ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_LOGW(TAG, "provisioning AP '%s' password '%s'", (char *)c->ap.ssid, (char *)c->ap.password);
}

static void ap_window_end(void *arg)
{
    (void)arg;
    if (s_ap_on && s_have_creds) {                /* window over: STA retries continue, AP off */
        esp_wifi_set_mode(WIFI_MODE_STA);
        s_ap_on = false;
        ESP_LOGW(TAG, "provisioning AP window closed");
    }
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
            if (++s_fail_count >= STA_FAIL_LIMIT && !s_ap_on && !s_ap_window_used) {
                wifi_config_t ap;
                ap_config(&ap);
                esp_wifi_set_config(WIFI_IF_AP, &ap);
                esp_wifi_set_mode(WIFI_MODE_APSTA);
                s_ap_on = true;
                s_ap_window_used = true;
                if (!s_ap_timer) {
                    const esp_timer_create_args_t a = {.callback = ap_window_end, .name = "zb_ap_win"};
                    esp_timer_create(&a, &s_ap_timer);
                }
                if (s_ap_timer) esp_timer_start_once(s_ap_timer, (uint64_t)AP_WINDOW_MS * 1000);
            }
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = data;
        wifi_ap_record_t ap;

        snprintf(s_sta_ip, sizeof(s_sta_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
            ESP_LOGI(TAG, "joined ap %02x:%02x:%02x:%02x:%02x:%02x ch %u rssi %d",
                     ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5],
                     (unsigned)ap.primary, ap.rssi);
        s_sta_up = true;
        s_fail_count = 0;
        if (s_ap_on && s_have_creds) {            /* fallback AP no longer needed */
            esp_wifi_set_mode(WIFI_MODE_STA);
            s_ap_on = false;
            if (s_ap_timer) esp_timer_stop(s_ap_timer);
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
        memcpy(sta.sta.ssid, s_ssid, strlen(s_ssid));          /* struct is zeroed, SSID <= 32 */
        memcpy(sta.sta.password, s_pass, strlen(s_pass));
        sta.sta.threshold.authmode = s_pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
        sta.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;           /* the default fast scan joins the first AP found: on a */
        sta.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;       /* mesh that is often a far node, so take the strongest */
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
        ESP_LOGI(TAG, "joining '%s'", s_ssid);
    } else {
        wifi_config_t ap;
        ap_config(&ap);
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
        s_ap_on = true;
        ESP_LOGW(TAG, "no credentials: provisioning AP up");
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

/* Sends one complete line without blocking the caller for long. A momentarily full send buffer (EAGAIN: a WiFi
 * hiccup, the S3 slow to ACK) is waited out for TX_STALL_MS, since a half-sent line would break the framing.
 * Any other failure, or a longer stall, drops the client (the S3 reconnects). A droppable line (a periodic
 * progress report) is skipped instead when not a byte of it fits, so reports never cost the link. */
static void send_line_opt(const char *s, int n, bool droppable)
{
    if (n <= 0 || !s_tx_mtx) return;
    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
    if (s_cfd >= 0) {
        int off = 0;
        int64_t t0 = esp_timer_get_time();
        while (off < n) {
            int r = send(s_cfd, s + off, n - off, MSG_DONTWAIT);
            if (r > 0) {
                off += r;
                continue;
            }
            if (r < 0 && errno == EINTR) continue;
            if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) && droppable && off == 0) break;
            if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) &&
                esp_timer_get_time() - t0 < (int64_t)TX_STALL_MS * 1000) {
                fd_set wr;
                struct timeval tv = {.tv_sec = 0, .tv_usec = 20 * 1000};
                FD_ZERO(&wr);
                FD_SET(s_cfd, &wr);
                select(s_cfd + 1, NULL, &wr, NULL, &tv);
                continue;
            }
            ESP_LOGW(TAG, "send failed (%d, errno %d, %d of %d bytes), dropping the client", r, errno, off, n);
            shutdown(s_cfd, SHUT_RDWR);       /* the RX loop sees EOF and closes */
            break;
        }
    }
    xSemaphoreGive(s_tx_mtx);
}

static void send_line(const char *s, int n)
{
    send_line_opt(s, n, false);
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
    case ZBRO_E_STOPPED:   return "stopped";
    default:               return "unknown command";
    }
}

static void reply_rc(int rc)
{
    char b[ZBRO_MAX_LINE];
    if (rc == 0) send_line(b, zbro_fmt_ok(b, sizeof(b)));
    else send_err(rc, zb_err_text(rc));
}

static void send_pos(bool droppable)
{
    struct zb_status s;
    char b[ZBRO_MAX_LINE];
    zb_motion_status(&s);
    send_line_opt(b, zbro_fmt_pos(b, sizeof(b), s.pos, (int)s.state, s.homed, s.endstop), droppable);
}

void zb_net_on_event(enum zb_evt_type type, float pos_mm, int err)
{
    char b[ZBRO_MAX_LINE];
    int n;
    s_client_op = false;                          /* any terminal event ends the client's operation */
    switch (type) {
    case ZB_EVT_HOMED: n = zbro_fmt_evt_homed(b, sizeof(b), pos_mm); break;
    case ZB_EVT_DONE:  n = zbro_fmt_evt_done(b, sizeof(b), pos_mm); break;
    default:           n = zbro_fmt_evt_err(b, sizeof(b), err); break;
    }
    send_line(b, n);
}

/* Send the compact CAL summary (no trials, bounded) as one line: "EVT CAL <json>" or "CAL <json>". */
static void cal_send(bool evt)
{
    const char *pre = evt ? "EVT CAL " : "CAL ";
    size_t pl = strlen(pre);
    char *o = malloc(pl + ZB_CAL_COMPACT_MAX + 2);
    if (!o) return;
    memcpy(o, pre, pl);
    size_t n = zb_cal_json_compact(o + pl, ZB_CAL_COMPACT_MAX + 1);
    if (n > 0) {
        o[pl + n] = '\n';
        send_line(o, (int)(pl + n + 1));
    } else if (!evt) {
        send_err(ZBRO_E_FAULT, "cal status too long");
    }
    free(o);
}

void zb_net_on_cal(void)
{
    static unsigned last_phase = ~0u;
    unsigned ph = zb_cal_phase();
    if (ph == last_phase) return;                 /* progress ticks are not pushed over :7700 */
    last_phase = ph;
    if (s_cfd >= 0) cal_send(true);
}

static void client_op_rc(int rc)
{
    if (rc != 0) s_client_op = false;
    reply_rc(rc);
}

static void handle_request(const struct zbro_req *r)
{
    char b[ZBRO_MAX_LINE];
    switch (r->cmd) {
    case ZBRO_C_HELLO: send_line(b, zbro_fmt_hello(b, sizeof(b), ZB_FW_VERSION)); break;
    case ZBRO_C_PING:  send_line(b, zbro_fmt_ok(b, sizeof(b))); break;
    case ZBRO_C_POS:   send_pos(false); break;
    case ZBRO_C_HOME:  s_client_op = true; client_op_rc(zb_home()); break;
    case ZBRO_C_MOVE:  s_client_op = true; client_op_rc(zb_move_abs(r->a, r->has_b ? r->b : 0)); break;
    case ZBRO_C_JOG:   s_client_op = true; client_op_rc(zb_move_rel(r->a, r->has_b ? r->b : 0)); break;
    case ZBRO_C_STOP:  reply_rc(0); zb_stop(); break;               /* OK first, then the EVT ERR 9 */
    case ZBRO_C_ESTOP: reply_rc(0); zb_fleet_note_estop(); zb_estop(); break;
    case ZBRO_C_GET: {
        float v;
        if (zb_set_get_by_key(r->key, &v)) send_line(b, zbro_fmt_val(b, sizeof(b), r->key, v));
        else send_err(ZBRO_E_ARG, "unknown key");
        break;
    }
    case ZBRO_C_SET: {
        int rc;
        if (zb_set_find(r->key) < 0) send_err(ZBRO_E_ARG, "unknown key");
        else if ((rc = zb_set_set_by_key(r->key, r->a, false)) == ZBRO_E_BUSY) send_err(ZBRO_E_BUSY, "busy");
        else if (rc != 0) send_err(ZBRO_E_ARG, "value rejected");
        else reply_rc(0);
        break;
    }
    case ZBRO_C_SAVE: reply_rc(zb_set_save() == 0 ? 0 : ZBRO_E_BUSY); break;
    case ZBRO_C_CAL:
        if (r->cal_op == ZBRO_CAL_STATUS) {
            cal_send(false);
        } else if (r->cal_op == ZBRO_CAL_ABORT) {
            zb_cal_abort();
            reply_rc(0);
        } else {
            const char *t[ZBRO_CAL_TESTS];
            for (int i = 0; i < r->ntests; i++) t[i] = r->tests[i];
            reply_rc(zb_cal_start(t, r->ntests));
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

/* The client is gone (or silent) while its move/home runs: nobody can stop it, so decelerate to a halt. */
static void stop_orphan_op(const char *why)
{
    if (!s_client_op) return;
    s_client_op = false;
    ESP_LOGW(TAG, "%s during a move: stopping", why);
    zb_stop();
}

static void drop_client(void)
{
    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
    if (s_cfd >= 0) {
        close(s_cfd);
        s_cfd = -1;
    }
    xSemaphoreGive(s_tx_mtx);
    stop_orphan_op("client disconnected");
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
                s_last_rx_us = esp_timer_get_time();
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
            if (r > 0) s_last_rx_us = esp_timer_get_time();
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

        if (cfd >= 0 && s_client_op && esp_timer_get_time() - s_last_rx_us > (int64_t)PING_TIMEOUT_MS * 1000)
            stop_orphan_op("client silent");

        /* POS reports: every rpt_hz while moving, once more when the axis comes to rest. */
        if (cfd >= 0) {
            float hz = zb_set_get(ZS_RPT_HZ);
            TickType_t period = pdMS_TO_TICKS(hz >= 1.0f ? (int)(1000.0f / hz) : 1000);
            TickType_t now = xTaskGetTickCount();
            bool moving = st.state != ZB_IDLE;
            if ((moving && now - last_rpt >= period) || (!moving && prev_state != ZB_IDLE)) {
                send_pos(moving);
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
