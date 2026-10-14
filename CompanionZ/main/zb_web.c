#include "zb_web.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "zb_cal.h"
#include "zb_fleet.h"
#include "zb_net.h"
#include "zb_settings.h"
#include "zbro_proto.h"

#define WWW_BASE   "/www"
#define BODY_MAX   1024
#define WS_MAX     8

static const char *TAG = "zb_web";
static httpd_handle_t s_srv;
static SemaphoreHandle_t s_ws_mtx;
static TaskHandle_t s_push_task;

/* --------------------------------------------------------- helpers ---- */

static esp_err_t send_json(httpd_req_t *req, const char *status, const char *body)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (status) httpd_resp_set_status(req, status);
    return httpd_resp_sendstr(req, body);
}

static esp_err_t send_error(httpd_req_t *req, const char *status, const char *text, const char *key, int code)
{
    char b[160];
    if (key) snprintf(b, sizeof(b), "{\"error\":\"%s\",\"key\":\"%s\",\"code\":%d}", text, key, code);
    else snprintf(b, sizeof(b), "{\"error\":\"%s\",\"code\":%d}", text, code);
    return send_json(req, status, b);
}

static esp_err_t send_rc(httpd_req_t *req, int rc)
{
    if (rc == 0) return send_json(req, NULL, "{\"ok\":true}");
    return send_error(req, rc == ZBRO_E_BUSY ? "409 Conflict" : "400 Bad Request", zb_err_text(rc), NULL, rc);
}

/* Reads the body into a malloc'd, NUL-terminated buffer. NULL on error or when empty and !allow_empty. */
static char *read_body(httpd_req_t *req)
{
    size_t len = req->content_len;
    if (len > BODY_MAX) return NULL;
    char *b = malloc(len + 1);
    if (!b) return NULL;
    size_t got = 0;
    while (got < len) {
        int r = httpd_req_recv(req, b + got, len - got);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
            free(b);
            return NULL;
        }
        got += (size_t)r;
    }
    b[len] = '\0';
    return b;
}

/* Parses the request body as a JSON object; replies 400 and returns NULL on failure. */
static cJSON *body_json(httpd_req_t *req, bool allow_empty)
{
    char *b = read_body(req);
    cJSON *j = NULL;
    if (b) {
        if (b[0] == '\0' && allow_empty) j = cJSON_CreateObject();
        else j = cJSON_Parse(b);
        free(b);
    }
    if (!j || !cJSON_IsObject(j)) {
        cJSON_Delete(j);
        send_error(req, "400 Bad Request", "bad json", NULL, ZBRO_E_ARG);
        return NULL;
    }
    return j;
}

static bool get_num(const cJSON *j, const char *k, float *out)
{
    const cJSON *i = cJSON_GetObjectItemCaseSensitive(j, k);
    if (!cJSON_IsNumber(i)) return false;
    *out = (float)i->valuedouble;
    return true;
}

static int status_json(char *b, size_t n)
{
    struct zb_status s;
    zb_motion_status(&s);
    return snprintf(b, n,
        "{\"pos\":%.3f,\"state\":\"%s\",\"homed\":%s,\"endstop\":%s,\"fw\":\"%s\",\"rssi\":%d,\"s3\":%s,"
        "\"uptime\":%lld}",
        (double)s.pos, zbro_state_name((int)s.state), s.homed ? "true" : "false", s.endstop ? "true" : "false",
        ZB_FW_VERSION, zb_net_rssi(), zb_net_s3_connected() ? "true" : "false",
        (long long)(esp_timer_get_time() / 1000000));
}

/* --------------------------------------------------------- websocket -- */

static void ws_broadcast(const char *s, size_t n)
{
    if (!s_srv) return;
    int fds[WS_MAX];
    size_t cnt = WS_MAX;
    xSemaphoreTake(s_ws_mtx, portMAX_DELAY);
    if (httpd_get_client_list(s_srv, &cnt, fds) == ESP_OK) {
        httpd_ws_frame_t f = {.type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t *)s, .len = n};
        for (size_t i = 0; i < cnt; i++)
            if (httpd_ws_get_fd_info(s_srv, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET)
                httpd_ws_send_frame_async(s_srv, fds[i], &f);
    }
    xSemaphoreGive(s_ws_mtx);
}

static bool ws_has_clients(void)
{
    int fds[WS_MAX];
    size_t cnt = WS_MAX;
    if (!s_srv || httpd_get_client_list(s_srv, &cnt, fds) != ESP_OK) return false;
    for (size_t i = 0; i < cnt; i++)
        if (httpd_ws_get_fd_info(s_srv, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) return true;
    return false;
}

static void push_status(void)
{
    char b[256];
    char *p = b;
    int n = status_json(b + 0, sizeof(b));
    if (n <= 0 || n >= (int)sizeof(b)) return;
    /* splice {"t":"status", in front of the object body */
    char o[288];
    int m = snprintf(o, sizeof(o), "{\"t\":\"status\",%s", p + 1);
    if (m > 0 && m < (int)sizeof(o)) ws_broadcast(o, (size_t)m);
}

static void push_task(void *arg)
{
    (void)arg;
    for (;;) {
        struct zb_status s;
        zb_motion_status(&s);
        float hz = zb_set_get(ZS_RPT_HZ);
        TickType_t period = s.state == ZB_IDLE ? pdMS_TO_TICKS(1000) : pdMS_TO_TICKS(hz >= 1.0f ? (int)(1000.0f / hz) : 1000);
        ulTaskNotifyTake(pdTRUE, period);               /* woken early by events */
        if (ws_has_clients()) push_status();
    }
}

void zb_web_on_event(enum zb_evt_type type, float pos_mm, int err)
{
    (void)type; (void)pos_mm; (void)err;
    if (s_push_task) xTaskNotifyGive(s_push_task);
}

void zb_web_on_cal(void)
{
    if (!s_srv || !ws_has_clients()) return;
    char *json = zb_cal_json();
    if (!json) return;
    size_t sz = strlen(json) + 16;
    char *o = malloc(sz);
    if (o && json[0] == '{') {
        int n = snprintf(o, sz, "{\"t\":\"cal\",%s", json + 1);
        if (n > 0 && (size_t)n < sz) ws_broadcast(o, (size_t)n);
    }
    free(o);
    free(json);
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) return ESP_OK;          /* handshake done by the server */
    uint8_t buf[64];
    httpd_ws_frame_t f = {.payload = buf, .type = HTTPD_WS_TYPE_TEXT};
    esp_err_t e = httpd_ws_recv_frame(req, &f, 0);        /* length only */
    if (e != ESP_OK) return e;
    if (f.len > 0 && f.len <= sizeof(buf)) return httpd_ws_recv_frame(req, &f, sizeof(buf));
    if (f.len > sizeof(buf)) return ESP_FAIL;             /* we never expect client data: drop the socket */
    return ESP_OK;
}

/* ------------------------------------------------------------- REST --- */

/* "fleet":{...} member for /api/status and /api/wifi (hub URL is validated at write: no quotes/backslashes). */
static int fleet_json(char *b, size_t n)
{
    struct zb_fleet_status f;
    char hub[128];
    zb_fleet_status(&f, hub, sizeof(hub));
    return snprintf(b, n, "\"fleet\":{\"enabled\":%s,\"registered\":%s,\"hb_age\":%d,\"hub\":\"%s\"}",
                    f.enabled ? "true" : "false", f.registered ? "true" : "false", f.last_hb_age_s, hub);
}

static esp_err_t h_status(httpd_req_t *req)
{
    char b[512];
    int n = status_json(b, sizeof(b));
    char f[300];
    int m = fleet_json(f, sizeof(f));
    if (n > 1 && n < (int)sizeof(b) && m > 0 && m < (int)sizeof(f) && n + m + 2 < (int)sizeof(b))
        snprintf(b + n - 1, sizeof(b) - (size_t)n + 1, ",%s}", f);   /* replaces the closing brace */
    return send_json(req, NULL, b);
}

static esp_err_t h_settings_get(httpd_req_t *req)
{
    char *j = zb_set_to_json();
    if (!j) return send_error(req, "500 Internal Server Error", "no memory", NULL, ZBRO_E_FAULT);
    esp_err_t e = send_json(req, NULL, j);
    free(j);
    return e;
}

static esp_err_t h_settings_put(httpd_req_t *req)
{
    cJSON *j = body_json(req, false);
    if (!j) return ESP_OK;
    esp_err_t e = ESP_OK;
    const char *bad = NULL;
    for (cJSON *i = j->child; i && !bad; i = i->next) {
        const char *k = i->string ? i->string : "";
        if (strcmp(k, "hostname") == 0 && cJSON_IsString(i)) {
            if (zb_hostname_set(i->valuestring) != 0) bad = k;
        } else if (zb_set_find(k) < 0 || !(cJSON_IsNumber(i) || cJSON_IsBool(i))) {
            bad = k;
        } else {
            float v = cJSON_IsBool(i) ? (cJSON_IsTrue(i) ? 1.0f : 0.0f) : (float)i->valuedouble;
            if (zb_set_set_by_key(k, v, false) != 0) bad = k;
        }
        if (bad) {
            char key[ZBRO_KEY_MAX + 1];
            snprintf(key, sizeof(key), "%.24s", bad);
            for (char *c = key; *c; c++)
                if (*c == '"' || *c == '\\' || (unsigned char)*c < 0x20) *c = '_';
            e = send_error(req, "400 Bad Request", "invalid", key, ZBRO_E_ARG);
        }
    }
    zb_set_save();
    cJSON_Delete(j);
    return bad ? e : send_json(req, NULL, "{\"ok\":true}");
}

static esp_err_t h_move(httpd_req_t *req)
{
    cJSON *j = body_json(req, false);
    if (!j) return ESP_OK;
    float z, f = 0;
    esp_err_t e;
    if (!get_num(j, "z", &z) || (cJSON_HasObjectItem(j, "f") && !get_num(j, "f", &f)) || f < 0)
        e = send_rc(req, ZBRO_E_ARG);
    else
        e = send_rc(req, zb_move_abs(z, f));
    cJSON_Delete(j);
    return e;
}

static esp_err_t h_jog(httpd_req_t *req)
{
    cJSON *j = body_json(req, false);
    if (!j) return ESP_OK;
    float dz, f = 0;
    esp_err_t e;
    if (!get_num(j, "dz", &dz) || (cJSON_HasObjectItem(j, "f") && !get_num(j, "f", &f)) || f < 0)
        e = send_rc(req, ZBRO_E_ARG);
    else
        e = send_rc(req, zb_move_rel(dz, f));
    cJSON_Delete(j);
    return e;
}

static esp_err_t h_home(httpd_req_t *req) { return send_rc(req, zb_home()); }
static esp_err_t h_stop(httpd_req_t *req) { return send_rc(req, zb_stop()); }
static esp_err_t h_estop(httpd_req_t *req) { zb_fleet_note_estop(); zb_estop(); return send_rc(req, 0); }

static esp_err_t h_cal_get(httpd_req_t *req)
{
    char *j = zb_cal_json();
    if (!j) return send_error(req, "500 Internal Server Error", "no memory", NULL, ZBRO_E_FAULT);
    esp_err_t e = send_json(req, NULL, j);
    free(j);
    return e;
}

static esp_err_t h_cal_start(httpd_req_t *req)
{
    cJSON *j = body_json(req, true);
    if (!j) return ESP_OK;
    const char *t[ZBRO_CAL_TESTS];
    int n = 0;
    bool ok = true;
    const cJSON *arr = cJSON_GetObjectItemCaseSensitive(j, "tests");
    if (arr) {
        ok = cJSON_IsArray(arr);
        const cJSON *i;
        cJSON_ArrayForEach(i, arr) {
            if (!cJSON_IsString(i) || n >= ZBRO_CAL_TESTS) { ok = false; break; }
            t[n++] = i->valuestring;
        }
    }
    esp_err_t e;
    if (!ok) e = send_rc(req, ZBRO_E_ARG);
    else if (!zb_cal_start(n ? t : NULL, n)) e = send_error(req, "409 Conflict", "cannot start", NULL, ZBRO_E_BUSY);
    else e = send_rc(req, 0);
    cJSON_Delete(j);
    return e;
}

static esp_err_t h_cal_abort(httpd_req_t *req)
{
    zb_cal_abort();
    return send_rc(req, 0);
}

static esp_err_t h_wifi_get(httpd_req_t *req)
{
    char ssid[40], ip[20], b[560], hub[128], f[300];
    zb_net_wifi_ssid(ssid, sizeof(ssid));
    zb_fleet_hub_get(hub, sizeof(hub));
    fleet_json(f, sizeof(f));
    zb_net_ip(ip, sizeof(ip));
    for (char *c = ssid; *c; c++)
        if (*c == '"' || *c == '\\' || (unsigned char)*c < 0x20) *c = '_';
    snprintf(b, sizeof(b), "{\"ssid\":\"%s\",\"ip\":\"%s\",\"connected\":%s,\"ap\":%s,\"rssi\":%d,\"fleet_hub\":\"%s\",%s}",
             ssid, ip, zb_net_sta_connected() ? "true" : "false", zb_net_ap_active() ? "true" : "false", zb_net_rssi(),
             hub, f);
    return send_json(req, NULL, b);
}

static esp_err_t h_wifi_put(httpd_req_t *req)
{
    cJSON *j = body_json(req, false);
    if (!j) return ESP_OK;
    const cJSON *s = cJSON_GetObjectItemCaseSensitive(j, "ssid");
    const cJSON *p = cJSON_GetObjectItemCaseSensitive(j, "pass");
    const cJSON *fh = cJSON_GetObjectItemCaseSensitive(j, "fleet_hub");
    esp_err_t e;
    if (fh) {                                    /* ESPFleet hub URL: "", "off" or http(s)://...; applied on next boot */
        if (!cJSON_IsString(fh) || zb_fleet_hub_set(fh->valuestring) != 0) {
            e = send_rc(req, ZBRO_E_ARG);
            cJSON_Delete(j);
            return e;
        }
        if (!s) {
            cJSON_Delete(j);
            return send_json(req, NULL, "{\"ok\":true,\"reboot_needed\":true}");
        }
    }
    if (!cJSON_IsString(s) || (p && !cJSON_IsString(p))) {
        e = send_rc(req, ZBRO_E_ARG);
    } else {
        int rc = zb_net_wifi_set(s->valuestring, p ? p->valuestring : "");
        if (rc != 0) {
            e = send_rc(req, rc);
        } else {
            e = send_json(req, NULL, "{\"ok\":true,\"reboot\":true}");
            zb_net_reboot_later(800);
        }
    }
    cJSON_Delete(j);
    return e;
}

static esp_err_t h_reboot(httpd_req_t *req)
{
    esp_err_t e = send_rc(req, 0);
    zb_net_reboot_later(500);
    return e;
}

/* ------------------------------------------------------ static files -- */

static const char *mime_for(const char *path)
{
    const char *dot = strrchr(path, '.');
    if (!dot) return "application/octet-stream";
    static const struct { const char *ext, *mime; } t[] = {
        {".html", "text/html; charset=utf-8"}, {".js", "text/javascript"}, {".mjs", "text/javascript"},
        {".css", "text/css"}, {".json", "application/json"}, {".svg", "image/svg+xml"},
        {".png", "image/png"}, {".ico", "image/x-icon"}, {".woff2", "font/woff2"}, {".woff", "font/woff"},
        {".txt", "text/plain"}, {".webmanifest", "application/manifest+json"},
    };
    for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++)
        if (strcmp(dot, t[i].ext) == 0) return t[i].mime;
    return "application/octet-stream";
}

static bool file_exists(const char *p)
{
    struct stat st;
    return stat(p, &st) == 0 && S_ISREG(st.st_mode);
}

static esp_err_t send_file(httpd_req_t *req, const char *rel)
{
    char path[96];
    bool gz = false;
    snprintf(path, sizeof(path), WWW_BASE "/%s.gz", rel);
    if (file_exists(path)) {
        gz = true;
    } else {
        snprintf(path, sizeof(path), WWW_BASE "/%s", rel);
        if (!file_exists(path)) return ESP_ERR_NOT_FOUND;
    }
    FILE *f = fopen(path, "rb");
    if (!f) return ESP_ERR_NOT_FOUND;
    httpd_resp_set_type(req, mime_for(rel));
    if (gz) httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", strncmp(rel, "assets/", 7) == 0 ? "public, max-age=31536000, immutable" : "no-cache");
    char *buf = malloc(1024);
    if (!buf) { fclose(f); return httpd_resp_send_500(req); }
    esp_err_t e = ESP_OK;
    size_t n;
    while ((n = fread(buf, 1, 1024, f)) > 0) {
        e = httpd_resp_send_chunk(req, buf, (ssize_t)n);
        if (e != ESP_OK) break;
    }
    if (e == ESP_OK) e = httpd_resp_send_chunk(req, NULL, 0);
    free(buf);
    fclose(f);
    return e;
}

static esp_err_t h_static(httpd_req_t *req)
{
    char rel[64];
    const char *u = req->uri;
    if (*u == '/') u++;
    size_t n = strcspn(u, "?#");
    if (n >= sizeof(rel) || strstr(u, "..") != NULL) return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, NULL);
    memcpy(rel, u, n);
    rel[n] = '\0';
    if (strncmp(rel, "api/", 4) == 0 || strcmp(rel, "ws") == 0) return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, NULL);
    if (rel[0] == '\0') snprintf(rel, sizeof(rel), "index.html");
    esp_err_t e = send_file(req, rel);
    if (e == ESP_ERR_NOT_FOUND) {
        if (strchr(rel, '.') == NULL || strstr(rel, "..")) e = send_file(req, "index.html");   /* SPA route */
        if (e == ESP_ERR_NOT_FOUND) return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "UI not installed");
    }
    return e;
}

/* ------------------------------------------------------------ start --- */

void zb_web_start(void)
{
    s_ws_mtx = xSemaphoreCreateMutex();

    esp_vfs_littlefs_conf_t fs = {
        .base_path = WWW_BASE, .partition_label = "storage", .format_if_mount_failed = false,
    };
    if (esp_vfs_littlefs_register(&fs) != ESP_OK) ESP_LOGW(TAG, "LittleFS not mounted: UI unavailable");

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 24;
    cfg.max_open_sockets = 6;
    cfg.stack_size = 6144;
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    if (httpd_start(&s_srv, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        s_srv = NULL;
        return;
    }

    static const struct { const char *uri; httpd_method_t m; esp_err_t (*h)(httpd_req_t *); } routes[] = {
        {"/api/status", HTTP_GET, h_status},
        {"/api/settings", HTTP_GET, h_settings_get},
        {"/api/settings", HTTP_PUT, h_settings_put},
        {"/api/move", HTTP_POST, h_move},
        {"/api/jog", HTTP_POST, h_jog},
        {"/api/home", HTTP_POST, h_home},
        {"/api/stop", HTTP_POST, h_stop},
        {"/api/estop", HTTP_POST, h_estop},
        {"/api/cal", HTTP_GET, h_cal_get},
        {"/api/cal/start", HTTP_POST, h_cal_start},
        {"/api/cal/abort", HTTP_POST, h_cal_abort},
        {"/api/wifi", HTTP_GET, h_wifi_get},
        {"/api/wifi", HTTP_PUT, h_wifi_put},
        {"/api/reboot", HTTP_POST, h_reboot},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_uri_t u = {.uri = routes[i].uri, .method = routes[i].m, .handler = routes[i].h};
        httpd_register_uri_handler(s_srv, &u);
    }
    httpd_uri_t ws = {.uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .is_websocket = true};
    httpd_register_uri_handler(s_srv, &ws);
    httpd_uri_t st = {.uri = "/*", .method = HTTP_GET, .handler = h_static};   /* last: catch-all */
    httpd_register_uri_handler(s_srv, &st);

    xTaskCreate(push_task, "zb_wspush", 3584, NULL, 3, &s_push_task);
}
