#include "zb_fleet.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "EspFleet.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "zb_cal.h"
#include "zb_driver.h"
#include "zb_net.h"
#include "zb_settings.h"

/* Log line format, stable and greppable: "EVT <name> key=value key=value ...", tag "zb_evt".
 *   EVT boot fw=<ver> idf=<ver> reset=<reason>
 *   EVT homed pos=<mm>
 *   EVT move_done pos=<mm> moves=<n> dist_mm=<mm>
 *   EVT err code=<n> text="<text>" pos=<mm> errors=<n>
 *   EVT estop count=<n>
 *   EVT s3_link state=up|down
 *   EVT cal_start tests=<n>            EVT cal_test name=<t> status=<s> limit=<x> unit=<u> working=<x>
 *   EVT cal_done ok=<n> failed=<n>     EVT cal_error msg="<text>"
 *   EVT fleet_cmd cmd=<cmd> rc=<n>     EVT identify host=<name> state=<state> moving=0
 * Plus the hub's own recognised lines: "WiFi connected, IP: x", "WiFi disconnected, reason: n",
 * "Booting firmware <ver>", "OTA partition marked valid", "Heap: free=n, min=n". */

#define EVT_TAG        "zb_evt"
#define TICK_MS        100
#define CONFIRM_S      90        /* mark the image valid after this long even if the hub never answered */
#define HEAP_LINE_S    300
#define EV_QUEUE_LEN   16

static const char *TAG = "zb_fleet";

struct ev { uint8_t type; int err; float pos; };

static QueueHandle_t s_evq;
static TaskHandle_t s_task;

/* Written by the motion task / any task, read by the glue task. Plain 32-bit accesses are atomic on RISC-V. */
static volatile uint32_t s_moves, s_errors, s_last_err, s_estops;
static volatile uint32_t s_wifi_reconn;
static volatile bool s_cal_dirty;
static volatile bool s_enabled = true, s_started;

/* Glue task state. */
static double s_dist_mm;
static char s_name[33];
static bool s_s3_prev;
static volatile bool s_sta_was_up;
static int s_hb_age_s = -1;
static unsigned s_hb_seen;
static int64_t s_hb_last_us;
static volatile bool s_identify_pending;

/* Calibration tracking (glue task only). */
#define CAL_TESTS 5
static bool s_cal_running;
static char s_cal_status[CAL_TESTS][10];

/* ------------------------------------------------------------ hooks --- */

void zb_fleet_on_event(enum zb_evt_type type, float pos_mm, int err)
{
    if (type == ZB_EVT_DONE) s_moves++;
    if (type == ZB_EVT_ERR) {
        s_errors++;
        s_last_err = (uint32_t)err;
    }
    if (!s_evq) return;
    struct ev e = {.type = (uint8_t)type, .err = err, .pos = pos_mm};
    xQueueSend(s_evq, &e, 0);                    /* never blocks: a full queue only loses a log line */
    if (s_task) xTaskNotifyGive(s_task);
}

void zb_fleet_on_cal(void)
{
    s_cal_dirty = true;
    if (s_task) xTaskNotifyGive(s_task);
}

void zb_fleet_note_estop(void)
{
    __atomic_fetch_add(&s_estops, 1, __ATOMIC_RELAXED);
    if (s_evq) {
        struct ev e = {.type = 0xE5, .err = 0, .pos = 0};
        xQueueSend(s_evq, &e, 0);
        if (s_task) xTaskNotifyGive(s_task);
    }
}

/* ------------------------------------------------------ agent hooks --- */

/* Called by the agent task before flash writes ("ota"), a hub reboot ("reboot") and the restart after a
 * good OTA ("restart"). Flash writes stall the cache and would disturb step generation, so an update is
 * only accepted while the axis is idle, and the driver is switched off first. */
static bool gate_cb(const char *what)
{
    struct zb_status st;
    zb_motion_status(&st);
    if (strcmp(what, "ota") == 0) {
        if (st.state != ZB_IDLE) {
            ESP_LOGW(EVT_TAG, "EVT fleet_cmd cmd=ota rc=busy state=%d", (int)st.state);
            return false;
        }
        zb_driver_enable(false);
        ESP_LOGW(EVT_TAG, "EVT fleet_cmd cmd=ota rc=0 driver=off");
        return true;
    }
    /* reboot / restart: halt whatever moves, then let go of the motor */
    if (st.state != ZB_IDLE) {
        zb_fleet_note_estop();
        zb_estop();
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    zb_driver_enable(false);
    return true;
}

static bool command_cb(const char *cmd, const char *arg)
{
    (void)arg;
    struct zb_status st;
    zb_motion_status(&st);
    int rc = -1;
    bool ok = false;

    if (strcmp(cmd, "identify") == 0) {
        /* No LED and no motion: a log line, a device-page extra, and an immediate heartbeat. */
        ESP_LOGW(EVT_TAG, "EVT identify host=%s state=%s moving=0", s_name, st.state == ZB_IDLE ? "IDLE" : "BUSY");
        s_identify_pending = true;
        ok = true;
        rc = 0;
    } else if (strcmp(cmd, "stop") == 0) {
        rc = zb_stop();
        ok = rc == 0;
    } else if (strcmp(cmd, "estop") == 0) {
        zb_fleet_note_estop();
        zb_estop();
        rc = 0;
        ok = true;
    } else if (strcmp(cmd, "home") == 0) {
        rc = (st.state == ZB_IDLE) ? zb_home() : ZB_E_BUSY;
        ok = rc == 0;
    } else if (strcmp(cmd, "calstatus") == 0) {
        s_cal_dirty = true;
        if (s_task) xTaskNotifyGive(s_task);
        rc = 0;
        ok = true;
    } else if (strcmp(cmd, "health") == 0) {
        espfleet_health_emit();
        rc = 0;
        ok = true;
    }
    if (rc >= 0) ESP_LOGI(EVT_TAG, "EVT fleet_cmd cmd=%s rc=%d", cmd, rc);
    return ok;                                   /* anything else (factory_reset, arbitrary moves): unsupported */
}

static const char *const k_task_names[] = {"zb_motion", "zb_tcp", "zb_beacon", "zb_fleet", "zb_cal", "zb_wspush",
                                           "httpd", "tiT", "sys_evt", "wifi", "mdns", "main"};

static void health_cb(char *stacks, size_t scap, char *cnt, size_t ccap)
{
    size_t u = 0;
    for (size_t i = 0; i < sizeof(k_task_names) / sizeof(k_task_names[0]); i++) {
        TaskHandle_t h = xTaskGetHandle(k_task_names[i]);
        if (!h || u + 32 >= scap) continue;
        u += (size_t)snprintf(stacks + u, scap - u, ",\"%s\":%u", k_task_names[i], (unsigned)uxTaskGetStackHighWaterMark(h));
    }
    snprintf(cnt, ccap, ",\"wifi_reconn\":%u,\"estop\":%u,\"motion_err\":%u,\"moves\":%u", (unsigned)s_wifi_reconn,
             (unsigned)s_estops, (unsigned)s_errors, (unsigned)s_moves);
}

/* ------------------------------------------------------------ wifi --- */

static void on_net_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = data;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_sta_was_up = true;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED && s_sta_was_up) {
        const wifi_event_sta_disconnected_t *ev = data;
        s_sta_was_up = false;                    /* log and count only the drop from a working link */
        s_wifi_reconn++;
        ESP_LOGW(TAG, "WiFi disconnected, reason: %d", ev ? (int)ev->reason : 0);
    }
}

/* -------------------------------------------------------- calibration --- */

static void cal_update(void)
{
    char *js = zb_cal_json();
    if (!js) return;
    cJSON *root = cJSON_Parse(js);
    free(js);
    if (!root) return;

    bool running = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(root, "running"));
    const cJSON *cur = cJSON_GetObjectItemCaseSensitive(root, "current");
    const cJSON *err = cJSON_GetObjectItemCaseSensitive(root, "error");
    const cJSON *tests = cJSON_GetObjectItemCaseSensitive(root, "tests");
    int n_ok = 0, n_fail = 0, i = 0;
    char summary[ESPFLEET_EXTRA_VALUE_LEN];
    const char *first_fail = NULL;

    if (running && !s_cal_running) {
        int nt = 0;
        const cJSON *t;
        cJSON_ArrayForEach(t, tests) {
            const cJSON *s = cJSON_GetObjectItemCaseSensitive(t, "status");
            if (cJSON_IsString(s) && strcmp(s->valuestring, "skipped") != 0) nt++;
        }
        ESP_LOGI(EVT_TAG, "EVT cal_start tests=%d", nt);
    }

    const cJSON *t;
    cJSON_ArrayForEach(t, tests) {
        if (i >= CAL_TESTS) break;
        const cJSON *name = cJSON_GetObjectItemCaseSensitive(t, "name");
        const cJSON *st = cJSON_GetObjectItemCaseSensitive(t, "status");
        const cJSON *res = cJSON_GetObjectItemCaseSensitive(t, "result");
        if (cJSON_IsString(name) && cJSON_IsString(st)) {
            if (!strcmp(st->valuestring, "ok")) n_ok++;
            else if (!strcmp(st->valuestring, "failed")) { n_fail++; if (!first_fail) first_fail = name->valuestring; }
            if (strcmp(s_cal_status[i], st->valuestring) != 0) {
                snprintf(s_cal_status[i], sizeof(s_cal_status[i]), "%s", st->valuestring);
                if (!strcmp(st->valuestring, "ok") || !strcmp(st->valuestring, "failed")) {
                    const cJSON *lim = cJSON_GetObjectItemCaseSensitive(res, "limit");
                    const cJSON *wrk = cJSON_GetObjectItemCaseSensitive(res, "working");
                    const cJSON *unit = cJSON_GetObjectItemCaseSensitive(res, "unit");
                    ESP_LOGI(EVT_TAG, "EVT cal_test name=%s status=%s limit=%.3f unit=%s working=%.3f", name->valuestring,
                             st->valuestring, cJSON_IsNumber(lim) ? lim->valuedouble : 0.0,
                             cJSON_IsString(unit) ? unit->valuestring : "-", cJSON_IsNumber(wrk) ? wrk->valuedouble : 0.0);
                }
            }
        }
        i++;
    }
    if (!running && s_cal_running) {
        if (cJSON_IsString(err) && err->valuestring[0])
            ESP_LOGW(EVT_TAG, "EVT cal_error msg=\"%s\"", err->valuestring);
        ESP_LOGI(EVT_TAG, "EVT cal_done ok=%d failed=%d", n_ok, n_fail);
    }
    s_cal_running = running;

    if (running) snprintf(summary, sizeof(summary), "running:%s", cJSON_IsString(cur) ? cur->valuestring : "");
    else if (n_fail) snprintf(summary, sizeof(summary), "failed:%s", first_fail ? first_fail : "?");
    else if (n_ok) snprintf(summary, sizeof(summary), "ok:%d", n_ok);
    else snprintf(summary, sizeof(summary), "none");
    espfleet_set_extra("calstatus", summary);
    cJSON_Delete(root);
}

/* ---------------------------------------------------------- publishing --- */

static void publish(const struct zb_status *st)
{
    float spm = zb_steps_per_mm();
    espfleet_set_metric_int("state", (int)st->state);
    espfleet_set_metric("pos_mm", st->pos);
    espfleet_set_metric_int("homed", st->homed);
    espfleet_set_metric_int("endstop", st->endstop);
    espfleet_set_metric_int("driver_en", zb_driver_is_enabled());
    espfleet_set_metric("speed_mm_s", st->speed);
    espfleet_set_metric("step_hz", fabsf(st->speed) * spm);
    espfleet_set_metric_int("s3_link", zb_net_s3_connected());
    espfleet_set_metric_int("tcp_clients", zb_net_s3_connected() ? 1 : 0);
    espfleet_set_metric_int("motion_err", s_errors);
    espfleet_set_metric_int("last_err", s_last_err);
    espfleet_set_metric_int("estop", s_estops);
    espfleet_set_metric_int("moves", s_moves);
    espfleet_set_metric("dist_mm", s_dist_mm);
    espfleet_set_metric("es_repeat_um", zb_set_get(ZS_ES_REPEAT_UM));
    espfleet_set_metric("lim_speed", zb_set_get(ZS_LIM_SPEED));
    espfleet_set_metric("lim_accel", zb_set_get(ZS_LIM_ACCEL));
    espfleet_set_metric("lim_decel", zb_set_get(ZS_LIM_DECEL));
    espfleet_set_metric("lim_home_fast", zb_set_get(ZS_LIM_HOME_FAST));
    espfleet_set_metric("cal_time", zb_set_get(ZS_CAL_TIME));
    espfleet_set_metric_int("wifi_reconn", s_wifi_reconn);
    espfleet_set_metric_int("cal_running", s_cal_running);

    char v[ESPFLEET_EXTRA_VALUE_LEN];
    espfleet_set_extra("fw", ZB_FW_VERSION);
    snprintf(v, sizeof(v), "%d", zb_driver_microstep());
    espfleet_set_extra("microstep", v);
    snprintf(v, sizeof(v), "%.3f", (double)spm);
    espfleet_set_extra("steps_mm", v);
}

static void heartbeat_age_update(int64_t now_us)
{
    unsigned hb = espfleet_heartbeat_count();
    if (hb != s_hb_seen) {
        s_hb_seen = hb;
        s_hb_last_us = now_us;
    }
    s_hb_age_s = s_hb_seen ? (int)((now_us - s_hb_last_us) / 1000000) : -1;
}

static const char *reset_name(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "int_wdt";
    case ESP_RST_TASK_WDT:  return "task_wdt";
    case ESP_RST_WDT:       return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    default:                return "unknown";
    }
}

static void agent_start(void)
{
    char hub[128];
    zb_fleet_hub_get(hub, sizeof(hub));
    if (strcmp(hub, "off") == 0) {
        s_enabled = false;
        ESP_LOGI(TAG, "fleet agent disabled by the hub setting");
        return;
    }
    zb_hostname_get(s_name, sizeof(s_name));

    espfleet_config_t cfg;
    espfleet_config_default(&cfg);
    cfg.hub = hub;                               /* "" = discover via mDNS _espfleet._tcp */
    cfg.name = s_name;
    cfg.project = NULL;                          /* build-time slug from the git origin, else auto */
    cfg.web_port = 80;
    cfg.web_path = "/";
    cfg.enable_ota = true;
    cfg.enable_mdns = true;
    cfg.spawn_task = true;
    espfleet_on_command(command_cb);
    espfleet_set_gate(gate_cb);
    espfleet_set_health_cb(health_cb);
    if (espfleet_start(&cfg) != 0) {
        ESP_LOGW(TAG, "fleet agent did not start");
        return;
    }
    s_started = true;
    const esp_app_desc_t *d = esp_app_get_description();
    ESP_LOGI(TAG, "Booting firmware %s", d ? d->version : ZB_FW_VERSION);
    ESP_LOGI(EVT_TAG, "EVT boot fw=%s idf=%s reset=%s", ZB_FW_VERSION, esp_get_idf_version(), reset_name());
}

/* --------------------------------------------------------------- task --- */

static void fleet_task(void *arg)
{
    (void)arg;
    bool decided = false, confirmed = false;
    bool have_last = false, last_homed = false;
    float last_pos = 0;
    int tick = 0;
    int64_t t0 = esp_timer_get_time();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TICK_MS));
        tick++;
        int64_t now = esp_timer_get_time();
        struct zb_status st;
        zb_motion_status(&st);

        struct ev e;
        while (xQueueReceive(s_evq, &e, 0) == pdTRUE) {
            switch (e.type) {
            case ZB_EVT_HOMED:
                ESP_LOGI(EVT_TAG, "EVT homed pos=%.3f", (double)e.pos);
                break;
            case ZB_EVT_DONE:
                ESP_LOGI(EVT_TAG, "EVT move_done pos=%.3f moves=%u dist_mm=%.1f", (double)e.pos, (unsigned)s_moves, s_dist_mm);
                break;
            case ZB_EVT_ERR:
                ESP_LOGW(EVT_TAG, "EVT err code=%d text=\"%s\" pos=%.3f errors=%u", e.err, zb_err_text(e.err), (double)e.pos,
                         (unsigned)s_errors);
                break;
            case 0xE5:
                ESP_LOGW(EVT_TAG, "EVT estop count=%u", (unsigned)s_estops);
                break;
            }
        }

        /* Travelled distance, sampled: a jump of the zero at homing or an unhomed/homed flip is not travel. */
        if (have_last && st.homed == last_homed) {
            float d = fabsf(st.pos - last_pos);
            if (d > 0.0f && d < 20.0f) s_dist_mm += d;
        }
        have_last = true;
        last_pos = st.pos;
        last_homed = st.homed;

        bool s3 = zb_net_s3_connected();
        if (s3 != s_s3_prev) {
            s_s3_prev = s3;
            ESP_LOGI(EVT_TAG, "EVT s3_link state=%s", s3 ? "up" : "down");
        }

        if (s_cal_dirty) {
            s_cal_dirty = false;
            cal_update();
        }

        if (!decided && zb_net_sta_connected()) {     /* never in SoftAP-only provisioning mode */
            decided = true;
            agent_start();
        }

        if (!confirmed && (now - t0) > (int64_t)CONFIRM_S * 1000000 && (zb_net_sta_connected() || zb_net_ap_active())) {
            confirmed = true;                         /* the lib confirms on the first hub heartbeat; this covers no hub */
            if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) ESP_LOGI(TAG, "OTA partition marked valid");
        }

        if (!s_started) continue;
        heartbeat_age_update(now);

        if (s_identify_pending) {
            s_identify_pending = false;
            char v[ESPFLEET_EXTRA_VALUE_LEN];
            snprintf(v, sizeof(v), "%lds idle=%d", (long)(now / 1000000), st.state == ZB_IDLE);
            espfleet_set_extra("identify", v);
            espfleet_poke();
        }

        if (tick % (1000 / TICK_MS) == 0) {
            char host[33];
            zb_hostname_get(host, sizeof(host));
            if (strcmp(host, s_name) != 0) {
                snprintf(s_name, sizeof(s_name), "%s", host);
                espfleet_set_name(s_name);
            }
            publish(&st);
        }
        if (tick % (HEAP_LINE_S * 1000 / TICK_MS) == 0)
            ESP_LOGI(TAG, "Heap: free=%u, min=%u", (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());
    }
}

void zb_fleet_start(void)
{
    s_evq = xQueueCreate(EV_QUEUE_LEN, sizeof(struct ev));
    if (!s_evq) return;
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, on_net_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_net_event, NULL);
    xTaskCreate(fleet_task, "zb_fleet", 4608, NULL, 1, &s_task);
}

void zb_fleet_status(struct zb_fleet_status *st, char *hub, size_t hub_n)
{
    st->enabled = s_enabled;
    st->started = s_started;
    st->heartbeats = s_started ? espfleet_heartbeat_count() : 0;
    st->registered = s_started && espfleet_online();
    st->last_hb_age_s = s_started ? s_hb_age_s : -1;
    if (!hub || !hub_n) return;
    if (!s_enabled) snprintf(hub, hub_n, "off");
    else if (s_started) snprintf(hub, hub_n, "%s", espfleet_hub_url());
    else hub[0] = '\0';
}
