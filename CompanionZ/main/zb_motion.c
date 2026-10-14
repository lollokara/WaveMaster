#include "zb_motion.h"

#include <math.h>
#include <string.h>

#include "driver/rmt_tx.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "zb_driver.h"
#include "zb_endstop.h"
#include "zb_pins.h"
#include "zb_settings.h"

#include "zb_math.c"   /* pure step generator, also built by the host test */

/* RMT TX trapezoid generator (modelled on the ESP-IDF peripherals/rmt/stepper_motor example): every
 * STEP is one RMT symbol (high pulse_us, low period-pulse_us at 1 MHz). Moves are cut into chunks of a
 * few ms that are queued 3 deep, so the position is known to one chunk during motion and exact at
 * standstill (the C3 has no PCNT; chunk completion callbacks count the steps). */

#define RING        4
#define INFLIGHT    3
#define CHUNK_MAX   256
#define RMT_RES_HZ  1000000

enum cmd_type { C_HOME, C_MOVE, C_NOP };
struct cmd { enum cmd_type type; int32_t delta; float speed; };

struct run {
    int32_t total;
    int dir;                 /* +1 / -1 in position space */
    float vs, vc, a, d;      /* steps/s, steps/s^2 */
    int pulse;
    bool es_stop_now;        /* abrupt stop on confirmed endstop (probe) */
    bool es_decel;           /* decelerated stop on confirmed endstop (homing approach, cal) */
    bool es_guard;           /* confirmed endstop in a normal move: stop and report ZB_E_ENDSTOP */
};

struct run_res {
    int err;                 /* ZB_E_* */
    bool stopped;            /* STOP / calibration abort consumed */
    bool halted;             /* ESTOP */
    bool es_hit;
    int32_t es_pos;          /* step counter snapshot at the endstop edge */
};

static zb_event_cb s_cb;
static QueueHandle_t s_q;
static SemaphoreHandle_t s_lock;          /* command acceptance / state transitions */
static SemaphoreHandle_t s_done_sem;
static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_enc;
static rmt_symbol_word_t s_buf[RING][CHUNK_MAX];
static int32_t s_chunk_n[RING];

static volatile int32_t s_pos_steps;      /* updated by the RMT done callback */
static volatile int s_dir_sign = 1;
static volatile uint32_t s_cb_idx;        /* chunks completed (ISR) */
static volatile enum zb_state s_state = ZB_IDLE;
static volatile bool s_homed;
static volatile float s_speed;            /* mm/s of the last generated step */
static volatile bool s_stop_req, s_estop, s_cal_abort;
static float s_homed_spm;
static int64_t s_last_active_us;

static bool IRAM_ATTR on_chunk_done(rmt_channel_handle_t ch, const rmt_tx_done_event_data_t *ed, void *u)
{
    (void)ch; (void)ed; (void)u;
    int32_t n = s_chunk_n[s_cb_idx % RING];
    s_pos_steps = s_pos_steps + s_dir_sign * n;
    s_cb_idx = s_cb_idx + 1;
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_done_sem, &hp);
    return hp == pdTRUE;
}

static bool chan_create(void)
{
    rmt_tx_channel_config_t cfg = {
        .gpio_num = ZB_PIN_STEP,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RES_HZ,
        .mem_block_symbols = 48,
        .trans_queue_depth = RING,
    };
    rmt_copy_encoder_config_t ec = {};
    rmt_tx_event_callbacks_t cbs = { .on_trans_done = on_chunk_done };
    s_chan = NULL; s_enc = NULL;
    if (rmt_new_tx_channel(&cfg, &s_chan) != ESP_OK) { s_chan = NULL; return false; }
    if (rmt_new_copy_encoder(&ec, &s_enc) != ESP_OK ||
        rmt_tx_register_event_callbacks(s_chan, &cbs, NULL) != ESP_OK ||
        rmt_enable(s_chan) != ESP_OK) {
        rmt_del_channel(s_chan);
        s_chan = NULL; s_enc = NULL;
        return false;
    }
    return true;
}

/* After ESTOP / stall: queued transactions would restart on re-enable, so rebuild the channel. */
static void chan_reset(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);   /* waits for zb_estop() to finish its rmt_disable */
    if (s_chan) {
        rmt_disable(s_chan);
        if (s_enc) rmt_del_encoder(s_enc);
        rmt_del_channel(s_chan);
    }
    s_chan = NULL; s_enc = NULL;
    chan_create();
    xSemaphoreGive(s_lock);
    s_cb_idx = 0;
}

static float spm_now(void) { return zb_steps_per_mm(); }

static int es_dir(void) { return zb_set_get(ZS_HOME_TO_MAX) != 0.0f ? 1 : -1; }

static void check_homed_scale(void)
{
    if (s_homed && fabsf(spm_now() - s_homed_spm) > 1e-3f) s_homed = false;   /* steps/mm changed */
}

static void emit(enum zb_evt_type t, int err)
{
    if (s_cb) s_cb(t, zb_steps_to_mm(s_pos_steps, spm_now()), err);
}

/* Runs one profile to completion (or stop). Called by the motion task or, in ZB_CAL, the cal task. */
static void run_profile(const struct run *r, struct run_res *res)
{
    memset(res, 0, sizeof *res);
    if (r->total <= 0 && !s_estop) return;
    if (!s_chan) { res->err = ZB_E_FAULT; return; }

    struct zb_gen g;
    zb_gen_init(&g, r->total, r->vs, r->vc, r->a, r->d);
    bool watch = r->es_stop_now || r->es_decel || r->es_guard;
    bool stopping = false, faulted = false;
    uint32_t issued = 0;
    const rmt_transmit_config_t tc = { .loop_count = 0 };
    const float chunk_s = r->es_stop_now ? 0.002f : 0.004f;

    s_dir_sign = r->dir;
    s_cb_idx = 0;
    while (xSemaphoreTake(s_done_sem, 0) == pdTRUE) {}
    zb_driver_set_dir(r->dir > 0);
    zb_driver_enable(true);
    if (watch) zb_endstop_rearm();

    int64_t last_prog = esp_timer_get_time();
    uint32_t seen = 0;

    for (;;) {
        if (s_estop) { res->halted = true; break; }
        if (!stopping) {
            if (s_stop_req || (s_state == ZB_CAL && s_cal_abort)) {
                stopping = true; res->stopped = true;
                zb_gen_truncate_to_stop(&g);
            } else if (watch && zb_endstop_poll(&res->es_pos)) {
                res->es_hit = true; stopping = true;
                if (r->es_stop_now) g.n = g.i;
                else {
                    zb_gen_truncate_to_stop(&g);
                    if (r->es_guard) res->err = ZB_E_ENDSTOP;
                }
            }
        }
        while (!faulted && g.i < g.n && (int32_t)(issued - s_cb_idx) < INFLIGHT) {
            float rate = g.v;
            int32_t n = (int32_t)(rate * chunk_s);
            if (n < 1) n = 1;
            if (n > CHUNK_MAX) n = CHUNK_MAX;
            if (n > g.n - g.i) n = g.n - g.i;
            rmt_symbol_word_t *b = s_buf[issued % RING];
            for (int32_t k = 0; k < n; k++) {
                rate = zb_gen_next(&g);
                uint32_t t = zb_period_ticks(rate, r->pulse);
                b[k].level0 = 1; b[k].duration0 = (uint32_t)r->pulse;
                b[k].level1 = 0; b[k].duration1 = t - (uint32_t)r->pulse;
            }
            s_chunk_n[issued % RING] = n;
            s_speed = rate / spm_now();
            if (rmt_transmit(s_chan, s_enc, b, (size_t)n * sizeof(rmt_symbol_word_t), &tc) != ESP_OK) {
                faulted = true; res->err = ZB_E_FAULT;
                g.n = g.i - n;   /* the rejected chunk never ran: don't count it as issued */
                break;
            }
            issued++;
        }
        uint32_t done = s_cb_idx;
        if (done != seen) { seen = done; last_prog = esp_timer_get_time(); }
        if (done == issued && (g.i >= g.n || faulted)) break;
        if (xSemaphoreTake(s_done_sem, pdMS_TO_TICKS(10)) != pdTRUE &&
            esp_timer_get_time() - last_prog > 500000) {
            res->err = ZB_E_FAULT;   /* no chunk finished for 0.5 s: hardware stuck */
            res->halted = true;
            break;
        }
    }
    s_speed = 0;
    if (res->halted) chan_reset();
    s_last_active_us = esp_timer_get_time();
}

static void set_state(enum zb_state st)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state = st;
    xSemaphoreGive(s_lock);
}

/* End of every command: clear request flags, go idle, then tell the world. */
static void finish_idle(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_stop_req = false;
    s_estop = false;
    if (s_state != ZB_FAULT) s_state = ZB_IDLE;
    xSemaphoreGive(s_lock);
}

static void mk_run(struct run *r, int32_t total, int dir, float speed, float accel, float decel)
{
    float spm = spm_now();
    r->pulse = (int)zb_set_get(ZS_PULSE_US);
    r->total = total < 0 ? -total : total;
    r->dir = dir;
    r->vc = zb_rate_clamp(speed * spm, r->pulse);
    r->vs = r->vc < 200.0f ? r->vc : 200.0f;
    r->a = accel * spm;
    r->d = decel * spm;
    r->es_stop_now = r->es_decel = r->es_guard = false;
}

static void sync_driver_microstep(void)
{
    int ms = (int)zb_set_get(ZS_MICROSTEP);
    if (ms != zb_driver_microstep()) zb_driver_set_microstep(ms);
}

/* Fault or ESTOP outcome shared by exec_*; returns true if the command is over. */
static bool handle_abnormal(const struct run_res *res)
{
    if (res->halted) {
        s_homed = false;
        finish_idle();
        if (res->err) { set_state(ZB_FAULT); emit(ZB_EVT_ERR, ZB_E_FAULT); }
        else emit(ZB_EVT_DONE, 0);
        return true;
    }
    if (res->err == ZB_E_FAULT) {
        s_homed = false;
        set_state(ZB_FAULT);
        emit(ZB_EVT_ERR, ZB_E_FAULT);
        return true;
    }
    return false;
}

static void exec_move(int32_t delta, float speed)
{
    sync_driver_microstep();
    check_homed_scale();
    struct run r;
    struct run_res res;
    mk_run(&r, delta, delta < 0 ? -1 : 1, speed, zb_set_get(ZS_ACCEL), zb_set_get(ZS_DECEL));
    r.es_guard = r.dir == es_dir();
    run_profile(&r, &res);
    if (handle_abnormal(&res)) return;
    if (res.err == ZB_E_ENDSTOP) {
        s_homed = false;
        finish_idle();
        emit(ZB_EVT_ERR, ZB_E_ENDSTOP);
        return;
    }
    finish_idle();
    emit(ZB_EVT_DONE, 0);
}

static void home_fail(int err)
{
    s_homed = false;
    finish_idle();
    emit(ZB_EVT_ERR, err);
}

static void exec_home(void)
{
    sync_driver_microstep();
    s_homed = false;
    const float spm = spm_now();
    const int dir = es_dir();
    const float backoff = zb_set_get(ZS_HOME_BACKOFF);
    const float accel = zb_set_get(ZS_ACCEL), decel = zb_set_get(ZS_DECEL);
    const float fast = zb_set_get(ZS_HOME_FAST), slow = zb_set_get(ZS_HOME_SLOW);
    const int32_t back_steps = zb_mm_to_steps(backoff, spm);
    struct run r;
    struct run_res res;

    /* 0: already on the switch: leave it first */
    if (zb_endstop_active()) {
        mk_run(&r, 2 * back_steps, -dir, fast, accel, decel);
        run_profile(&r, &res);
        if (handle_abnormal(&res)) return;
        if (res.stopped) { finish_idle(); emit(ZB_EVT_DONE, 0); return; }
        vTaskDelay(pdMS_TO_TICKS((uint32_t)zb_set_get(ZS_ES_DEBOUNCE_US) / 1000 + 3));
        if (zb_endstop_active()) { home_fail(ZB_E_HOME_FAIL); return; }
    }

    /* 1: fast approach, decelerating stop once the switch is confirmed */
    mk_run(&r, zb_mm_to_steps(zb_set_get(ZS_MAX_RANGE) * 1.25f + 5.0f, spm), dir, fast, accel, decel);
    r.es_decel = true;
    run_profile(&r, &res);
    if (handle_abnormal(&res)) return;
    if (res.stopped) { finish_idle(); emit(ZB_EVT_DONE, 0); return; }
    if (!res.es_hit) { home_fail(ZB_E_HOME_FAIL); return; }
    int32_t over = s_pos_steps - res.es_pos;
    if (over < 0) over = -over;

    /* 2: back off past the switch */
    mk_run(&r, over + back_steps, -dir, fast, accel, decel);
    run_profile(&r, &res);
    if (handle_abnormal(&res)) return;
    if (res.stopped) { finish_idle(); emit(ZB_EVT_DONE, 0); return; }
    vTaskDelay(pdMS_TO_TICKS((uint32_t)zb_set_get(ZS_ES_DEBOUNCE_US) / 1000 + 3));
    if (zb_endstop_active()) { home_fail(ZB_E_HOME_FAIL); return; }

    /* 3: slow re-probe, abrupt stop, exact trigger position */
    mk_run(&r, 2 * back_steps + 50, dir, slow, 1.0e6f, 1.0e6f);
    r.vs = r.vc;
    r.es_stop_now = true;
    run_profile(&r, &res);
    if (handle_abnormal(&res)) return;
    if (res.stopped) { finish_idle(); emit(ZB_EVT_DONE, 0); return; }
    if (!res.es_hit) { home_fail(ZB_E_HOME_FAIL); return; }

    /* position space: the trigger point is the zero reference (or the max end if homing to max) */
    int32_t ref = dir > 0 ? zb_mm_to_steps(zb_set_get(ZS_MAX_RANGE), spm) : 0;
    s_pos_steps = s_pos_steps + (ref - res.es_pos);
    s_homed_spm = spm;
    s_homed = true;

    /* 4: park backoff mm inside the range */
    int32_t park = ref - dir * back_steps;
    int32_t maxs = zb_mm_to_steps(zb_set_get(ZS_MAX_RANGE), spm);
    if (park < 0) park = 0;
    if (park > maxs) park = maxs;
    mk_run(&r, park - s_pos_steps, park < s_pos_steps ? -1 : 1, fast, accel, decel);
    run_profile(&r, &res);
    if (handle_abnormal(&res)) return;
    finish_idle();
    emit(ZB_EVT_HOMED, 0);
}

static void motion_task(void *arg)
{
    (void)arg;
    struct cmd c;
    for (;;) {
        if (xQueueReceive(s_q, &c, pdMS_TO_TICKS(100)) != pdTRUE) {
            float off = zb_set_get(ZS_IDLE_OFF_S);
            if (s_state == ZB_IDLE && zb_driver_is_enabled() && off > 0.0f &&
                esp_timer_get_time() - s_last_active_us > (int64_t)(off * 1.0e6f))
                zb_driver_enable(false);
            continue;
        }
        switch (c.type) {
        case C_HOME: exec_home(); break;
        case C_MOVE:
            if (c.delta == 0) { finish_idle(); emit(ZB_EVT_DONE, 0); }
            else exec_move(c.delta, c.speed);
            break;
        case C_NOP:
        default:
            finish_idle();
            emit(ZB_EVT_DONE, 0);
            break;
        }
        s_last_active_us = esp_timer_get_time();
    }
}

bool zb_motion_start(zb_event_cb cb)
{
    s_cb = cb;
    zb_driver_init();
    s_lock = xSemaphoreCreateMutex();
    s_done_sem = xSemaphoreCreateBinary();
    s_q = xQueueCreate(4, sizeof(struct cmd));
    if (!s_lock || !s_done_sem || !s_q) return false;
    zb_endstop_init(&s_pos_steps);
    if (!chan_create()) return false;
    s_last_active_us = esp_timer_get_time();
    return xTaskCreate(motion_task, "zb_motion", 4096, NULL, 10, NULL) == pdPASS;
}

void zb_motion_status(struct zb_status *s)
{
    check_homed_scale();
    s->pos = zb_steps_to_mm(s_pos_steps, spm_now());
    s->state = s_state;
    s->homed = s_homed;
    s->endstop = zb_endstop_active();
    s->speed = s_speed;
}

/* ---- commands ---- */

/* Accept a command if the axis can take it; sets the new state under the lock. */
static int accept(enum zb_state new_state, const struct cmd *c, bool allow_fault)
{
    int rc = ZB_E_NONE;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_state == ZB_FAULT && !allow_fault) rc = ZB_E_FAULT;
    else if (s_state != ZB_IDLE && !(s_state == ZB_FAULT && allow_fault)) rc = ZB_E_BUSY;
    else {
        s_stop_req = false;
        s_estop = false;
        s_state = new_state;
        if (xQueueSend(s_q, c, 0) != pdTRUE) { s_state = ZB_IDLE; rc = ZB_E_BUSY; }
    }
    xSemaphoreGive(s_lock);
    return rc;
}

int zb_home(void)
{
    struct cmd c = { C_HOME, 0, 0 };
    return accept(ZB_HOMING, &c, true);
}

static int move_common(float target_mm, bool relative, float speed)
{
    if (!isfinite(target_mm) || !isfinite(speed) || speed < 0.0f) return ZB_E_ARG;
    check_homed_scale();
    float spm = spm_now();
    float max_speed = zb_set_get(ZS_MAX_SPEED);
    if (speed == 0.0f || speed > max_speed) speed = max_speed;
    if (speed < 0.1f) speed = 0.1f;
    bool unhomed_ok = relative && zb_set_get(ZS_ALLOW_UNHOMED) != 0.0f;
    if (!s_homed && !unhomed_ok) return ZB_E_NOT_HOMED;
    int32_t pos = s_pos_steps;
    int32_t tgt = relative ? pos + zb_mm_to_steps(target_mm, spm) : zb_mm_to_steps(target_mm, spm);
    if (s_homed) {
        int32_t maxs = zb_mm_to_steps(zb_set_get(ZS_MAX_RANGE), spm);
        if (tgt < 0 || tgt > maxs) return ZB_E_RANGE;
    }
    struct cmd c = { C_MOVE, tgt - pos, speed };
    return accept(ZB_MOVING, &c, false);
}

int zb_move_abs(float mm, float speed_mm_s) { return move_common(mm, false, speed_mm_s); }
int zb_move_rel(float dmm, float speed_mm_s) { return move_common(dmm, true, speed_mm_s); }

int zb_stop(void)
{
    struct cmd c = { C_NOP, 0, 0 };
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_state == ZB_MOVING || s_state == ZB_HOMING) s_stop_req = true;
    else if (s_state == ZB_CAL) s_cal_abort = true;
    else if (s_state == ZB_IDLE) { s_state = ZB_MOVING; xQueueSend(s_q, &c, 0); }   /* reports DONE */
    xSemaphoreGive(s_lock);
    return ZB_E_NONE;
}

void zb_estop(void)
{
    struct cmd c = { C_NOP, 0, 0 };
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_homed = false;
    if (s_state == ZB_IDLE || s_state == ZB_FAULT) {
        s_state = ZB_MOVING;
        xQueueSend(s_q, &c, 0);
    } else if (!s_estop) {
        s_estop = true;
        s_cal_abort = true;
        if (s_chan) rmt_disable(s_chan);   /* halts the running transmission, STEP idles low */
    }
    xSemaphoreGive(s_lock);
}

/* ---- calibration primitives ---- */

bool zb_cal_enter(void)
{
    bool ok = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    check_homed_scale();
    if (s_state == ZB_IDLE && s_homed) {
        s_state = ZB_CAL;
        s_cal_abort = false;
        s_stop_req = false;
        s_estop = false;
        ok = true;
    }
    xSemaphoreGive(s_lock);
    if (ok) sync_driver_microstep();
    return ok;
}

void zb_cal_leave(void)
{
    if (s_cal_abort) s_homed = false;   /* an aborted run may have stopped anywhere unreferenced */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_estop = false;
    s_stop_req = false;
    if (s_state == ZB_CAL) s_state = ZB_IDLE;
    xSemaphoreGive(s_lock);
    s_last_active_us = esp_timer_get_time();
}

void zb_cal_request_abort(void) { s_cal_abort = true; }
bool zb_cal_abort_requested(void) { return s_cal_abort || s_estop; }
float zb_cal_position(void) { return zb_steps_to_mm(s_pos_steps, spm_now()); }

int zb_cal_move(float target_mm, float speed, float accel, float decel, bool stop_on_endstop, float *trigger_mm)
{
    float spm = spm_now();
    if (trigger_mm) *trigger_mm = NAN;
    if (s_state != ZB_CAL || !isfinite(target_mm) || speed <= 0.0f || accel <= 0.0f || decel <= 0.0f)
        return ZB_E_ARG;
    int32_t delta = zb_mm_to_steps(target_mm, spm) - s_pos_steps;
    if (fabsf(zb_steps_to_mm(delta, spm)) > zb_set_get(ZS_MAX_RANGE) + 10.0f) return ZB_E_RANGE;
    struct run r;
    struct run_res res;
    mk_run(&r, delta, delta < 0 ? -1 : 1, speed, accel, decel);
    r.es_decel = stop_on_endstop;
    run_profile(&r, &res);
    if (res.halted || res.err == ZB_E_FAULT) return ZB_E_FAULT;
    if (res.stopped) return ZB_E_FAULT;
    if (res.es_hit) {
        if (trigger_mm) *trigger_mm = zb_steps_to_mm(res.es_pos, spm);
        return ZB_E_ENDSTOP;
    }
    return ZB_E_NONE;
}

int zb_cal_probe(float speed_mm_s, float *trigger_mm)
{
    float spm = spm_now();
    if (trigger_mm) *trigger_mm = NAN;
    if (s_state != ZB_CAL || !isfinite(speed_mm_s) || speed_mm_s <= 0.0f) return ZB_E_ARG;
    struct run r;
    struct run_res res;
    mk_run(&r, zb_mm_to_steps(zb_set_get(ZS_MAX_RANGE) + 5.0f, spm), es_dir(), speed_mm_s, 1.0e6f, 1.0e6f);
    r.vs = r.vc;
    r.es_stop_now = true;
    run_profile(&r, &res);
    if (res.halted || res.err == ZB_E_FAULT || res.stopped) return ZB_E_FAULT;
    if (!res.es_hit) return ZB_E_HOME_FAIL;
    if (trigger_mm) *trigger_mm = zb_steps_to_mm(res.es_pos, spm);
    return ZB_E_NONE;
}
