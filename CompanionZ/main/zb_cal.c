/* Z-Bro calibration state machine (docs/ZBRO_PLAN.md section 5, API shape in section 6).
 *
 * Lost-step detection: after out-and-back cycles at the candidate parameter, slow-probe the endstop and
 * compare the trigger position with the previous one. A moved trigger means steps were lost. The probe
 * result also re-syncs a position offset `s_off` (believed - true), so after a failed trial the following
 * moves still target the true positions. */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "zb_cal.h"
#include "zb_cal_search.h"
#include "zb_motion.h"
#include "zb_settings.h"

#define TOL_MM        0.05f   /* lost-step tolerance */
#define CYCLES        4       /* out-and-back cycles per trial */
#define MAX_STEP_HZ   100000.0f
#define MAX_TRIALS_UI 24      /* trials kept per test */
#define WORKING_FRAC  0.8f

enum { T_ENDSTOP, T_SPEED, T_ACCEL, T_DECEL, T_HOMING, T_COUNT };
static const char *const k_names[T_COUNT] = { "endstop", "speed", "accel", "decel", "homing" };
static const char *const k_units[T_COUNT] = { "um", "mm/s", "mm/s2", "mm/s2", "mm/s" };
enum { S_PENDING, S_RUNNING, S_OK, S_FAILED, S_SKIPPED };
static const char *const k_status[] = { "pending", "running", "ok", "failed", "skipped" };

struct trial { float value; bool ok; };
struct test {
    int status;
    float progress;
    bool has_result;
    float limit, working;
    int ntrials;
    struct trial trials[MAX_TRIALS_UI];
};

static struct {
    bool running;
    int current;                 /* -1 none */
    char error[48];
    bool needs_home;
    struct test t[T_COUNT];
} s_st = { .current = -1 };

static SemaphoreHandle_t s_mu;
static zb_cal_cb s_cb;
static bool s_sel[T_COUNT];

/* Geometry / reference, true coordinates (position after homing is trusted at start). */
static float s_ref;      /* trigger position of the endstop (true) */
static float s_off;      /* believed - true */
static float s_dir;      /* +1/-1: direction away from the endstop */
static float s_near;     /* parking position near the endstop (true) */
static float s_far;      /* far end of the test stroke (true) */
static float s_lo, s_hi; /* allowed envelope (true) */
static char  s_fatal[48];

static void lock(void)   { xSemaphoreTake(s_mu, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_mu); }
static void notify(void) { if (s_cb) s_cb(); }

void zb_cal_set_cb(zb_cal_cb cb) { s_cb = cb; }

static void set_progress(int ti, float p)
{
    lock(); s_st.t[ti].progress = p > 1.0f ? 1.0f : p; unlock();
    notify();
}

/* ---- motion helpers (true coordinates) ---- */

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* Returns 0 ok, 1 endstop hit (steps lost / unexpected), -1 abort or fatal. */
static int mv(float target, float speed, float accel, float decel)
{
    float trig = NAN;
    int rc;
    target = clampf(target, s_lo, s_hi);
    rc = zb_cal_move(target + s_off, speed, accel, decel, true, &trig);
    if (zb_cal_abort_requested()) return -1;
    if (rc == ZB_E_ENDSTOP || !isnan(trig)) return 1;
    if (rc != ZB_E_NONE) { strcpy(s_fatal, "motion error"); return -1; }
    return 0;
}

/* Slow probe from the parking position; updates s_off from the trigger. Returns the trigger in true
 * coordinates via *true_trig (before offset update). 0 ok, -1 abort/fatal. */
static int probe(float *believed_trig)
{
    int rc = zb_cal_probe(zb_set_get(ZS_HOME_SLOW), believed_trig);
    if (zb_cal_abort_requested()) return -1;
    if (rc != ZB_E_NONE) { strcpy(s_fatal, "probe failed"); return -1; }
    return 0;
}

/* Move back from the endstop to the parking position (endstop is triggered, so no stop_on_endstop). */
static int back_off(void)
{
    float sp = zb_set_get(ZS_MAX_SPEED) * 0.25f;
    int rc;
    if (sp < 1.0f) sp = 1.0f;
    rc = zb_cal_move(s_near + s_off, sp, zb_set_get(ZS_ACCEL), zb_set_get(ZS_DECEL), false, NULL);
    if (zb_cal_abort_requested()) return -1;
    if (rc != ZB_E_NONE) { strcpy(s_fatal, "motion error"); return -1; }
    return 0;
}

/* Probe + back off; returns the new offset error delta (believed trigger vs the previous reference). */
static int probe_and_park(float *delta)
{
    float t;
    if (probe(&t) < 0) return -1;
    *delta = (t - s_off) - s_ref;           /* true-coordinate error of the trigger point */
    s_off = t - s_ref;                      /* re-sync */
    return back_off();
}

/* Establish reference + geometry. n probes; returns spread (max-min) in mm via *spread. */
static int take_reference(int n, float *spread, int ti, bool report_progress)
{
    float lo = 1e9f, hi = -1e9f, sum = 0.0f;
    float maxr = zb_set_get(ZS_MAX_RANGE);
    float stroke = 0.0f, backoff = zb_set_get(ZS_HOME_BACKOFF);
    for (int i = 0; i < n; i++) {
        float t;
        if (probe(&t) < 0) return -1;
        if (t < lo) lo = t;
        if (t > hi) hi = t;
        sum += t;
        if (i == 0) { s_ref = t; s_off = 0.0f; }     /* provisional, geometry follows */
        if (i == 0) {
            s_dir = (t > 0.5f * maxr) ? -1.0f : 1.0f;
            s_near = t + s_dir * backoff;
            stroke = maxr * 0.5f; if (stroke > 40.0f) stroke = 40.0f;
            s_far = t + s_dir * stroke;
            s_lo = (t < 0.0f ? t : 0.0f); s_hi = (t > maxr ? t : maxr);
            s_far = clampf(s_far, s_lo, s_hi);
        }
        if (back_off() < 0) return -1;
        if (report_progress) set_progress(ti, (float)(i + 1) / (float)n);
    }
    s_ref = sum / (float)n;
    s_off = 0.0f;
    if (fabsf(s_far - s_near) < 5.0f) { strcpy(s_fatal, "no free travel"); return -1; }
    if (spread) *spread = hi - lo;
    return 0;
}

/* ---- trials ---- */

struct sctx { int ti; int n; int est; float speed, accel, decel; bool homing; };

static void record(int ti, float v, bool ok)
{
    lock();
    struct test *t = &s_st.t[ti];
    if (t->ntrials < MAX_TRIALS_UI) { t->trials[t->ntrials].value = v; t->trials[t->ntrials].ok = ok; t->ntrials++; }
    unlock();
}

/* One trial: cycles at the parameter under test, then slow probe vs reference. 1 pass, 0 fail, -1 abort. */
static int trial(void *ctx, float v)
{
    struct sctx *c = ctx;
    float speed = c->speed, accel = c->accel, decel = c->decel, delta = 0.0f;
    bool ok = true;
    int rc;

    if (c->ti == T_SPEED) speed = v;
    else if (c->ti == T_ACCEL) accel = v;
    else if (c->ti == T_DECEL) decel = v;

    if (c->homing) {
        /* approach the endstop at speed v, must stop inside home_backoff after the trigger */
        float trig = NAN, pos;
        rc = mv(s_far, zb_set_get(ZS_MAX_SPEED), accel, decel);
        if (rc < 0) return -1;
        rc = zb_cal_move(clampf(s_ref - s_dir * 2.0f * zb_set_get(ZS_HOME_BACKOFF), s_lo, s_hi) + s_off,
                         v, accel, decel, true, &trig);
        if (zb_cal_abort_requested()) return -1;
        pos = zb_cal_position();
        if (rc != ZB_E_NONE && rc != ZB_E_ENDSTOP) { strcpy(s_fatal, "motion error"); return -1; }
        if (isnan(trig) || fabsf(pos - trig) > zb_set_get(ZS_HOME_BACKOFF)) ok = false;
        /* retreat (endstop may still be active) before the sync probe */
        rc = zb_cal_move(s_near + s_off, zb_set_get(ZS_MAX_SPEED) * 0.25f, accel, decel, false, NULL);
        if (zb_cal_abort_requested()) return -1;
        if (rc != ZB_E_NONE) { strcpy(s_fatal, "motion error"); return -1; }
    } else {
        for (int i = 0; i < CYCLES && ok; i++) {
            rc = mv(s_far, speed, accel, decel);
            if (rc < 0) return -1;
            if (rc > 0) { ok = false; break; }
            rc = mv(s_near, speed, accel, decel);
            if (rc < 0) return -1;
            if (rc > 0) ok = false;
        }
    }
    if (probe_and_park(&delta) < 0) return -1;
    if (fabsf(delta) > TOL_MM) ok = false;

    record(c->ti, v, ok);
    c->n++;
    set_progress(c->ti, (float)c->n / (float)c->est);
    return ok ? 1 : 0;
}

static void store_limit(int ti, enum zb_set_id lim_id, enum zb_set_id work_id, float limit)
{
    float lo, hi, w = limit * WORKING_FRAC;
    zb_set_set(lim_id, limit, true);
    zb_set_range(work_id, &lo, &hi);
    zb_set_set(work_id, clampf(w, lo, hi), true);
    lock();
    s_st.t[ti].has_result = true;
    s_st.t[ti].limit = limit;
    s_st.t[ti].working = zb_set_get(work_id);
    unlock();
}

/* Returns 0 normally, -1 on abort/fatal. */
static int run_search_test(int ti)
{
    struct sctx c = { ti, 0, 0, zb_set_get(ZS_MAX_SPEED), zb_set_get(ZS_ACCEL), zb_set_get(ZS_DECEL), ti == T_HOMING };
    struct zbs_cfg cfg = { 0, 1.5f, 0, 3 };
    enum zb_set_id work, lim;
    float lo, hi;
    float spm = zb_steps_per_mm();
    struct zbs_result r;

    switch (ti) {
    case T_SPEED:  work = ZS_MAX_SPEED; lim = ZS_LIM_SPEED; cfg.start = 2.0f; break;
    case T_ACCEL:  work = ZS_ACCEL; lim = ZS_LIM_ACCEL; cfg.start = 20.0f; break;
    case T_DECEL:  work = ZS_DECEL; lim = ZS_LIM_DECEL; cfg.start = 20.0f; break;
    default:       work = ZS_HOME_FAST; lim = ZS_LIM_HOME_FAST; cfg.start = 2.0f; break;
    }
    zb_set_range(work, &lo, &hi);
    cfg.max = hi;
    if (ti == T_SPEED && spm > 0.0f && cfg.max > MAX_STEP_HZ / spm) cfg.max = MAX_STEP_HZ / spm;
    if (ti == T_HOMING && cfg.max > zb_set_get(ZS_MAX_SPEED)) cfg.max = zb_set_get(ZS_MAX_SPEED);
    if (cfg.start < lo) cfg.start = lo;
    c.est = zbs_est_trials(&cfg);

    r = zbs_search(&cfg, trial, &c);
    if (r.status == ZBS_ABORTED) return -1;
    lock();
    s_st.t[ti].status = (r.status == ZBS_FAIL_START) ? S_FAILED : S_OK;
    s_st.t[ti].progress = 1.0f;
    unlock();
    if (r.status != ZBS_FAIL_START) store_limit(ti, lim, work, r.limit);
    return 0;
}

static int run_endstop_test(int ti)
{
    float spread = 0.0f, jitter_us, lo, hi, deb;
    if (take_reference(10, &spread, ti, true) < 0) return -1;
    zb_set_set(ZS_ES_REPEAT_UM, spread * 1000.0f, true);
    /* time jitter of the trigger at the probe speed -> debounce at twice that, within the allowed range */
    jitter_us = spread / zb_set_get(ZS_HOME_SLOW) * 1e6f;
    zb_set_range(ZS_ES_DEBOUNCE_US, &lo, &hi);
    deb = clampf(2.0f * jitter_us, 500.0f, 10000.0f);
    deb = clampf(deb, lo, hi);
    zb_set_set(ZS_ES_DEBOUNCE_US, deb, true);
    lock();
    s_st.t[ti].has_result = true;
    s_st.t[ti].limit = spread * 1000.0f;
    s_st.t[ti].working = deb;
    s_st.t[ti].progress = 1.0f;
    s_st.t[ti].status = spread > TOL_MM ? S_FAILED : S_OK;
    unlock();
    return 0;
}

static void finish(const char *err)
{
    lock();
    s_st.running = false;
    s_st.current = -1;
    if (err) { strncpy(s_st.error, err, sizeof s_st.error - 1); s_st.error[sizeof s_st.error - 1] = 0; }
    for (int i = 0; i < T_COUNT; i++)
        if (s_st.t[i].status == S_RUNNING) s_st.t[i].status = S_FAILED;
    unlock();
    notify();
}

static void cal_task(void *arg)
{
    bool entered = false, referenced = false, aborted = false;
    (void)arg;

    if (!zb_cal_enter()) { finish("axis not ready"); vTaskDelete(NULL); return; }
    entered = true;

    for (int i = 0; i < T_COUNT && !aborted; i++) {
        int rc;
        if (!s_sel[i]) continue;
        lock(); s_st.current = i; s_st.t[i].status = S_RUNNING; unlock();
        notify();
        if (i == T_ENDSTOP) {
            rc = run_endstop_test(i);
            referenced = (rc == 0);
        } else {
            if (!referenced) {                       /* reference from 3 probes when endstop test not run */
                rc = take_reference(3, NULL, i, false);
                referenced = (rc == 0);
                if (rc < 0) { aborted = true; break; }
            }
            rc = run_search_test(i);
        }
        if (rc < 0) { aborted = true; break; }
        zb_set_save();
        if (s_st.t[i].status == S_FAILED && i == T_ENDSTOP) {   /* unreliable endstop: later results meaningless */
            for (int j = i + 1; j < T_COUNT; j++)
                if (s_sel[j]) s_st.t[j].status = S_SKIPPED;
            break;
        }
    }
    zb_set_set(ZS_CAL_TIME, (float)time(NULL), true);
    zb_set_save();

    if (aborted) {
        finish(s_fatal[0] ? s_fatal : "aborted");
    } else {
        /* park at the safe position near the endstop (true coordinates), after a last sync */
        if (referenced) {
            float d;
            if (probe_and_park(&d) == 0 && fabsf(s_off) > TOL_MM) s_st.needs_home = true;
        }
        finish(s_fatal[0] ? s_fatal : NULL);
    }
    if (entered) zb_cal_leave();
    vTaskDelete(NULL);
}

/* ---- public API ---- */

bool zb_cal_start(const char *const *tests, int n)
{
    struct zb_status st;
    bool sel[T_COUNT] = { false };
    bool run;

    if (!s_mu) s_mu = xSemaphoreCreateMutex();
    if (!s_mu) return false;
    if (n <= 0 || !tests) {
        for (int i = 0; i < T_COUNT; i++) sel[i] = true;
    } else {
        for (int k = 0; k < n; k++) {
            int i;
            for (i = 0; i < T_COUNT; i++) if (tests[k] && !strcmp(tests[k], k_names[i])) break;
            if (i == T_COUNT) return false;
            sel[i] = true;
        }
    }
    zb_motion_status(&st);
    if (!st.homed || st.state != ZB_IDLE) return false;

    lock();
    run = s_st.running;
    if (!run) {
        memset(&s_st, 0, sizeof s_st);
        s_st.running = true;
        s_st.current = -1;
        for (int i = 0; i < T_COUNT; i++) {
            s_sel[i] = sel[i];
            s_st.t[i].status = sel[i] ? S_PENDING : S_SKIPPED;
        }
        s_fatal[0] = 0;
    }
    unlock();
    if (run) return false;
    if (xTaskCreate(cal_task, "zb_cal", 4096, NULL, 4, NULL) != pdPASS) {
        finish("no memory");
        return false;
    }
    return true;
}

void zb_cal_abort(void)
{
    zb_cal_request_abort();
}

char *zb_cal_json(void)
{
    cJSON *root, *arr;
    char *out;

    if (!s_mu) s_mu = xSemaphoreCreateMutex();
    root = cJSON_CreateObject();
    arr = cJSON_AddArrayToObject(root, "tests");
    if (!root || !arr) { cJSON_Delete(root); return NULL; }
    lock();
    cJSON_AddBoolToObject(root, "running", s_st.running);
    cJSON_AddStringToObject(root, "current", s_st.current >= 0 ? k_names[s_st.current] : "");
    if (s_st.error[0]) cJSON_AddStringToObject(root, "error", s_st.error);
    if (s_st.needs_home) cJSON_AddBoolToObject(root, "needs_home", true);
    for (int i = 0; i < T_COUNT; i++) {
        const struct test *t = &s_st.t[i];
        cJSON *o = cJSON_CreateObject(), *tr = cJSON_CreateArray();
        if (!o || !tr) { cJSON_Delete(o); cJSON_Delete(tr); continue; }
        cJSON_AddStringToObject(o, "name", k_names[i]);
        cJSON_AddStringToObject(o, "status", k_status[t->status]);
        cJSON_AddNumberToObject(o, "progress", t->progress);
        if (t->has_result) {
            cJSON *r = cJSON_AddObjectToObject(o, "result");
            cJSON_AddNumberToObject(r, "limit", t->limit);
            cJSON_AddStringToObject(r, "unit", k_units[i]);
            cJSON_AddNumberToObject(r, "working", t->working);
        } else {
            cJSON_AddNullToObject(o, "result");
        }
        for (int k = 0; k < t->ntrials; k++) {
            cJSON *e = cJSON_CreateObject();
            cJSON_AddNumberToObject(e, "value", t->trials[k].value);
            cJSON_AddBoolToObject(e, "ok", t->trials[k].ok);
            cJSON_AddItemToArray(tr, e);
        }
        cJSON_AddItemToObject(o, "trials", tr);
        cJSON_AddItemToArray(arr, o);
    }
    unlock();
    out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}
