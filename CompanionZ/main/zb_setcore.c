#include "zb_setcore.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const float k_ms_opts[] = { 1, 2, 4, 8, 16 };

#define NUM(k, l, g, u, d, lo, hi, st) { k, l, g, u, d, lo, hi, st, ZBC_NUM, NULL, 0 }
#define BOOL(k, l, g, d)               { k, l, g, "", d, 0, 1, 1, ZBC_BOOL, NULL, 0 }

const struct zbc_def zbc_defs[ZS_COUNT] = {
    [ZS_STEPS_REV]     = NUM("steps_rev",      "Steps per revolution", "Motion",  "",       200,  1,    1000,  1),
    [ZS_LEAD_MM]       = NUM("lead_mm",        "Lead per revolution",  "Motion",  "mm",     8,    0.1f, 100,   0.1f),
    [ZS_MICROSTEP]     = { "microstep", "Microstepping", "Motion", "1/n", 16, 1, 16, 1, ZBC_ENUM, k_ms_opts, 5 },
    [ZS_DIR_INVERT]    = BOOL("dir_invert",    "Invert direction",     "Motion",  0),
    [ZS_ES_INVERT]     = BOOL("es_invert",     "Endstop active low (NO switch)", "Homing", 0),
    [ZS_HOME_TO_MAX]   = BOOL("home_to_max",   "Home toward max end",  "Homing",  1),
    [ZS_MAX_RANGE]     = NUM("max_range",      "Travel range",         "Motion",  "mm",     50,   1,    500,   1),
    [ZS_MAX_SPEED]     = NUM("max_speed",      "Max speed",            "Motion",  "mm/s",   20,   0.1f, 100,   0.5f),
    [ZS_ACCEL]         = NUM("accel",          "Acceleration",         "Motion",  "mm/s2",  100,  1,    5000,  10),
    [ZS_DECEL]         = NUM("decel",          "Deceleration",         "Motion",  "mm/s2",  100,  1,    5000,  10),
    [ZS_HOME_FAST]     = NUM("home_fast",      "Homing fast speed",    "Homing",  "mm/s",   10,   0.5f, 50,    0.5f),
    [ZS_HOME_SLOW]     = NUM("home_slow",      "Homing slow speed",    "Homing",  "mm/s",   1,    0.1f, 10,    0.1f),
    [ZS_HOME_BACKOFF]  = NUM("home_backoff",   "Homing back-off",      "Homing",  "mm",     2,    0.5f, 20,    0.5f),
    [ZS_ES_DEBOUNCE_US]= NUM("es_debounce_us", "Endstop debounce",     "Homing",  "us",     2000, 0,    20000, 100),
    [ZS_PULSE_US]      = NUM("pulse_us",       "STEP pulse width",     "Driver",  "us",     2,    1,    20,    1),
    [ZS_IDLE_OFF_S]    = NUM("idle_off_s",     "Motor off after idle (0 = never)", "Driver", "s", 5, 0, 3600,  1),
    [ZS_SLEEP_IDLE]    = BOOL("sleep_idle",    "Sleep driver when idle", "Driver", 1),
    [ZS_RPT_HZ]        = NUM("rpt_hz",         "Position report rate", "Driver",  "Hz",     10,   1,    50,    1),
    [ZS_ALLOW_UNHOMED] = BOOL("allow_unhomed", "Allow jog when unhomed", "Driver", 0),
    [ZS_LIM_SPEED]     = NUM("lim_speed",      "Measured speed limit", "Calibration", "mm/s",  0, 0, 1000,  0.1f),
    [ZS_LIM_ACCEL]     = NUM("lim_accel",      "Measured accel limit", "Calibration", "mm/s2", 0, 0, 100000, 1),
    [ZS_LIM_DECEL]     = NUM("lim_decel",      "Measured decel limit", "Calibration", "mm/s2", 0, 0, 100000, 1),
    [ZS_LIM_HOME_FAST] = NUM("lim_home_fast",  "Measured homing limit", "Calibration", "mm/s", 0, 0, 1000,  0.1f),
    [ZS_ES_REPEAT_UM]  = NUM("es_repeat_um",   "Endstop repeatability", "Calibration", "um",   0, 0, 10000, 1),
    [ZS_CAL_TIME]      = NUM("cal_time",       "Calibrated at (unix s)", "Calibration", "s",   0, 0, 4294967040.0f, 1),
};

static int cap_id(enum zb_set_id id)
{
    switch (id) {
    case ZS_MAX_SPEED: return ZS_LIM_SPEED;
    case ZS_ACCEL:     return ZS_LIM_ACCEL;
    case ZS_DECEL:     return ZS_LIM_DECEL;
    case ZS_HOME_FAST: return ZS_LIM_HOME_FAST;
    default:           return -1;
    }
}

void zbc_range(const float *vals, enum zb_set_id id, float *lo, float *hi)
{
    const struct zbc_def *d = &zbc_defs[id];
    float h = d->max;
    int c = cap_id(id);
    if (c >= 0 && vals[c] > 0.0f && vals[c] < h) h = vals[c];
    if (lo) *lo = d->min;
    if (hi) *hi = h;
}

int zbc_validate(const float *vals, enum zb_set_id id, float v)
{
    if ((int)id < 0 || id >= ZS_COUNT || !isfinite(v)) return 4;
    const struct zbc_def *d = &zbc_defs[id];
    switch (d->type) {
    case ZBC_BOOL:
        return (v == 0.0f || v == 1.0f) ? 0 : 4;
    case ZBC_ENUM:
        for (int i = 0; i < d->n_opts; i++) if (d->opts[i] == v) return 0;
        return 4;
    default: {
        float lo, hi;
        zbc_range(vals, id, &lo, &hi);
        /* tiny tolerance so a value that was clamped to the limit with float rounding still passes */
        return (v >= lo && v <= hi * 1.000001f) ? 0 : 4;
    }
    }
}

int zbc_hostname_valid(const char *name)
{
    if (!name) return 0;
    size_t n = strlen(name);
    if (n < 1 || n > 31 || name[0] == '-' || name[n - 1] == '-') return 0;
    for (size_t i = 0; i < n; i++) {
        char c = name[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-')) return 0;
    }
    return 1;
}

struct jb { char *buf; size_t n, len; };

static void jput(struct jb *j, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void jput(struct jb *j, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(j->buf && j->len < j->n ? j->buf + j->len : NULL,
                      j->buf && j->len < j->n ? j->n - j->len : 0, fmt, ap);
    va_end(ap);
    if (w > 0) j->len += (size_t)w;
}

size_t zbc_json(const float *vals, char *buf, size_t n)
{
    static const char *const groups[] = { "Motion", "Homing", "Driver", "Calibration" };
    struct jb j = { buf, n, 0 };
    jput(&j, "{\"groups\":[");
    for (size_t g = 0; g < sizeof groups / sizeof groups[0]; g++) {
        jput(&j, "%s{\"name\":\"%s\",\"items\":[", g ? "," : "", groups[g]);
        int first = 1;
        for (int i = 0; i < ZS_COUNT; i++) {
            const struct zbc_def *d = &zbc_defs[i];
            if (strcmp(d->group, groups[g]) != 0) continue;
            float lo, hi;
            zbc_range(vals, (enum zb_set_id)i, &lo, &hi);
            const char *type = d->type == ZBC_BOOL ? "bool" : d->type == ZBC_ENUM ? "enum" : "number";
            jput(&j, "%s{\"key\":\"%s\",\"label\":\"%s\",\"value\":%.7g,\"min\":%.7g,\"max\":%.7g,"
                     "\"step\":%.7g,\"unit\":\"%s\",\"type\":\"%s\",\"options\":[",
                 first ? "" : ",", d->key, d->label, (double)vals[i], (double)lo, (double)hi,
                 (double)d->step, d->unit, type);
            for (int o = 0; d->type == ZBC_ENUM && o < d->n_opts; o++)
                jput(&j, "%s%.7g", o ? "," : "", (double)d->opts[o]);
            jput(&j, "]}");
            first = 0;
        }
        jput(&j, "]}");
    }
    jput(&j, "]}");
    if (buf && n) buf[j.len < n ? j.len : n - 1] = 0;
    return j.len;
}
