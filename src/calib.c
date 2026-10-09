#include "calib.h"
#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "calib";
#define NVS_NS "wmcal"

/*
 * One table drives load, save, validation and the "$$" dump. NVS keys of
 * settings that existed before the galvo rework ($100/$101 scale, the X/Y
 * offsets, k1, the delay and wobble values) are kept so a board that was
 * already calibrated keeps its numbers.
 */
struct calib_def {
    int number;        /* "$" number */
    int alias;         /* second "$" number writing the same value, or -1 */
    const char *key;   /* NVS key (<= 15 chars), NULL = read-only constant */
    float def;
    float min, max;
};

static const struct calib_def s_defs[CAL_COUNT] = {
    [CAL_AXIS_INVERT]        = {   3,  -1, "axinv",     0.0f,      0.0f,        3.0f },
    [CAL_ARC_TOL]            = {  12,  -1, "arctol",    0.01f,     0.0005f,     1.0f },
    [CAL_S_MAX]              = {  30,  -1, "smax",      1000.0f,   1.0f,        100000.0f },
    [CAL_X_SCALE]            = { 100,  -1, "xvpm",      0.1f,     -100.0f,      100.0f },
    [CAL_Y_SCALE]            = { 101,  -1, "yvpm",      0.1f,     -100.0f,      100.0f },
    [CAL_MAX_MARK_RATE]      = { 110, 111, "maxrate",   300000.0f, 1.0f,        3000000.0f },
    [CAL_JUMP_ACCEL]         = { 120, 121, "jaccel",    2000000.0f, 0.0f,       1.0e9f },
    [CAL_FIELD_W]            = { 130,  -1, "fieldw",    100.0f,    1.0f,        1000.0f },
    [CAL_FIELD_H]            = { 131,  -1, "fieldh",    100.0f,    1.0f,        1000.0f },
    [CAL_X_OFFSET]           = { 140,  -1, "xoff",      0.0f,     -10.0f,       10.0f },
    [CAL_Y_OFFSET]           = { 141,  -1, "yoff",      0.0f,     -10.0f,       10.0f },
    [CAL_RADIAL_K1]          = { 142,  -1, "radk1",     0.0f,     -1.0f,        1.0f },
    [CAL_SWAP_XY]            = { 143,  -1, "swapxy",    0.0f,      0.0f,        1.0f },
    [CAL_ORIGIN_CENTER]      = { 144,  -1, "origc",     0.0f,      0.0f,        1.0f },
    [CAL_WOBBLE_D]           = { 150,  -1, "wobd",      0.0f,      0.0f,        10.0f },
    [CAL_WOBBLE_PITCH]       = { 151,  -1, "wobp",      0.5f,      0.001f,      100.0f },
    [CAL_TICK_US]            = { 200,  -1, "tickus",    10.0f,     2.0f,        200.0f },
    [CAL_JUMP_SPEED]         = { 201,  -1, "jspeed",    3000.0f,   1.0f,        100000.0f },
    [CAL_DEFAULT_MARK_SPEED] = { 203,  -1, "dmspeed",   500.0f,    0.1f,        100000.0f },
    [CAL_LASER_ON_DELAY]     = { 210,  -1, "laserond",  100.0f,    0.0f,        100000.0f },
    [CAL_LASER_OFF_DELAY]    = { 211,  -1, "laseroffd", 120.0f,    0.0f,        100000.0f },
    [CAL_JUMP_DELAY]         = { 212,  -1, "jumpdly",   300.0f,    0.0f,        1000000.0f },
    [CAL_JUMP_DELAY_PER_MM]  = { 213,  -1, "jdlypmm",   0.0f,      0.0f,        100000.0f },
    [CAL_MARK_DELAY]         = { 214,  -1, "markdly",   100.0f,    0.0f,        1000000.0f },
    [CAL_POLYGON_DELAY]      = { 215,  -1, "polydly",   0.0f,      0.0f,        100000.0f },
    [CAL_POLYGON_ANGLE]      = { 216,  -1, "polyang",   30.0f,     0.0f,        180.0f },
    [CAL_PRR_HZ]             = { 220,  -1, "prrhz",     30000.0f,  0.0f,        2000000.0f },
    [CAL_PRR_DUTY]           = { 221,  -1, "prrduty",   50.0f,     1.0f,        99.0f },
    [CAL_POWER_MODE]         = { 223,  -1, "pwrmode",   0.0f,      0.0f,        1.0f },
    [CAL_POWER_MIN]          = { 224,  -1, "pwrmin",    0.0f,      0.0f,        100.0f },
    [CAL_POWER_MAX]          = { 225,  -1, "pwrmax",    100.0f,    0.0f,        100.0f },
    [CAL_AUTO_ARM]           = { 226,  -1, "autoarm",   1.0f,      0.0f,        1.0f },
    [CAL_ARM_TIMEOUT_MS]     = { 227,  -1, "armtmo",    4000.0f,   0.0f,        60000.0f },
    [CAL_PD_PERIOD]          = { 229,  -1, "pdper",     10.0f,     2.0f,        255.0f },
    [CAL_GATE_ACTIVE_LOW]    = { 230,  -1, "gatelow",   0.0f,      0.0f,        1.0f },
    [CAL_AUTO_GUIDE]         = { 231,  -1, "autoguide", 1.0f,      0.0f,        1.0f },
};

/* Read-only GRBL settings reported for sender compatibility. */
struct calib_const {
    int number;
    float value;
};

static const struct calib_const s_consts[] = {
    { 0, 10 }, { 1, 25 }, { 2, 0 }, { 4, 0 }, { 5, 0 }, { 6, 0 },
    { 10, 1 }, { 11, 0.010f }, { 13, 0 }, { 20, 0 }, { 21, 0 }, { 22, 0 },
    { 23, 0 }, { 24, 25 }, { 25, 500 }, { 26, 250 }, { 27, 1 },
    { 31, 0 }, { 32, 1 }, { 102, 1 }, { 112, 1000 }, { 122, 10 }, { 132, 0 },
};

static float s_val[CAL_COUNT];
static unsigned s_gen;
static bool s_nvs_ok;

static bool flags_are_integral(enum calib_id id)
{
    return id == CAL_AXIS_INVERT || id == CAL_SWAP_XY || id == CAL_ORIGIN_CENTER ||
           id == CAL_POWER_MODE || id == CAL_AUTO_ARM || id == CAL_GATE_ACTIVE_LOW ||
           id == CAL_AUTO_GUIDE ||
           id == CAL_PD_PERIOD;
}

void calib_init(void)
{
    esp_err_t err = nvs_flash_init();
    nvs_handle_t h;
    int i;

    for (i = 0; i < CAL_COUNT; i++)
        s_val[i] = s_defs[i].def;

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s, settings will not persist",
                 esp_err_to_name(err));
        return;
    }
    s_nvs_ok = true;

    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK)
        return; /* nothing stored yet - defaults */

    for (i = 0; i < CAL_COUNT; i++) {
        float v;
        size_t sz = sizeof(v);

        if (!s_defs[i].key)
            continue;
        if (nvs_get_blob(h, s_defs[i].key, &v, &sz) != ESP_OK || sz != sizeof(v))
            continue;
        /* A stored value outside the current valid range (e.g. from an
         * older firmware where the key meant something slightly different)
         * falls back to the default rather than being trusted. */
        if (v >= s_defs[i].min && v <= s_defs[i].max)
            s_val[i] = v;
        else
            ESP_LOGW(TAG, "stored %s=%g out of range, using default %g",
                     s_defs[i].key, (double)v, (double)s_defs[i].def);
    }
    nvs_close(h);
}

float calib_get(enum calib_id id)
{
    if ((unsigned)id >= CAL_COUNT)
        return 0.0f;
    return s_val[id];
}

static void save(enum calib_id id)
{
    nvs_handle_t h;

    if (!s_nvs_ok || !s_defs[id].key)
        return;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open (write) failed for '%s'", s_defs[id].key);
        return;
    }
    nvs_set_blob(h, s_defs[id].key, &s_val[id], sizeof(s_val[id]));
    nvs_commit(h);
    nvs_close(h);
}

int calib_set_param(int number, float value)
{
    int i;

    for (i = 0; i < CAL_COUNT; i++) {
        const struct calib_def *d = &s_defs[i];

        if (d->number != number && d->alias != number)
            continue;
        if (!d->key)
            return 3;
        /* $110/$111 are written by senders in mm/min like GRBL, everything
         * else is in the unit documented in calib.h. */
        if (value != value || value < d->min || value > d->max)
            return 3;
        if (flags_are_integral((enum calib_id)i))
            value = (float)(int)(value + 0.5f);
        s_val[i] = value;
        save((enum calib_id)i);
        s_gen++;
        return 0;
    }
    return 3;
}

static void fmt_value(char *buf, size_t len, int number, float v)
{
    /* Integral values print without decimals (GRBL style). Always plain
     * numbers, never exponent form: senders parse \$(\d+)=([\d\.-]+). */
    if (v == (float)(long)v && v < 1e9f && v > -1e9f) {
        snprintf(buf, len, "$%d=%ld\r\n", number, (long)v);
    } else {
        /* ~7 significant digits (what a float holds), trailing zeros
         * trimmed. A fixed %.4f printed small values such as the radial k1
         * (~1e-6) as 0.0000, losing the calibration on readback. */
        float a = v < 0.0f ? -v : v;
        int dec = 6;
        char *p;
        int n;

        while (a >= 10.0f && dec > 0) {
            a /= 10.0f;
            dec--;
        }
        while (a < 1.0f && a > 0.0f && dec < 12) {
            a *= 10.0f;
            dec++;
        }
        n = snprintf(buf, len, "$%d=%.*f", number, dec, (double)v);
        if (n > 0 && (size_t)n < len && dec > 0) {
            p = buf + n - 1;
            while (*p == '0')
                *p-- = '\0';
            if (*p == '.')
                *p = '\0';
        }
        strncat(buf, "\r\n", len - strlen(buf) - 1);
    }
}

void calib_dump(void (*write_line)(const char *line))
{
    char buf[48];
    size_t ci = 0;
    int i;

    /* Merge the read-only constants and the live table in ascending "$"
     * order so the dump looks like stock GRBL's. */
    for (int number = 0; number <= 255; number++) {
        bool printed = false;

        while (ci < sizeof(s_consts) / sizeof(s_consts[0]) && s_consts[ci].number < number)
            ci++;
        if (ci < sizeof(s_consts) / sizeof(s_consts[0]) && s_consts[ci].number == number) {
            fmt_value(buf, sizeof(buf), number, s_consts[ci].value);
            write_line(buf);
            continue;
        }
        for (i = 0; i < CAL_COUNT && !printed; i++) {
            float v = s_val[i];

            if (s_defs[i].number == number) {
                fmt_value(buf, sizeof(buf), number, v);
                write_line(buf);
                printed = true;
            } else if (s_defs[i].alias == number) {
                fmt_value(buf, sizeof(buf), number, v);
                write_line(buf);
                printed = true;
            }
        }
    }
}

unsigned calib_generation(void)
{
    return s_gen;
}
