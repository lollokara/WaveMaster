#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "zb_math.h"
#include "zb_setcore.h"

static int fails, checks;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

/* Run a whole move; returns time in s, checks bounds. */
static double run_move(int32_t n, float vs, float vc, float a, float d, float *vmax, float *vend, int32_t trunc_at)
{
    struct zb_gen g;
    zb_gen_init(&g, n, vs, vc, a, d);
    double t = 0;
    float mx = 0, last = 0;
    int32_t steps = 0;
    while (g.i < g.n) {
        if (trunc_at >= 0 && g.i == trunc_at) zb_gen_truncate_to_stop(&g);
        if (g.i >= g.n) break;
        float v = zb_gen_next(&g);
        if (v > mx) mx = v;
        t += 1.0 / v;
        last = v;
        steps++;
    }
    *vmax = mx; *vend = last;
    if (trunc_at < 0) CHECK(steps == n);
    return t;
}

static void test_profile(void)
{
    float vmax, vend;
    /* long move: reaches vc, ends near vs, time close to analytic trapezoid */
    float vs = 200, vc = 8000, a = 40000, d = 40000;
    int32_t n = 20000;
    double t = run_move(n, vs, vc, a, d, &vmax, &vend, -1);
    double n_acc = (vc * vc - vs * vs) / (2 * a), n_dec = (vc * vc - vs * vs) / (2 * d);
    double ta = (vc - vs) / a, td = (vc - vs) / d, tc = (n - n_acc - n_dec) / vc;
    CHECK(fabs(vmax - vc) < 1);
    CHECK(vend < vs * 1.6f);
    CHECK(fabs(t - (ta + td + tc)) / (ta + td + tc) < 0.02);

    /* asymmetric accel/decel */
    t = run_move(n, vs, vc, 20000, 80000, &vmax, &vend, -1);
    n_acc = (vc * vc - vs * vs) / (2 * 20000.0); n_dec = (vc * vc - vs * vs) / (2 * 80000.0);
    ta = (vc - vs) / 20000.0; td = (vc - vs) / 80000.0; tc = (n - n_acc - n_dec) / vc;
    CHECK(fabs(t - (ta + td + tc)) / (ta + td + tc) < 0.02);

    /* triangle: short move never reaches vc, peak matches formula */
    n = 400;
    t = run_move(n, vs, vc, a, d, &vmax, &vend, -1);
    float vp = sqrtf(vs * vs + 2.0f * n * a * d / (a + d));
    CHECK(vmax < vc && fabs(vmax - vp) / vp < 0.05);

    /* tiny moves */
    for (int32_t k = 1; k < 40; k++) run_move(k, vs, vc, a, d, &vmax, &vend, -1);

    /* constant speed (probe): vs == vc */
    t = run_move(1000, 400, 400, 1e6f, 1e6f, &vmax, &vend, -1);
    CHECK(fabs(t - 2.5) < 0.01 && vmax <= 400.01f);

    /* truncate during cruise: stops within the braking distance, ends slow */
    struct zb_gen g;
    zb_gen_init(&g, 100000, vs, vc, a, d);
    while (g.i < 5000) zb_gen_next(&g);
    zb_gen_truncate_to_stop(&g);
    int32_t extra = g.n - g.i;
    CHECK(extra > 0 && fabs(extra - (vc * vc - vs * vs) / (2 * d)) < 3);
    float last = 0;
    while (g.i < g.n) last = zb_gen_next(&g);
    CHECK(last < vs * 1.6f);

    /* truncate during acceleration */
    run_move(100000, vs, vc, a, d, &vmax, &vend, 300);
    CHECK(vend < vs * 1.8f);
    /* truncate at standstill: no further steps */
    zb_gen_init(&g, 100, vs, vs, a, d);
    zb_gen_truncate_to_stop(&g);
    CHECK(g.n == 0);
}

static void test_math(void)
{
    CHECK(zb_mm_to_steps(1.0f, 400.0f) == 400);
    CHECK(zb_mm_to_steps(-0.5f, 400.0f) == -200);
    CHECK(zb_mm_to_steps(0.00126f, 400.0f) == 1);
    CHECK(fabsf(zb_steps_to_mm(400, 400.0f) - 1.0f) < 1e-6f);
    CHECK(zb_period_ticks(100000.0f, 2) == 10);
    CHECK(zb_period_ticks(1e9f, 2) == 4);
    CHECK(zb_period_ticks(1.0f, 2) == 32000);
    CHECK(zb_rate_clamp(1e7f, 2) == 100000.0f);
    CHECK(zb_rate_clamp(1e7f, 20) == 25000.0f);
    CHECK(zb_rate_clamp(1.0f, 2) == 35.0f);
}

static void defaults(float *v) { for (int i = 0; i < ZS_COUNT; i++) v[i] = zbc_defs[i].def; }

static void test_settings(void)
{
    float v[ZS_COUNT];
    defaults(v);
    for (int i = 0; i < ZS_COUNT; i++) {
        CHECK(strlen(zbc_defs[i].key) > 0 && strlen(zbc_defs[i].key) <= 15);
        CHECK(zbc_defs[i].group && zbc_defs[i].label);
        CHECK(zbc_validate(v, (enum zb_set_id)i, zbc_defs[i].def) == 0);
        for (int j = 0; j < i; j++) CHECK(strcmp(zbc_defs[i].key, zbc_defs[j].key) != 0);
    }
    CHECK(zbc_validate(v, ZS_MAX_SPEED, 0.0f) == 4);
    CHECK(zbc_validate(v, ZS_MAX_SPEED, 100.0f) == 0);
    CHECK(zbc_validate(v, ZS_MAX_SPEED, 100.5f) == 4);
    CHECK(zbc_validate(v, ZS_MAX_SPEED, NAN) == 4);
    CHECK(zbc_validate(v, ZS_DIR_INVERT, 2.0f) == 4);
    CHECK(zbc_validate(v, ZS_DIR_INVERT, 1.0f) == 0);
    CHECK(zbc_validate(v, ZS_MICROSTEP, 8.0f) == 0);
    CHECK(zbc_validate(v, ZS_MICROSTEP, 3.0f) == 4);
    /* calibrated limits cap the working values */
    v[ZS_LIM_SPEED] = 30; v[ZS_LIM_ACCEL] = 500; v[ZS_LIM_DECEL] = 800; v[ZS_LIM_HOME_FAST] = 5;
    CHECK(zbc_validate(v, ZS_MAX_SPEED, 30.0f) == 0);
    CHECK(zbc_validate(v, ZS_MAX_SPEED, 31.0f) == 4);
    CHECK(zbc_validate(v, ZS_ACCEL, 501.0f) == 4);
    CHECK(zbc_validate(v, ZS_DECEL, 800.0f) == 0);
    CHECK(zbc_validate(v, ZS_HOME_FAST, 6.0f) == 4);
    CHECK(zbc_validate(v, ZS_HOME_SLOW, 6.0f) == 0);
    float lo, hi;
    zbc_range(v, ZS_MAX_SPEED, &lo, &hi);
    CHECK(hi == 30.0f && lo == 0.1f);
    zbc_range(v, ZS_MAX_RANGE, &lo, &hi);
    CHECK(hi == 500.0f);

    CHECK(zbc_hostname_valid("zbro"));
    CHECK(zbc_hostname_valid("z-bro-2"));
    CHECK(!zbc_hostname_valid(""));
    CHECK(!zbc_hostname_valid("-bad"));
    CHECK(!zbc_hostname_valid("has space"));
    CHECK(!zbc_hostname_valid("0123456789012345678901234567890123"));

    /* JSON: right size, balanced, grouped, every key present once */
    size_t need = zbc_json(v, NULL, 0);
    char *buf = malloc(need + 1);
    CHECK(zbc_json(v, buf, need + 1) == need && strlen(buf) == need);
    int br = 0, sq = 0, inq = 0;
    for (const char *p = buf; *p; p++) {
        if (*p == '"') inq = !inq;
        if (inq) continue;
        br += (*p == '{') - (*p == '}'); sq += (*p == '[') - (*p == ']');
        CHECK(br >= 0 && sq >= 0);
    }
    CHECK(br == 0 && sq == 0 && !inq);
    for (int i = 0; i < ZS_COUNT; i++) {
        char k[40];
        snprintf(k, sizeof k, "\"key\":\"%s\"", zbc_defs[i].key);
        CHECK(strstr(buf, k) != NULL);
    }
    CHECK(strstr(buf, "\"name\":\"Motion\"") && strstr(buf, "\"name\":\"Homing\"") &&
          strstr(buf, "\"name\":\"Driver\"") && strstr(buf, "\"name\":\"Calibration\""));
    CHECK(strstr(buf, "\"options\":[1,2,4,8,16]") != NULL);
    CHECK(strstr(buf, "\"key\":\"max_speed\",\"label\":\"Max speed\",\"value\":20,\"min\":0.1,\"max\":30") != NULL);
    /* truncated buffer must not overflow and still reports the full size */
    char small[16];
    CHECK(zbc_json(v, small, sizeof small) == need && strlen(small) == sizeof small - 1);
    free(buf);
}

int main(void)
{
    test_profile();
    test_math();
    test_settings();
    printf("%d checks, %d failures\n", checks, fails);
    if (fails == 0) printf("zbro motion tests passed\n");
    return fails != 0;
}
