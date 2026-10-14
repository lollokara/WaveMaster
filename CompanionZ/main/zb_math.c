#include "zb_math.h"

#include <math.h>

void zb_gen_init(struct zb_gen *g, int32_t total, float vs, float vc, float a, float d)
{
    if (vs > vc) vs = vc;
    if (vs < 1.0f) vs = 1.0f;
    if (a < 1.0f) a = 1.0f;
    if (d < 1.0f) d = 1.0f;
    g->v = vs; g->vs = vs; g->vc = vc; g->a = a; g->d = d;
    g->n = total; g->i = 0;
}

static float brake_steps(const struct zb_gen *g)
{
    float x = (g->v * g->v - g->vs * g->vs) / (2.0f * g->d);
    return x < 0.0f ? 0.0f : x;
}

/* Second-order term so v^2 changes by exactly 2*acc per step; skipped when the step is huge. */
static float corr(float acc, float v)
{
    float r = acc / (v * v);
    return r < 0.4f ? 0.5f * r : 0.0f;
}

float zb_gen_next(struct zb_gen *g)
{
    float out = g->v;
    float rem = (float)(g->n - g->i - 1);
    if (rem <= brake_steps(g)) {
        g->v -= g->d / g->v * (1.0f + corr(g->d, g->v));
        if (g->v < g->vs) g->v = g->vs;
    } else if (g->v < g->vc) {
        g->v += g->a / g->v * (1.0f - corr(g->a, g->v));
        if (g->v > g->vc) g->v = g->vc;
    } else {
        g->v = g->vc;
    }
    g->i++;
    return out;
}

void zb_gen_truncate_to_stop(struct zb_gen *g)
{
    int32_t k = (int32_t)ceilf(brake_steps(g));
    if (g->i + k < g->n) g->n = g->i + k;
}

float zb_rate_clamp(float rate, int pulse_us)
{
    if (pulse_us < 1) pulse_us = 1;
    float hi = 1.0e6f / (2.0f * (float)pulse_us);
    if (hi > 100000.0f) hi = 100000.0f;
    if (rate > hi) rate = hi;
    if (rate < 35.0f) rate = 35.0f;
    return rate;
}

uint32_t zb_period_ticks(float rate, int pulse_us)
{
    if (pulse_us < 1) pulse_us = 1;
    uint32_t t = (uint32_t)(1.0e6f / rate + 0.5f);
    if (t < 2u * (uint32_t)pulse_us) t = 2u * (uint32_t)pulse_us;
    if (t > 32000u) t = 32000u;
    return t;
}

int32_t zb_mm_to_steps(float mm, float spm)
{
    float v = mm * spm;
    if (!(v == v)) return 0;                       /* NaN */
    if (v >= 1.0e9f) return 1000000000;            /* saturate: the float -> int32 cast must never overflow */
    if (v <= -1.0e9f) return -1000000000;
    return (int32_t)lroundf(v);
}

void zb_soft_limits(int32_t max_steps, int32_t back, bool home_to_max, int32_t *lo, int32_t *hi)
{
    if (back < 0) back = 0;
    if (back > max_steps / 2) back = max_steps / 2;
    if (lo) *lo = home_to_max ? 0 : back;
    if (hi) *hi = home_to_max ? max_steps - back : max_steps;
}

int zb_plan_move(float target_mm, bool relative, bool homed, int32_t pos, float spm,
                 float max_range_mm, float backoff_mm, bool home_to_max, int32_t *delta)
{
    if (!isfinite(target_mm) || !(spm > 0.0f) || !(max_range_mm > 0.0f)) return 4;
    if (relative ? fabsf(target_mm) > max_range_mm
                 : (target_mm < 0.0f || target_mm > max_range_mm)) return 3;
    int64_t d = zb_mm_to_steps(target_mm, spm);
    int64_t tgt = relative ? (int64_t)pos + d : d;
    if (homed) {
        int32_t lo, hi;
        zb_soft_limits(zb_mm_to_steps(max_range_mm, spm), zb_mm_to_steps(backoff_mm, spm), home_to_max, &lo, &hi);
        if (tgt < lo || tgt > hi) return 3;
    } else if (tgt > 1000000000 || tgt < -1000000000) {
        return 3;
    }
    if (delta) *delta = (int32_t)(tgt - pos);
    return 0;
}

float zb_home_speed_limit(float decel, float backoff)
{
    if (!(decel > 0.0f) || !(backoff > 0.0f)) return 0.0f;
    return sqrtf(2.0f * decel * backoff);
}

float zb_steps_to_mm(int32_t steps, float spm)
{
    return spm > 0.0f ? (float)steps / spm : 0.0f;
}
