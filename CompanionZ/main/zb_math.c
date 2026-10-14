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
    return (int32_t)lroundf(mm * spm);
}

float zb_steps_to_mm(int32_t steps, float spm)
{
    return spm > 0.0f ? (float)steps / spm : 0.0f;
}
