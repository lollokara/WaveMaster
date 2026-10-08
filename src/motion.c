/*
 * Galvo trajectory generator - see motion.h for the contract.
 * Pure C99, float math only in the per-tick paths.
 */
#include "motion.h"
#include <math.h>
#include <string.h>

#define MARK_MIN_LEN_MM   1e-5f
#define JUMP_MIN_LEN_MM   1e-4f
#define SNAP_EPS          1e-3f   /* fraction of a tick treated as "on the vertex" */
#define RAD2DEG           57.29577951f
#define TWO_PI_F          6.28318531f

static inline void put(struct motion_gen *g, float x, float y, uint8_t p)
{
    g->x = x;
    g->y = y;
    g->ticks_emitted++;
    g->sink.tick(g->sink.ctx, x, y, p);
}

static inline uint32_t hold_ticks(const struct motion_gen *g, float us)
{
    if (!(us > 0.0f) || !(g->p.tick_us > 0.0f))
        return 0;
    return (uint32_t)lroundf(us / g->p.tick_us);
}

static void hold(struct motion_gen *g, uint32_t n, uint8_t power)
{
    float x = g->x, y = g->y;
    while (n--)
        put(g, x, y, power);
}

void motion_gen_init(struct motion_gen *g, const struct motion_params *p,
                     const struct motion_sink *sink, float x0, float y0)
{
    memset(g, 0, sizeof(*g));
    g->p = *p;
    g->sink = *sink;
    motion_gen_reset(g, x0, y0);
}

void motion_gen_set_params(struct motion_gen *g, const struct motion_params *p)
{
    g->p = *p;
}

bool motion_gen_has_pending(const struct motion_gen *g)
{
    return g->have_pending;
}

void motion_gen_reset(struct motion_gen *g, float x, float y)
{
    g->x = g->vx = g->tail_x = x;
    g->y = g->vy = g->tail_y = y;
    g->have_pending = false;
    g->in_polyline = false;
    g->carry_t = 0.0f;
    g->path_s_mm = 0.0f;
    g->last_ux = g->last_uy = 0.0f;
    g->last_power = 0;
}

/* ------------------------------------------------------------------ jump */

static void emit_jump(struct motion_gen *g, const struct motion_seg *s, bool chained)
{
    const float x0 = g->x, y0 = g->y;
    const float dx = s->x - x0, dy = s->y - y0;
    const float L = sqrtf(dx * dx + dy * dy);
    if (L < JUMP_MIN_LEN_MM)
        return;

    const float tick_s = g->p.tick_us * 1e-6f;
    float v = g->p.jump_speed_mm_s > 1.0f ? g->p.jump_speed_mm_s : 1.0f;
    const float a = g->p.jump_accel_mm_s2;
    float ta = 0.0f, tc = 0.0f, T;   /* accel time, cruise time, total */

    if (a <= 0.0f) {
        T = L / v;
    } else {
        float dr = v * v / (2.0f * a);
        if (L >= 2.0f * dr) {
            ta = v / a;
            tc = (L - 2.0f * dr) / v;
        } else {
            ta = sqrtf(L / a);
            v = a * ta;
            tc = 0.0f;
        }
        T = 2.0f * ta + tc;
    }

    int32_t n = (int32_t)ceilf(T / tick_s - 1e-3f);
    if (n < 1)
        n = 1;
    const float dt = T / (float)n;
    const float inv_L = 1.0f / L;
    const float dr = (a > 0.0f) ? 0.5f * a * ta * ta : 0.0f;

    for (int32_t k = 1; k <= n; k++) {
        if (k == n) {
            put(g, s->x, s->y, 0);
            break;
        }
        float t = (float)k * dt, d;
        if (a <= 0.0f) {
            d = L * t / T;
        } else if (t < ta) {
            d = 0.5f * a * t * t;
        } else if (t < ta + tc) {
            d = dr + v * (t - ta);
        } else {
            float r = T - t;
            d = L - 0.5f * a * r * r;
        }
        float f = d * inv_L;
        put(g, x0 + dx * f, y0 + dy * f, 0);
    }

    if (!chained)
        hold(g, hold_ticks(g, g->p.jump_delay_us + g->p.jump_delay_per_mm_us * L), 0);
}

/* ------------------------------------------------------------------ mark */

static void emit_mark(struct motion_gen *g, const struct motion_seg *s,
                      const struct motion_seg *next)
{
    if (!g->in_polyline) {
        g->vx = g->x;
        g->vy = g->y;
        g->carry_t = 0.0f;
        g->path_s_mm = 0.0f;
    }
    const float sx = g->vx, sy = g->vy;
    const float dx = s->x - sx, dy = s->y - sy;
    const float L = sqrtf(dx * dx + dy * dy);
    const bool next_mark = next && next->type == MOTION_MARK;

    if (L < MARK_MIN_LEN_MM) {
        /* Degenerate (normally filtered in push): nothing to travel. */
        g->vx = s->x;
        g->vy = s->y;
        goto after_segment;
    }

    const float ux = dx / L, uy = dy / L;
    float vmm = s->speed_mm_s > 1.0f ? s->speed_mm_s : 1.0f;
    const float step = vmm * g->p.tick_us * 1e-6f;
    const uint8_t pw = s->power;

    /* Polygon delay decision (needs the outgoing direction). */
    uint32_t poly_n = 0;
    if (next_mark && g->p.polygon_delay_us > 0.0f) {
        float nx = next->x - s->x, ny = next->y - s->y;
        float nl = sqrtf(nx * nx + ny * ny);
        if (nl > 0.0f) {
            float c = (ux * nx + uy * ny) / nl;
            if (c > 1.0f) c = 1.0f;
            if (c < -1.0f) c = -1.0f;
            float ang = acosf(c) * RAD2DEG;
            if (ang >= g->p.polygon_angle_deg && ang > 0.0f)
                poly_n = hold_ticks(g, g->p.polygon_delay_us * ang / 180.0f);
        }
    }
    const bool exact_end = !next_mark || poly_n > 0;

    const float d0 = (1.0f - g->carry_t) * step;     /* first sample distance */
    int32_t n = 0;
    if (L - d0 >= -SNAP_EPS * step)
        n = (int32_t)floorf((L - d0) / step + SNAP_EPS) + 1;
    const float d_last = d0 + (float)(n - 1) * step;
    const bool snapped = (n > 0) && (L - d_last < SNAP_EPS * step);

    const bool wob = g->p.wobble_diameter_mm > 0.0f && g->p.wobble_pitch_mm > 0.0f;
    const float wr = 0.5f * g->p.wobble_diameter_mm;
    const float wk = wob ? TWO_PI_F / g->p.wobble_pitch_mm : 0.0f;
    const float s0 = g->path_s_mm;

    for (int32_t k = 0; k < n; k++) {
        float x, y;
        if (k == n - 1 && snapped && exact_end) {
            x = s->x;
            y = s->y;
        } else {
            float d = d0 + (float)k * step;
            x = sx + ux * d;
            y = sy + uy * d;
            if (wob) {
                float phi = wk * (s0 + d);
                float cr = wr * (cosf(phi) - 1.0f), sr = wr * sinf(phi);
                x += cr * ux - sr * uy;
                y += cr * uy + sr * ux;
            }
        }
        put(g, x, y, pw);
    }

    if (snapped) {
        g->carry_t = 0.0f;
    } else {
        float c = (L - d_last) / step;   /* n == 0: d_last = -carry*step */
        g->carry_t = c < 0.0f ? 0.0f : (c > 1.0f ? 1.0f : c);
    }
    g->path_s_mm += L;
    g->vx = s->x;
    g->vy = s->y;
    g->last_ux = ux;
    g->last_uy = uy;
    g->last_power = pw;

    if (exact_end && !snapped)
        put(g, s->x, s->y, pw);          /* land exactly on the vertex */
    if (exact_end)
        g->carry_t = 0.0f;
    if (poly_n > 0)
        hold(g, poly_n, pw);

after_segment:
    if (next_mark) {
        g->in_polyline = true;
    } else {
        g->in_polyline = false;
        g->carry_t = 0.0f;
        float us = g->p.mark_delay_us > g->p.laser_off_delay_us ?
                   g->p.mark_delay_us : g->p.laser_off_delay_us;
        hold(g, hold_ticks(g, us), 0);
    }
}

/* -------------------------------------------------------------- dispatch */

static void emit_pending(struct motion_gen *g, const struct motion_seg *next)
{
    const struct motion_seg s = g->pending;   /* copy: sink may block for long */
    switch (s.type) {
    case MOTION_JUMP:
        emit_jump(g, &s, next && next->type == MOTION_JUMP);
        break;
    case MOTION_MARK:
        emit_mark(g, &s, next);
        break;
    case MOTION_DWELL:
        hold(g, hold_ticks(g, s.dwell_us), 0);
        break;
    default:
        break;
    }
}

void motion_gen_push(struct motion_gen *g, const struct motion_seg *seg)
{
    if (seg->type == MOTION_MARK || seg->type == MOTION_JUMP) {
        float dx = seg->x - g->tail_x, dy = seg->y - g->tail_y;
        float l2 = dx * dx + dy * dy;
        float m = seg->type == MOTION_MARK ? MARK_MIN_LEN_MM : JUMP_MIN_LEN_MM;
        if (l2 < m * m)
            return;                     /* zero-length: nothing to do */
        g->tail_x = seg->x;
        g->tail_y = seg->y;
    } else if (seg->type != MOTION_DWELL) {
        return;
    }

    if (g->have_pending)
        emit_pending(g, seg);
    g->pending = *seg;
    g->have_pending = true;
}

void motion_gen_finish(struct motion_gen *g)
{
    if (g->have_pending) {
        g->have_pending = false;
        emit_pending(g, NULL);
    }
    g->in_polyline = false;
    g->carry_t = 0.0f;
    g->tail_x = g->x;
    g->tail_y = g->y;
}
