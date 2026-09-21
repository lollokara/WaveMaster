#include "motion_planner.h"
#include <math.h>
#include <stddef.h>

/* 16 pending segments, matching the ballpark of real GRBL's default
 * planner buffer depth (its RX/planner buffers are commonly configured
 * around 15-35 blocks). Small enough that recomputing the whole buffer's
 * profile on every soft/hard flush (rather than an incremental sliding
 * update like real GRBL) is cheap - at most 16 iterations of simple
 * float math, nowhere near a performance concern on this hardware. */
#define MOTION_PLANNER_DEPTH 16

struct block {
    float x0, y0, x1, y1;
    float length;
    float ux, uy; /* unit direction vector, start->end */
    float nominal_speed; /* mm/s, from feed (or max velocity if unpaced) */
    float entry_speed;   /* mm/s, filled in by replan */
    float exit_speed;    /* mm/s, filled in by replan */
    bool gate_on_power;
};

static struct block s_buf[MOTION_PLANNER_DEPTH];
static size_t s_count = 0;
static float s_carry_exit_speed = 0.0f; /* speed continuity anchor across a soft flush */
static bool s_first_pending = true;     /* next executed block should mark is_first_in_sequence */

static float s_max_accel = 1000.0f;
static float s_junction_deviation = 0.01f;
static float s_max_velocity = 200.0f;

static void (*s_execute_cb)(const struct motion_block_result *);

void motion_planner_init(void (*execute)(const struct motion_block_result *))
{
    s_execute_cb = execute;
    s_count = 0;
    s_carry_exit_speed = 0.0f;
    s_first_pending = true;
}

void motion_planner_set_params(float max_accel_mm_s2, float junction_deviation_mm,
                                float max_velocity_mm_s)
{
    if (max_accel_mm_s2 > 0.0f)
        s_max_accel = max_accel_mm_s2;
    if (junction_deviation_mm > 0.0f)
        s_junction_deviation = junction_deviation_mm;
    if (max_velocity_mm_s > 0.0f)
        s_max_velocity = max_velocity_mm_s;
}

/* GRBL-style junction-deviation cornering speed, independently derived
 * (see motion_planner.h's header comment) rather than ported from GRBL's
 * planner.c. theta_interior is the interior angle of the path at the
 * joint between block a (incoming) and block b (outgoing): pi for a
 * straight continuation (no speed limit beyond each block's own nominal
 * speed), 0 for a full reversal/cusp (must stop). */
static float junction_speed(const struct block *a, const struct block *b)
{
    float cos_ti = -(a->ux * b->ux + a->uy * b->uy);
    float sin_half, denom, r, v, nominal_min;

    if (cos_ti > 1.0f) cos_ti = 1.0f;
    if (cos_ti < -1.0f) cos_ti = -1.0f;
    sin_half = sqrtf(fmaxf(0.0f, (1.0f - cos_ti) * 0.5f));
    nominal_min = fminf(a->nominal_speed, b->nominal_speed);

    if (sin_half > 0.9999f)
        return nominal_min; /* straight-through: no cornering limit */

    denom = 1.0f - sin_half;
    if (denom < 0.0001f)
        denom = 0.0001f;
    r = s_junction_deviation * sin_half / denom;
    v = sqrtf(s_max_accel * r);
    return fminf(v, nominal_min);
}

/* Max speed reachable after accelerating from v0 over distance d (or the
 * max speed that could decelerate down to v0 over distance d - same
 * formula, used both directions in the reverse pass below). */
static float speed_after_accel(float v0, float d)
{
    float v2 = v0 * v0 + 2.0f * s_max_accel * d;
    return v2 > 0.0f ? sqrtf(v2) : 0.0f;
}

/* Runs one reverse+forward planning pass over whatever's currently
 * buffered and executes every block in order via s_execute_cb(). hard
 * selects hard-flush semantics (decelerate to 0 by the last block, mark
 * it is_last_in_sequence, reset the carry-over anchor) vs soft (buffer
 * overflow - more marking still coming, carry the real exit speed
 * forward, never mark a block is_last_in_sequence). */
static void replan_and_execute(bool hard)
{
    size_t i;

    if (s_count == 0)
        return;

    /* Reverse pass: the last block's exit speed is conservatively 0 for a
     * hard flush (nothing scheduled after it) and also 0 for a soft flush
     * (we don't yet know what comes next in the following batch - the
     * batch-boundary conservatism documented in motion_planner.h). Each
     * block's entry speed is then the largest value that's simultaneously
     * reachable by decelerating from its own exit speed over its own
     * length, within the junction limit to its predecessor, and within
     * both blocks' nominal speeds. */
    s_buf[s_count - 1].exit_speed = 0.0f;
    for (i = s_count; i-- > 0; ) {
        float max_entry = fminf(s_buf[i].nominal_speed,
                                 speed_after_accel(s_buf[i].exit_speed, s_buf[i].length));

        if (i > 0) {
            float jv = junction_speed(&s_buf[i - 1], &s_buf[i]);

            max_entry = fminf(max_entry, jv);
            s_buf[i].entry_speed = max_entry;
            s_buf[i - 1].exit_speed = max_entry;
        } else {
            /* First buffered block: entry speed is also capped by
             * whatever speed the previous batch actually left us at
             * (continuity anchor) - 0 after a hard flush or at startup. */
            s_buf[i].entry_speed = fminf(max_entry, s_carry_exit_speed);
        }
    }

    /* Emit each block with its own trapezoid: entry -> peak (limited by
     * nominal speed and by what's reachable given the segment length) ->
     * exit. If the segment is too short to reach the peak implied by
     * entry/exit speeds and length, this degrades to a monotonic
     * accel-or-decel ramp (no cruise plateau) - see motion_planner.h. */
    for (i = 0; i < s_count; i++) {
        struct motion_block_result r;
        float v0 = s_buf[i].entry_speed;
        float v1 = s_buf[i].exit_speed;
        float peak2 = (2.0f * s_max_accel * s_buf[i].length + v0 * v0 + v1 * v1) * 0.5f;
        float cruise = peak2 > 0.0f ? sqrtf(peak2) : 0.0f;

        cruise = fminf(cruise, s_buf[i].nominal_speed);
        if (cruise < v0) cruise = v0;
        if (cruise < v1) cruise = v1;

        r.x0_mm = s_buf[i].x0;
        r.y0_mm = s_buf[i].y0;
        r.x1_mm = s_buf[i].x1;
        r.y1_mm = s_buf[i].y1;
        r.entry_speed_mm_s = v0;
        r.cruise_speed_mm_s = cruise;
        r.exit_speed_mm_s = v1;
        r.accel_mm_s2 = s_max_accel;
        r.gate_on_power = s_buf[i].gate_on_power;
        r.is_first_in_sequence = s_first_pending;
        s_first_pending = false;
        r.is_last_in_sequence = hard && (i == s_count - 1);

        if (s_execute_cb)
            s_execute_cb(&r);
    }

    s_carry_exit_speed = hard ? 0.0f : s_buf[s_count - 1].exit_speed;
    if (hard)
        s_first_pending = true;
    s_count = 0;
}

void motion_planner_enqueue(float x0_mm, float y0_mm, float x1_mm, float y1_mm,
                            float feed_mm_per_min, bool gate_on_power)
{
    struct block *b;
    float dx = x1_mm - x0_mm, dy = y1_mm - y0_mm;
    float len = sqrtf(dx * dx + dy * dy);

    if (len < 0.0001f)
        return; /* zero-length segment: nothing to plan or move */

    if (s_count >= MOTION_PLANNER_DEPTH)
        replan_and_execute(false); /* soft flush: free up a slot */

    b = &s_buf[s_count++];
    b->x0 = x0_mm;
    b->y0 = y0_mm;
    b->x1 = x1_mm;
    b->y1 = y1_mm;
    b->length = len;
    b->ux = dx / len;
    b->uy = dy / len;
    b->nominal_speed = fminf(feed_mm_per_min > 0.0f ? feed_mm_per_min / 60.0f : s_max_velocity,
                              s_max_velocity);
    b->gate_on_power = gate_on_power;
    b->entry_speed = 0.0f;
    b->exit_speed = 0.0f;
}

void motion_planner_flush(void)
{
    replan_and_execute(true);
}

void motion_planner_clear(void)
{
    s_count = 0;
    s_carry_exit_speed = 0.0f;
    s_first_pending = true;
}
