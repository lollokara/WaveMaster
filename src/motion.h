#ifndef MOTION_H_
#define MOTION_H_

/*
 * Galvo trajectory generator - pure C, no ESP-IDF dependencies, so it can be
 * unit-tested on the host (see test/host/).
 *
 * Turns a stream of motion segments (jumps, marks, dwells) into a stream of
 * fixed-period "ticks": one (x, y, power) sample every tick_us microseconds.
 * The output stage (galvo_out.c) writes exactly one DAC point per tick, so
 * the tick stream IS the galvo's commanded trajectory, sample by sample.
 *
 * Why ticks instead of "move to the endpoint as fast as possible": a galvo
 * commanded straight from A to B arrives almost instantly, then sits at B.
 * With the laser on, the beam dwells at the endpoints and races through the
 * middle - corners burn, lines are faint. Sampling the path at a constant
 * speed puts the same amount of time (and so energy) on every millimetre.
 *
 * Semantics (modelled on SCANLAB / EZCAD marking cards):
 *
 *  MARK segments
 *   - Travel at constant speed (the segment's speed, mm/s). No acceleration
 *     ramp: the speed is constant across segment boundaries and the corner
 *     between two marks is passed at full speed (the galvo servo rounds it
 *     by its own tracking lag). This is deliberate - slowing down at
 *     corners without also lowering power would overburn them.
 *   - Arc-length sampling: the distance travelled per tick is exactly
 *     speed * tick_us, carried across segment boundaries, so many tiny
 *     segments (linearised curves) still produce perfectly uniform motion.
 *   - Consecutive MARK segments form one "polyline" regardless of power.
 *     A MARK with power 0 is a constant-speed move with the laser off
 *     (raster overscan, gaps in a scanline) - it does NOT end the polyline.
 *   - Polygon delay: at a vertex whose direction change exceeds
 *     polygon_angle_deg, the path lands exactly on the vertex and holds
 *     there for polygon_delay_us * (turn_angle / 180deg) with power
 *     unchanged.
 *   - End of polyline (next segment is a JUMP/DWELL, or motion_gen_finish()):
 *     the last tick lands exactly on the end point, then the position is
 *     held for max(mark_delay_us, laser_off_delay_us) with power 0 emitted
 *     for those hold ticks. (The output stage delays the laser-off edge by
 *     laser_off_delay_us, so the laser really turns off inside this hold.)
 *
 *  JUMP segments
 *   - Laser off (power 0 on every tick).
 *   - Trapezoidal velocity profile, jump_speed_mm_s cruise and
 *     jump_accel_mm_s2 accel/decel, starting and ending at rest. If
 *     jump_accel_mm_s2 <= 0, constant speed (no ramp). The sample times are
 *     stretched slightly so the final tick lands exactly on the target.
 *   - Followed by a hold of jump_delay_us + jump_delay_per_mm_us * length,
 *     UNLESS the next segment is also a JUMP (consecutive jumps chain; only
 *     the last one gets the settle delay). If motion_gen_finish() is called
 *     after a jump, the delay is still emitted.
 *   - Zero-length jumps (< 1e-4 mm) emit nothing.
 *
 *  DWELL segments
 *   - Hold the current position for dwell_us with power 0.
 *
 * Laser on/off delays are NOT applied here: the generator emits the
 * *intended* power per tick, aligned with position. galvo_out.c delays the
 * physical gate edges (rising by laser_on_delay, falling by
 * laser_off_delay). The generator only guarantees the end-of-polyline hold
 * above is long enough for the delayed falling edge.
 *
 * Wobble (wobble_diameter_mm > 0 and wobble_pitch_mm > 0): on MARK ticks
 * the position gets a circular offset that advances one revolution per
 * wobble_pitch_mm of path distance:
 *     p + r*(cos(phi) - 1)*u + r*sin(phi)*n,  phi = 2*pi*s/pitch, r = d/2
 * where u is the unit direction of travel, n its left normal and s the
 * distance travelled along the polyline. It starts and ends on the path.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

enum motion_seg_type {
    MOTION_JUMP = 0,
    MOTION_MARK = 1,
    MOTION_DWELL = 2,
};

struct motion_seg {
    uint8_t type;        /* enum motion_seg_type */
    uint8_t power;       /* MARK only: 0..255 laser power word, 0 = laser off */
    float x, y;          /* JUMP/MARK target, machine mm */
    float speed_mm_s;    /* MARK only: constant marking speed, > 0 */
    float dwell_us;      /* DWELL only */
};

struct motion_params {
    float tick_us;               /* sample period, > 0 (actual hardware period) */
    float jump_speed_mm_s;       /* > 0 */
    float jump_accel_mm_s2;      /* <= 0: no ramp */
    float jump_delay_us;
    float jump_delay_per_mm_us;
    float mark_delay_us;
    float laser_off_delay_us;    /* only used to size the end-of-polyline hold */
    float polygon_delay_us;      /* at a 180 deg reversal; scaled by angle */
    float polygon_angle_deg;     /* turns smaller than this get no delay */
    float wobble_diameter_mm;    /* 0 = off */
    float wobble_pitch_mm;
};

/* Output callbacks. tick() is called once per tick, in order. It may block
 * (backpressure from the output ring) - the generator is synchronous. */
struct motion_sink {
    void (*tick)(void *ctx, float x_mm, float y_mm, uint8_t power);
    void *ctx;
};

/* Opaque-ish generator state; fields are private to motion.c. Declared here
 * so callers can allocate it statically. */
struct motion_gen {
    struct motion_params p;
    struct motion_sink sink;
    float x, y;                 /* position of the last emitted tick */
    bool have_pending;          /* one-segment look-ahead slot is full */
    struct motion_seg pending;
    /* MARK polyline state */
    bool in_polyline;
    float carry_mm;             /* distance into the current segment already covered */
    float path_s_mm;            /* distance along the polyline (wobble phase) */
    float last_ux, last_uy;     /* direction of the previous mark segment */
    uint8_t last_power;
    uint64_t ticks_emitted;     /* statistics */
};

void motion_gen_init(struct motion_gen *g, const struct motion_params *p,
                     const struct motion_sink *sink, float x0, float y0);

/* Replace parameters (takes effect for segments not yet emitted). */
void motion_gen_set_params(struct motion_gen *g, const struct motion_params *p);

/* Feed one segment. Because of the one-segment look-ahead, this emits the
 * ticks for the *previous* segment (it now knows what follows it). */
void motion_gen_push(struct motion_gen *g, const struct motion_seg *seg);

/* No more segments are coming for now: emit the pending segment as the end
 * of its polyline / jump chain (end-of-polyline hold or jump delay
 * included). Leaves the generator at rest; a later push starts afresh. */
void motion_gen_finish(struct motion_gen *g);

/* True if a segment is held in the look-ahead slot (motion is "open"). */
bool motion_gen_has_pending(const struct motion_gen *g);

/* Drop the look-ahead slot and polyline state without emitting anything,
 * and set the current position (used after an abort). */
void motion_gen_reset(struct motion_gen *g, float x, float y);

#endif /* MOTION_H_ */
