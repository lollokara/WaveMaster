#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_

#include <stdbool.h>

/*
 * Real look-ahead motion planner for marking (G1) runs, replacing
 * constant-velocity feed pacing with GRBL-style trapezoidal velocity
 * profiles and junction-deviation cornering across a small buffer of
 * pending segments - the biggest remaining gap versus a real GRBL/
 * EZCAD-class controller identified in STATUS.md.
 *
 * Scope, stated up front (see STATUS.md for the full writeup):
 * - Applies to plain G1 marking runs only (grbl_task.c's process_gcode_line,
 *   non-repeat, non-fill case). Arcs, hatch-fill lines, and repeat-count
 *   (R word) marks stay on the pre-existing immediate mark_stroke() path -
 *   documented, not an oversight.
 * - This is a *batch* look-ahead planner, not a continuously sliding
 *   window like real GRBL: it buffers up to MOTION_PLANNER_DEPTH segments,
 *   then (on overflow or an explicit flush) runs one reverse+forward pass
 *   over the whole currently-buffered set and executes all of them in
 *   order. A "soft" flush (buffer overflow, more marking still coming)
 *   carries the batch's final exit speed forward as the next batch's
 *   entry-speed anchor, so there's no unnecessary stop at that boundary.
 *   A "hard" flush (motion_planner_flush() - called before anything that
 *   isn't a continuation of marking motion: G0, arcs, fill, dwell,
 *   program end, mode changes) always decelerates to a full stop and
 *   resets the anchor to 0, which is the physically correct behavior
 *   there. The practical difference from real GRBL's sliding window: a
 *   segment near the tail of a *soft*-flushed batch can't see into the
 *   next batch's shape yet, so cornering exactly at that boundary can be
 *   slightly more conservative than optimal - never incorrect/unsafe,
 *   just occasionally not maximally fast. With MOTION_PLANNER_DEPTH=16
 *   this only matters for very long straight runs of marking G-code.
 * - Junction-deviation formula is independently derived here (see
 *   motion_planner.c), not a byte-for-byte port of GRBL's planner.c.
 * - Trapezoid shape per segment (accel/cruise/decel) uses the standard
 *   v^2=v0^2+2*a*d kinematics; if a segment is too short to reach the
 *   nominal/junction-limited cruise speed, it degrades to a single
 *   accel-or-decel ramp (no cruise plateau) - approximated, not an exact
 *   S-curve or jerk-limited profile.
 */

/* One finalized, ready-to-execute segment - the planner has already
 * decided its entry/cruise/exit speeds; the caller (grbl_task.c's
 * execute callback) just needs to time-sample the path accordingly. */
struct motion_block_result {
    float x0_mm, y0_mm, x1_mm, y1_mm;
    float entry_speed_mm_s;
    float cruise_speed_mm_s;
    float exit_speed_mm_s;
    float accel_mm_s2;
    bool gate_on_power;
    /* True only for the very first segment executed since the last hard
     * flush (motion_planner_flush()) - the natural point to apply a
     * skywriting lead-in, since the galvo is coming from a genuine stop. */
    bool is_first_in_sequence;
    /* True only for the last segment executed by a hard flush (never true
     * for a soft/overflow flush, since more marking is still coming right
     * after) - the natural point to apply a skywriting lead-out. */
    bool is_last_in_sequence;
};

/* execute() is called synchronously, once per finalized segment, in path
 * order, from within motion_planner_enqueue() (on buffer overflow) and
 * motion_planner_flush(). It's expected to time-sample the segment's
 * velocity profile into DAC points and submit them (see grbl_task.c). */
void motion_planner_init(void (*execute)(const struct motion_block_result *block));

/* Live-updates the kinematic limits from calib.h's $180/$181/$182 - safe
 * to call at any time, including mid-sequence (affects only segments not
 * yet finalized/executed). */
void motion_planner_set_params(float max_accel_mm_s2, float junction_deviation_mm,
                                float max_velocity_mm_s);

/* Buffers one straight marking segment. feed_mm_per_min<=0 means "no
 * explicit feed given" - treated as the configured max velocity (i.e.
 * unpaced/as-fast-as-the-planner-allows), matching this firmware's prior
 * F=0 behavior for the non-planned path. May trigger a soft flush
 * internally if the buffer is full. */
void motion_planner_enqueue(float x0_mm, float y0_mm, float x1_mm, float y1_mm,
                            float feed_mm_per_min, bool gate_on_power);

/* Hard flush: finalizes and executes every currently-buffered segment,
 * decelerating to a full stop by the last one, and resets the planner's
 * speed-continuity anchor to 0. Call this before anything that isn't a
 * continuation of marking motion (see the scope note above). A no-op if
 * nothing is buffered. */
void motion_planner_flush(void);

/* Discards every currently-buffered segment WITHOUT executing them (unlike
 * motion_planner_flush()) and resets the continuity anchor to 0. Use this
 * for a genuine abort (e.g. Ctrl-X soft reset), where pending motion
 * should be cancelled rather than completed. */
void motion_planner_clear(void);

#endif /* MOTION_PLANNER_H_ */
