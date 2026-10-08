#include "grbl_task.h"
#include "laser_ctrl.h"
#include "laser_fill.h"
#include "dac_task.h"
#include "motion_planner.h"
#include "calib.h"
#include "atmega_link.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h> /* strncasecmp */
#include <ctype.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "grbl_task";

#define GRBL_TASK_STACK_SIZE 4096
#define GRBL_TASK_PRIORITY   9
#define GRBL_TASK_CORE       1
#define GRBL_LINE_MAX        128

/*
 * Word/M-code assignments (deviating from stock GRBL where noted, since
 * this hardware splits laser control across two controllers):
 *   S<0-1000>  laser power, GRBL's standard laser-mode convention -
 *              scaled to 0-100% and sent to the ATMEGA (atmega_link.h),
 *              which owns the actual power setpoint. NOT PRR.
 *   Q<hz>      pulse repetition rate in Hz, driven directly by this
 *              ESP32's own PWM output (prr_pwm.h) - custom, since S is
 *              taken by power. Placeholder range - adjust PRR_MAX_HZ to
 *              whatever the laser driver actually expects.
 *   F<mm/min>  feedrate, sticky (persists across lines, standard GRBL
 *              behavior). Plain (non-repeat) G1 marking moves go through
 *              the real look-ahead motion planner (src/motion_planner.h) -
 *              trapezoidal accel/decel ramps and GRBL-style junction-
 *              deviation cornering across a buffer of pending segments,
 *              not just constant-velocity pacing. F=0 (the default) means
 *              "no explicit feed" - the planner still applies (accel
 *              ramps, cornering), just capped at $182's max velocity
 *              instead of a requested speed. See STATUS.md for the exact
 *              scope (batch look-ahead, not a continuously sliding
 *              window; arcs/hatch-fill/repeat-count marks don't go
 *              through the planner - see below).
 *   R<n>       repeat count for this line's G0/G1 move - re-marks the same
 *              segment n times before advancing. Bypasses the look-ahead
 *              planner (re-marking one path doesn't benefit from cross-
 *              segment planning) - uses the older constant-pacing
 *              mark_stroke() path instead. Custom: repurposes the unused
 *              arc R-radius letter, since this firmware only supports the
 *              I/J center-offset arc form (R-radius form is not
 *              implemented) - see apply_arc().
 *   M62 / M63  guide (preview) laser on / off - repurposing GRBL's
 *              standard "turn on/off digital output" M-codes, sent to
 *              the ATMEGA. Only one output channel exists here, so the P
 *              word is accepted but not distinguished.
 *   M64 / M65  custom: start / end a fill region. Every G0/G1 X../Y..
 *              line between M64 and M65 is recorded as a polygon vertex
 *              instead of actually moving; M65 traces the polygon outline
 *              and generates a hatch fill inside it (see mark_stroke(),
 *              laser_fill_generate(), $170/$171 hatch settings).
 *   M68 P<hz>  custom: an alternative to M65 - loops just the recorded
 *              shape's outline continuously on-device at the given
 *              refresh rate (default 100Hz), instead of tracing it once
 *              and filling it. For watching a shape live (e.g. with the
 *              guide laser under M66 preview mode) at a rate far beyond
 *              what one-line-per-point serial G-code can sustain - see
 *              preview_loop_outline().
 *   M66 / M67  custom: red-light guide preview on / off. While on, the
 *              guide laser is forced on and the marking laser is forced
 *              off for all motion (G0/G1/G2/G3/fill), regardless of M3/M4
 *              intent, so an operator can trace the full job outline
 *              before actually firing.
 *   M10 / M11  arm / disarm the system - custom, sent to the ATMEGA
 *              (stock GRBL doesn't define these meaningfully).
 */
#define GRBL_S_MAX     1000.0f
#define PRR_MAX_HZ     50000.0f

/* Chord-error tolerance used to decide how many line segments approximate
 * a G2/G3 arc, mirroring the role of real GRBL's $12 arc tolerance. */
#define ARC_TOLERANCE_MM 0.002f
#define ARC_TWO_PI       6.28318530717958647692f

/* Upper bound on points generated for one mark_stroke()/submit_run() call
 * (wobble revolutions or feed-pacing resolution could otherwise grow
 * unbounded for a very long or very slow segment) - keeps RAM and
 * per-stroke build time bounded. */
#define STROKE_MAX_POINTS 2000

/* Bounds one M68 preview-loop burst's point count (see
 * preview_loop_outline() below) - ~160KB for the mm-space x/y arrays at
 * peak, comfortably inside the ~300KB heap this board has free. */
#define PREVIEW_MAX_POINTS 20000

/* Machine-space position tracking, in mm (not volts - conversion happens
 * at the point of submitting a move, via laser_ctrl_mm_to_volts()). */
static float s_pos_x_mm = 0.0f;
static float s_pos_y_mm = 0.0f;
static bool s_relative = false;      /* G91 vs G90 */
static bool s_inches = false;        /* G20 vs G21 */
static float s_feed_mm_per_min = 0.0f; /* sticky F; 0 = unpaced/max speed */

/* Laser-mode semantics (matches real GRBL's $32=1 laser mode, which this
 * controller always behaves as, since it has no spindle to speak of):
 * M3/M4 record *intent* to fire; the laser is only actually commanded on
 * while that intent is set AND the last motion word was a marking move
 * (G1/G2/G3) - G0 rapid moves always force it off regardless of intent,
 * so raster/vector jobs don't burn during traversal. This is what lets a
 * raster stream of bare "G1 X.. S.." lines (no M3 per line) work. */
static bool s_laser_intent = false;
static int s_last_motion_g = 1;      /* last G0/G1/G2/G3 seen; G1 by default */
static bool s_engrave_actual = false;

/* M66/M67: while true, every motion command traces its path with the
 * marking laser forced off (and the guide laser forced on), regardless of
 * M3/M4 intent - see update_engrave_from_laser_mode(). */
static bool s_preview_mode = false;

/* M64/M65: while recording, G0/G1 X../Y.. lines append to this polygon
 * instead of moving the DAC. Growable (realloc-doubling), reset on both
 * M64 (start) and after M65 (end) consumes it. */
static float *s_fill_x = NULL;
static float *s_fill_y = NULL;
static size_t s_fill_n = 0;
static size_t s_fill_cap = 0;
static bool s_fill_recording = false;

/* Written directly to stdout (bound to USB-Serial-JTAG via the sdkconfig
 * primary console setting - see sdkconfig.defaults), the same physical
 * link this task reads commands from via stdin. */
static void grbl_write_str(const char *s)
{
    fputs(s, stdout);
    fflush(stdout);
}

static void send_status_report(void)
{
    char buf[96];
    snprintf(buf, sizeof(buf), "<%s|MPos:%.3f,%.3f,0.000|FS:0,0>\r\n",
             s_engrave_actual ? "Run" : "Idle", (double)s_pos_x_mm, (double)s_pos_y_mm);
    grbl_write_str(buf);
}

static void send_settings_dump(void)
{
    /* Minimal, largely fake settings block for numbers this controller
     * doesn't actually use, just enough that senders which parse "$$"
     * output during their handshake get well-formed lines back - followed
     * by the real, live calibration/marking-quality settings ($100/$101/
     * $110/$111/$130-$134/$140/$150/$151/$160/$170/$171). */
    grbl_write_str(
        "$0=10\r\n$1=25\r\n$2=0\r\n$3=0\r\n$4=0\r\n$5=0\r\n$10=1\r\n"
        "$20=0\r\n$21=0\r\n$30=1000\r\n$31=0\r\n$32=1\r\n");
    calib_dump(grbl_write_str);
    grbl_write_str("ok\r\n");
}

/* Sends the engrave GPIO command only when the actual (post laser-mode
 * logic) state changes, to avoid spamming the queue on every line of a
 * raster stream that doesn't touch M-codes at all. Used both by the
 * M3/M4/M5-driven state machine below and directly by mark_stroke() to
 * bracket individual marking runs (e.g. for skywriting's laser-off
 * lead-in/lead-out). */
static void set_engrave_actual(bool on)
{
    struct laser_cmd cmd;

    if (on == s_engrave_actual)
        return;

    s_engrave_actual = on;
    cmd.type = LASER_CMD_ENGRAVE;
    cmd.gate_on_power = false;
    cmd.engrave_on = on;
    if (!laser_ctrl_submit(&cmd))
        ESP_LOGW(TAG, "engrave command dropped, queue full");
}

/* Recomputes the actual engrave output from (intent AND last-motion-was-a
 * marking-move AND not in preview mode) - see the s_laser_intent and
 * s_preview_mode comments above. Called after any change to intent
 * (M3/M4/M5), to the last motion word (G0/G1/G2/G3), or to preview mode
 * (M66/M67). Not used by mark_stroke() itself, which manages on/off
 * around its own runs directly via set_engrave_actual(). */
static void update_engrave_from_laser_mode(void)
{
    set_engrave_actual(!s_preview_mode && s_laser_intent && s_last_motion_g != 0);
}

static void apply_engrave_intent(bool on)
{
    s_laser_intent = on;
    update_engrave_from_laser_mode();
}

/* Submits a single point (already in mm) as the cheapest possible
 * command: a single LASER_CMD_MOVE if there's exactly one, otherwise a
 * LASER_CMD_STREAM. Takes ownership of xv/yv (frees them on the
 * single-point path; the queue consumer frees them on the stream path -
 * see dac_task.c). */
/* repeats applies to paced streams only (see laser_ctrl.h): 1 for a normal
 * one-shot run, 0 to loop the buffer until another command is queued. */
static void submit_points(float *xv, float *yv, size_t n, bool gate_on_power,
                           bool paced, uint32_t point_delay_us, uint32_t repeats)
{
    struct laser_cmd cmd;

    if (n == 1) {
        cmd.type = LASER_CMD_MOVE;
        cmd.gate_on_power = gate_on_power;
        cmd.move.x_volts = xv[0];
        cmd.move.y_volts = yv[0];
        free(xv);
        free(yv);
        if (!laser_ctrl_submit(&cmd))
            ESP_LOGW(TAG, "move command dropped, queue full");
        return;
    }

    cmd.type = LASER_CMD_STREAM;
    cmd.gate_on_power = gate_on_power;
    cmd.stream.x_volts = xv;
    cmd.stream.y_volts = yv;
    cmd.stream.n_points = n;
    cmd.stream.paced = paced;
    cmd.stream.point_delay_us = point_delay_us;
    cmd.stream.repeats = repeats;
    if (!laser_ctrl_submit(&cmd)) {
        ESP_LOGW(TAG, "stream command dropped, queue full");
        free(xv);
        free(yv);
    }
}

/*
 * Builds and submits a straight run from (x0,y0) to (x1,y1) in mm,
 * applying (if enabled) wobble - a circular offset perpendicular to
 * travel direction, for kerf-widening - and feed-rate pacing (constant-
 * velocity point spacing so dac_task's per-point delay approximates the
 * requested speed; no accel/decel ramp or junction-deviation planning).
 * Does NOT touch engrave state or position tracking - the caller
 * (mark_stroke() or the plain-jump path in process_gcode_line()) owns
 * both. gate_on_power is forwarded to the underlying command for
 * S-word/power-settle ordering (see laser_ctrl.h).
 */
static void submit_run(float x0, float y0, float x1, float y1, bool gate_on_power,
                        bool wobble_enabled, float feed_mm_per_min)
{
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx * dx + dy * dy);
    float ux = (len > 0.0001f) ? dx / len : 0.0f;
    float uy = (len > 0.0001f) ? dy / len : 0.0f;
    float wobble_d = wobble_enabled ? calib_wobble_diameter_mm() : 0.0f;
    float wobble_pitch = calib_wobble_pitch_mm();
    bool paced = feed_mm_per_min > 0.0f;
    size_t n = 2, i;
    uint32_t point_delay_us = 0;
    float *xs, *ys, *xv, *yv;

    if (wobble_d > 0.001f && wobble_pitch > 0.001f && len > 0.001f) {
        size_t n_wobble = (size_t)((len / wobble_pitch) * 8.0f) + 2;
        if (n_wobble > n)
            n = n_wobble;
    }
    if (paced && len > 0.02f) {
        size_t n_feed = (size_t)(len / 0.02f) + 1; /* ~1 point/20um of travel */
        if (n_feed > n)
            n = n_feed;
    }
    if (n > STROKE_MAX_POINTS)
        n = STROKE_MAX_POINTS;

    xs = malloc(n * sizeof(float));
    ys = malloc(n * sizeof(float));
    if (!xs || !ys) {
        ESP_LOGE(TAG, "submit_run: out of memory for %u points", (unsigned)n);
        free(xs);
        free(ys);
        return;
    }

    for (i = 0; i < n; i++) {
        float frac = (n > 1) ? (float)i / (float)(n - 1) : 1.0f;
        float px = x0 + dx * frac;
        float py = y0 + dy * frac;

        if (wobble_d > 0.001f && len > 0.001f) {
            float phase = (len * frac / wobble_pitch) * ARC_TWO_PI;
            float wr = wobble_d * 0.5f;

            px += -uy * wr * sinf(phase);
            py += ux * wr * sinf(phase);
        }
        xs[i] = px;
        ys[i] = py;
    }

    xv = malloc(n * sizeof(float));
    yv = malloc(n * sizeof(float));
    if (!xv || !yv) {
        ESP_LOGE(TAG, "submit_run: out of memory for volts buffers");
        free(xs);
        free(ys);
        free(xv);
        free(yv);
        return;
    }
    for (i = 0; i < n; i++)
        laser_ctrl_mm_to_volts(xs[i], ys[i], &xv[i], &yv[i]);
    free(xs);
    free(ys);

    if (paced) {
        float total_ms = (len / feed_mm_per_min) * 60000.0f;
        point_delay_us = (uint32_t)((total_ms * 1000.0f) / (float)n);
    }
    submit_points(xv, yv, n, gate_on_power, paced, point_delay_us, 1);
}

/*
 * Full mark stroke from (x0,y0) to (x1,y1): brackets the run with engrave
 * on/off (via set_engrave_actual(), so consecutive already-on strokes
 * don't redundantly re-toggle), and - if calib_skywrite_margin_mm() > 0 -
 * adds separate laser-OFF lead-in/lead-out runs extending the path by
 * that margin, so the galvo is already moving at speed when the laser
 * turns on and keeps moving past the endpoint (instead of decelerating
 * into it) when the laser turns off. Real EZCAD-class boards do this per
 * stroke; so does this, which means consecutive strokes of what a sender
 * considers one continuous polyline each get their own lead-in/out (some
 * redundant on/off toggling at shared vertices, each paying
 * laser_on_delay/laser_off_delay) rather than one continuously-skywritten
 * path - a deliberate scope simplification, see STATUS.md. With the
 * default $160=0 (skywriting disabled), this reduces to the single
 * set_engrave_actual(true) + submit_run() call, identical to this
 * firmware's prior (pre-this-feature) behavior.
 *
 * gate_on_power should be true only once per G-code line (when its S-word
 * changed power) - callers doing repeat counts (R word) should pass true
 * only for the first repetition.
 */
static void mark_stroke(float x0, float y0, float x1, float y1, bool gate_on_power)
{
    float margin = calib_skywrite_margin_mm();
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx * dx + dy * dy);

    if (margin > 0.001f && len > 0.001f) {
        float ux = dx / len, uy = dy / len;
        float in_x = x0 - ux * margin, in_y = y0 - uy * margin;
        float out_x = x1 + ux * margin, out_y = y1 + uy * margin;

        set_engrave_actual(false);
        submit_run(in_x, in_y, x0, y0, false, false, 0.0f);
        set_engrave_actual(true);
        submit_run(x0, y0, x1, y1, gate_on_power, true, s_feed_mm_per_min);
        set_engrave_actual(false);
        submit_run(x1, y1, out_x, out_y, false, false, 0.0f);
    } else {
        set_engrave_actual(true);
        submit_run(x0, y0, x1, y1, gate_on_power, true, s_feed_mm_per_min);
    }
}

/* Plain (non-marking) travel move: a rapid G0, or a G1/G2/G3 while laser
 * intent/preview mode says not to fire. No wobble/skywrite - those are
 * marking-quality concepts a jump has no use for - but IS genuinely
 * interpolated and paced at $182's max velocity (reusing submit_run(),
 * the same interpolation/pacing machinery marking runs use), not an
 * instantaneous single-point jump like this used to be. Found necessary
 * for the same physical reason M68's preview loop needed edge
 * interpolation: an instantaneous voltage step exceeds the galvo's
 * mechanical slew rate regardless of whether the laser is firing during
 * the move - "how is travel/jump speed defined?" has to have an answer,
 * and previously the answer was "however fast the DAC can step," which
 * is not a real answer. jump_delay (dac_task, keyed off engrave state)
 * still applies as additional settle time after the move completes. */
static void submit_plain_move(float x0_mm, float y0_mm, float x1_mm, float y1_mm,
                               bool gate_on_power)
{
    submit_run(x0_mm, y0_mm, x1_mm, y1_mm, gate_on_power, false,
               calib_max_velocity_mm_s() * 60.0f);
}

/*
 * motion_planner.h's execute callback: given one finalized segment
 * (entry/cruise/exit speed + accel already decided by the planner's
 * junction-deviation/trapezoid logic), samples the path at fixed time
 * steps - which, since position is computed from the accel/cruise/decel
 * kinematics rather than linear interpolation, naturally encodes the
 * non-constant velocity without needing per-point variable delays; a
 * single point_delay_us (the fixed time step) is all dac_task's existing
 * paced-stream path needs (see dac_task.c). Wobble is applied the same
 * way submit_run() does, keyed off true distance-traveled rather than
 * time, so wobble pitch stays correct regardless of the speed profile.
 *
 * Skywriting (if calib_skywrite_margin_mm() > 0) is applied only at true
 * sequence boundaries - lead-in on the first block since the last hard
 * flush (is_first_in_sequence), lead-out on the last block of a hard
 * flush (is_last_in_sequence) - not on every block, since consecutive
 * planned blocks already maintain continuous velocity through their
 * shared junction and don't need a laser-off gap between them.
 */
static void motion_block_execute_cb(const struct motion_block_result *blk)
{
    float dx = blk->x1_mm - blk->x0_mm;
    float dy = blk->y1_mm - blk->y0_mm;
    float len = sqrtf(dx * dx + dy * dy);
    float ux = (len > 0.0001f) ? dx / len : 0.0f;
    float uy = (len > 0.0001f) ? dy / len : 0.0f;
    float margin = calib_skywrite_margin_mm();
    float wobble_d = calib_wobble_diameter_mm();
    float wobble_pitch = calib_wobble_pitch_mm();
    float v0 = blk->entry_speed_mm_s;
    float vc = blk->cruise_speed_mm_s;
    float v1 = blk->exit_speed_mm_s;
    float a = blk->accel_mm_s2;

    if (blk->is_first_in_sequence && margin > 0.001f && len > 0.001f) {
        float in_x = blk->x0_mm - ux * margin, in_y = blk->y0_mm - uy * margin;

        set_engrave_actual(false);
        submit_run(in_x, in_y, blk->x0_mm, blk->y0_mm, false, false, 0.0f);
    }

    set_engrave_actual(true);

    if (len < 0.0001f || a <= 0.0f) {
        /* Degenerate (shouldn't normally happen - motion_planner_enqueue()
         * already rejects zero-length segments): just land on the endpoint. */
        float *pxv = malloc(sizeof(float));
        float *pyv = malloc(sizeof(float));

        if (pxv && pyv) {
            laser_ctrl_mm_to_volts(blk->x1_mm, blk->y1_mm, pxv, pyv);
            submit_points(pxv, pyv, 1, blk->gate_on_power, false, 0, 1);
        } else {
            free(pxv);
            free(pyv);
        }
    } else {
        float t1 = (vc > v0) ? (vc - v0) / a : 0.0f;
        float d1 = v0 * t1 + 0.5f * a * t1 * t1;
        float t3 = (vc > v1) ? (vc - v1) / a : 0.0f;
        float d3 = vc * t3 - 0.5f * a * t3 * t3;
        float d2, t2, total_time;
        size_t n, i;
        float *xs, *ys, *xv, *yv;

        if (d1 + d3 > len) {
            /* Segment too short for a full trapezoid - degrade to a
             * single ramp scaled to fit (approximation - not an exact
             * S-curve, see motion_planner.h). */
            float scale = len / (d1 + d3);

            t1 *= scale;
            t3 *= scale;
            d1 *= scale;
            d3 *= scale;
        }
        d2 = len - d1 - d3;
        if (d2 < 0.0f)
            d2 = 0.0f;
        t2 = (vc > 0.001f) ? d2 / vc : 0.0f;
        total_time = t1 + t2 + t3;
        if (total_time < 0.0005f)
            total_time = 0.0005f;

        /* ~1 point per 0.5ms of travel time, or enough for wobble
         * fidelity if that asks for more - whichever is finer, capped. */
        n = (size_t)(total_time / 0.0005f) + 2;
        if (wobble_d > 0.001f && wobble_pitch > 0.001f) {
            size_t n_wobble = (size_t)((len / wobble_pitch) * 8.0f) + 2;

            if (n_wobble > n)
                n = n_wobble;
        }
        if (n > STROKE_MAX_POINTS)
            n = STROKE_MAX_POINTS;
        if (n < 2)
            n = 2;

        xs = malloc(n * sizeof(float));
        ys = malloc(n * sizeof(float));
        if (!xs || !ys) {
            ESP_LOGE(TAG, "motion_block_execute_cb: out of memory for %u points", (unsigned)n);
            free(xs);
            free(ys);
        } else {
            for (i = 0; i < n; i++) {
                float t = total_time * (float)i / (float)(n - 1);
                float dist, frac, px, py;

                if (t < t1)
                    dist = v0 * t + 0.5f * a * t * t;
                else if (t < t1 + t2)
                    dist = d1 + vc * (t - t1);
                else {
                    float tau = t - t1 - t2;
                    dist = d1 + d2 + vc * tau - 0.5f * a * tau * tau;
                }
                if (dist < 0.0f) dist = 0.0f;
                if (dist > len) dist = len;
                frac = dist / len;

                px = blk->x0_mm + dx * frac;
                py = blk->y0_mm + dy * frac;
                if (wobble_d > 0.001f && wobble_pitch > 0.001f) {
                    float phase = (dist / wobble_pitch) * ARC_TWO_PI;
                    float wr = wobble_d * 0.5f;

                    px += -uy * wr * sinf(phase);
                    py += ux * wr * sinf(phase);
                }
                xs[i] = px;
                ys[i] = py;
            }

            xv = malloc(n * sizeof(float));
            yv = malloc(n * sizeof(float));
            if (!xv || !yv) {
                ESP_LOGE(TAG, "motion_block_execute_cb: out of memory for volts buffers");
                free(xs);
                free(ys);
                free(xv);
                free(yv);
            } else {
                uint32_t point_delay_us;

                for (i = 0; i < n; i++)
                    laser_ctrl_mm_to_volts(xs[i], ys[i], &xv[i], &yv[i]);
                free(xs);
                free(ys);

                point_delay_us = (uint32_t)((total_time * 1.0e6f) / (float)n);
                submit_points(xv, yv, n, blk->gate_on_power, true, point_delay_us, 1);
            }
        }
    }

    if (blk->is_last_in_sequence) {
        set_engrave_actual(false);
        if (margin > 0.001f && len > 0.001f) {
            float out_x = blk->x1_mm + ux * margin, out_y = blk->y1_mm + uy * margin;

            submit_run(blk->x1_mm, blk->y1_mm, out_x, out_y, false, false, 0.0f);
        }
    }
}

/*
 * G2 (clockwise) / G3 (counter-clockwise) circular interpolation, I/J
 * center-offset form only (R-radius form is not supported - R is
 * repurposed as the repeat-count word instead, see the word/M-code table
 * above). Per GRBL semantics, I/J are always incremental offsets from the
 * arc's start point to its center, regardless of G90/G91 mode; X/Y still
 * respect G90/G91 like any other move.
 *
 * This controller has no native circular DAC output mode, so the arc is
 * interpolated into short straight segments (sized by ARC_TOLERANCE_MM,
 * mirroring the role of real GRBL's $12 arc tolerance) and streamed as
 * one fast (unpaced) burst - no wobble/skywrite/feed-pacing applied to
 * arcs in this pass, a deliberate scope simplification (see STATUS.md);
 * those apply to straight G1 marking runs via mark_stroke() above.
 */
static void apply_arc(bool cw, bool have_x, float x, bool have_y, float y,
                       bool have_i, float i_off, bool have_j, float j_off)
{
    float start_x = s_pos_x_mm, start_y = s_pos_y_mm;
    float target_x = start_x, target_y = start_y;
    float center_x, center_y, radius;
    float start_angle, end_angle, angular_travel;
    int segments, k;

    if (s_relative) {
        if (have_x)
            target_x = start_x + x;
        if (have_y)
            target_y = start_y + y;
    } else {
        if (have_x)
            target_x = x;
        if (have_y)
            target_y = y;
    }

    center_x = start_x + (have_i ? i_off : 0.0f);
    center_y = start_y + (have_j ? j_off : 0.0f);

    radius = sqrtf((start_x - center_x) * (start_x - center_x) +
                    (start_y - center_y) * (start_y - center_y));
    if (radius < 0.001f) {
        /* No usable I/J (degenerate arc): fall back to a direct move
         * rather than dividing by ~zero below. */
        ESP_LOGW(TAG, "arc with no usable I/J, treating as a direct move");
        submit_plain_move(start_x, start_y, target_x, target_y, false);
        s_pos_x_mm = target_x;
        s_pos_y_mm = target_y;
        return;
    }

    start_angle = atan2f(start_y - center_y, start_x - center_x);
    end_angle = atan2f(target_y - center_y, target_x - center_x);

    if (!have_x && !have_y) {
        /* No target given: a full circle back to the start point. */
        angular_travel = ARC_TWO_PI;
    } else if (cw) {
        angular_travel = start_angle - end_angle;
        if (angular_travel <= 0.0f)
            angular_travel += ARC_TWO_PI;
    } else {
        angular_travel = end_angle - start_angle;
        if (angular_travel <= 0.0f)
            angular_travel += ARC_TWO_PI;
    }

    segments = (int)floorf(angular_travel /
                            (2.0f * acosf(1.0f - ARC_TOLERANCE_MM / radius)));
    if (segments < 1)
        segments = 1;

    /* Batch the whole arc into one LASER_CMD_STREAM instead of one
     * LASER_CMD_MOVE per segment: dac_task clocks it out as a single SPI
     * burst (ad3552r_board_stream_xy()), which amortizes per-transaction
     * driver overhead across the whole arc instead of paying it per
     * point - the same overhead that dominated the per-write speed test
     * (see STATUS.md). RAM is plentiful for this (segments is bounded by
     * ARC_TOLERANCE_MM/radius, never more than a few hundred for any
     * arc size sane on a galvo). */
    {
        float *xv = malloc((size_t)segments * sizeof(float));
        float *yv = malloc((size_t)segments * sizeof(float));

        if (!xv || !yv) {
            ESP_LOGE(TAG, "arc: out of memory for %d segments, sending direct move instead",
                     segments);
            free(xv);
            free(yv);
            submit_plain_move(start_x, start_y, target_x, target_y, false);
            s_pos_x_mm = target_x;
            s_pos_y_mm = target_y;
            return;
        }

        for (k = 1; k <= segments; k++) {
            float px, py;

            if (k == segments) {
                /* Land exactly on the commanded target (or back at the
                 * start point, for a full circle) instead of
                 * accumulating float error from the last interpolated
                 * angle. */
                px = target_x;
                py = target_y;
            } else {
                float frac = (float)k / (float)segments;
                float angle = cw ? start_angle - angular_travel * frac
                                  : start_angle + angular_travel * frac;
                px = center_x + radius * cosf(angle);
                py = center_y + radius * sinf(angle);
            }
            laser_ctrl_mm_to_volts(px, py, &xv[k - 1], &yv[k - 1]);
        }

        s_pos_x_mm = target_x;
        s_pos_y_mm = target_y;

        submit_points(xv, yv, (size_t)segments, false, false, 0, 1);
    }
}

static void apply_prr_hz(float hz)
{
    struct laser_cmd cmd;

    if (hz < 0.0f)
        hz = 0.0f;
    if (hz > PRR_MAX_HZ)
        hz = PRR_MAX_HZ;

    cmd.type = LASER_CMD_PRR;
    cmd.gate_on_power = false;
    cmd.prr_hz = hz;
    if (!laser_ctrl_submit(&cmd))
        ESP_LOGW(TAG, "prr command dropped, queue full");
}

/* M64/M65 fill-region point recording, growable (doubling) buffer. */
static void fill_reset(void)
{
    free(s_fill_x);
    free(s_fill_y);
    s_fill_x = NULL;
    s_fill_y = NULL;
    s_fill_n = 0;
    s_fill_cap = 0;
}

static void fill_add_point(float x, float y)
{
    if (s_fill_n == s_fill_cap) {
        size_t new_cap = s_fill_cap ? s_fill_cap * 2 : 64;
        float *nx = realloc(s_fill_x, new_cap * sizeof(float));
        float *ny = realloc(s_fill_y, new_cap * sizeof(float));

        if (!nx || !ny) {
            ESP_LOGE(TAG, "fill: out of memory recording point, dropping it");
            return;
        }
        s_fill_x = nx;
        s_fill_y = ny;
        s_fill_cap = new_cap;
    }
    s_fill_x[s_fill_n] = x;
    s_fill_y[s_fill_n] = y;
    s_fill_n++;
}

/* laser_fill_generate()'s emit_stroke callback: each hatch span becomes
 * one ordinary mark_stroke() run (so hatch lines get the same
 * wobble/skywrite/pacing treatment as any other marking run). */
static void fill_stroke_cb(float x0, float y0, float x1, float y1)
{
    mark_stroke(x0, y0, x1, y1, false);
}

/* M65: traces the recorded polygon's outline, then fills its interior
 * with hatch lines (see laser_fill.c - even-odd scanline fill, single
 * non-self-intersecting contour only). Leaves position at the first
 * recorded vertex (the outline's closing point). */
static void generate_hatch_and_outline(void)
{
    size_t i;

    for (i = 0; i < s_fill_n; i++) {
        size_t j = (i + 1) % s_fill_n;
        mark_stroke(s_fill_x[i], s_fill_y[i], s_fill_x[j], s_fill_y[j], false);
    }

    laser_fill_generate(s_fill_x, s_fill_y, s_fill_n,
                         calib_hatch_angle_deg(), calib_hatch_spacing_mm(),
                         fill_stroke_cb);

    s_pos_x_mm = s_fill_x[0];
    s_pos_y_mm = s_fill_y[0];
}

/*
 * M68 P<hz>: on-device repeat-loop of the shape recorded via M64 (an
 * alternative to M65 - both consume/reset the same recording, M65 traces
 * it once and hatch-fills the interior, M68 instead loops just the
 * outline continuously at a target refresh rate). Exists because this
 * firmware's serial protocol is one G-code line per point, waiting for
 * "ok" each time (~5 moves/sec measured - see STATUS.md's "Serial
 * bandwidth" section) - nowhere near fast enough for a live preview
 * refresh rate. Building the outline once and pacing it entirely
 * on-device (dac_task's existing paced-stream mechanism, added earlier
 * this session for feed-rate pacing) sidesteps the serial bottleneck
 * completely for this one narrow use case. A general character-counting
 * streaming protocol would be the real, general-purpose fix (still not
 * implemented - see STATUS.md); this is a much smaller, immediately-
 * useful one.
 *
 * One call loops indefinitely: dac_task replays the buffer until another
 * command is queued (see laser_ctrl.h's stream.repeats), so the preview
 * is continuous and needs no host round-trip per revolution. It ends on
 * the next command of any kind - a new G0, M63/M67, or Ctrl-X.
 *
 * PREVIEW_MAX_POINTS now bounds a single revolution rather than a whole
 * burst, which is a far looser constraint: the heap cost is one
 * repetition (~4KB for a typical shape), not the ~160KB the repeated
 * path used to take.
 */
static void preview_loop_outline(float hz)
{
    /* Each recorded edge is subdivided into this many intermediate points
     * rather than jumped in one instantaneous step - found necessary by
     * direct visual inspection on real hardware: a single voltage step
     * between corners exceeds the galvo's mechanical slew rate, causing
     * overshoot/ringing ("hotspots") right at each corner. But too few
     * subdivisions just moves the problem: each intermediate point then
     * becomes its own visible dwell spot instead (also found by direct
     * inspection). Live-tunable via $190 (calib.h) rather than a fixed
     * constant, since the right value depends on the specific galvo/
     * mirror's slew rate and this board's mechanics weren't characterized
     * ahead of time - raise it until the motion looks continuous. */
    size_t subdivisions = (size_t)calib_preview_edge_subdivisions();
    size_t n, i;
    float *xs, *ys;
    uint32_t point_delay_us;

    if (subdivisions < 1)
        subdivisions = 1;
    n = s_fill_n * subdivisions;

    if (hz < 1.0f)
        hz = 1.0f;
    if (hz > 2000.0f)
        hz = 2000.0f;

    if (n > PREVIEW_MAX_POINTS) {
        ESP_LOGE(TAG, "M68: shape needs %u points (%u edges x %u subdivisions), "
                 "over the %u limit - lower $190", (unsigned)n, (unsigned)s_fill_n,
                 (unsigned)subdivisions, (unsigned)PREVIEW_MAX_POINTS);
        return;
    }

    xs = malloc(n * sizeof(float));
    ys = malloc(n * sizeof(float));
    if (!xs || !ys) {
        ESP_LOGE(TAG, "M68: out of memory for %u points", (unsigned)n);
        free(xs);
        free(ys);
        return;
    }

    /* Just one repetition of the outline (each edge linearly subdivided).
     * dac_task replays this buffer itself - see laser_ctrl.h's
     * stream.repeats. This used to memcpy the same points ~41 times into
     * one ~160KB buffer, which both wasted heap (two coexisting bursts
     * overflowed it) and forced the host to keep re-issuing M68, parking
     * the galvo in the gap each time. */
    {
        size_t p = 0, j;

        for (j = 0; j < s_fill_n; j++) {
            size_t j2 = (j + 1) % s_fill_n;
            size_t sidx;

            for (sidx = 0; sidx < subdivisions; sidx++) {
                float frac = (float)sidx / (float)subdivisions;

                xs[p] = s_fill_x[j] + (s_fill_x[j2] - s_fill_x[j]) * frac;
                ys[p] = s_fill_y[j] + (s_fill_y[j2] - s_fill_y[j]) * frac;
                p++;
            }
        }
    }

    /* Convert mm -> volts in place (xs[i]/ys[i]'s mm values are passed by
     * value into laser_ctrl_mm_to_volts() before it writes the volts
     * result back through the same pointers, so this is safe) instead of
     * allocating separate output buffers - halves peak memory, which
     * matters at PREVIEW_MAX_POINTS scale on a ~300KB heap. */
    for (i = 0; i < n; i++)
        laser_ctrl_mm_to_volts(xs[i], ys[i], &xs[i], &ys[i]);

    point_delay_us = (uint32_t)((1.0e6f / hz) / (float)n);

    /* repeats = 0: dac_task replays this outline until another command is
     * queued, so the preview runs continuously with no host round-trip and
     * no gap between revolutions. It ends on the next command of any kind -
     * a new G0, M63/M67, or Ctrl-X. */
    submit_points(xs, ys, n, false, true, point_delay_us, 0);

    /* Only what this side knows: the shape and what was asked for. Whether
     * the request is actually achievable depends on which pacing path
     * dac_task takes - the SPI clock paces it exactly when the rate is
     * within the divider's range, otherwise it software-paces and bottoms
     * out around dac_task_us_per_point(). dac_task logs the rate it really
     * achieved once the run starts; guessing at it here would just risk
     * printing a second, disagreeing number. */
    ESP_LOGI(TAG, "M68: looping a %u-point (%u edges x %u subdivisions) shape "
             "continuously at a requested %.1fHz (%luus/point)",
             (unsigned)n, (unsigned)s_fill_n, (unsigned)subdivisions,
             (double)hz, (unsigned long)point_delay_us);
}

/* Parses one G-code line (already NUL-terminated, no trailing CR/LF) and
 * dispatches the resulting commands. Recognizes G0/G1/G2/G3/G4/G20/G21/
 * G90/G91/G92, M2/M3/M4/M5/M10/M11/M30/M62/M63/M64/M65/M66/M67/M68, and
 * X/Y/I/J/P/Q/S/F/R words (see the word/M-code table near the top of this
 * file for the non-stock ones). Unrecognized codes are silently accepted
 * (GRBL senders generally expect "ok" for anything they consider a no-op
 * on this controller) rather than erroring, to maximize sender
 * compatibility. */
static void process_gcode_line(const char *line, bool jog)
{
    /* Units (G20/G21) and distance mode (G90/G91) are modal setters that
     * routinely share a line with a motion word ("G90 G21 F1000 X3 Y-2",
     * which is exactly what GRBL senders put in a $J= jog payload), so
     * they get their own slots - a single have_g would keep only the
     * last G word on the line and silently drop the others. */
    int have_g = -1, have_m = -1, have_units_g = -1, have_dist_g = -1;
    bool have_x = false, have_y = false, have_i = false, have_j = false;
    bool have_p = false, have_s = false, have_q = false, have_f = false, have_r = false;
    float x = 0, y = 0, i_off = 0, j_off = 0, p_val = 0, s_val = 0, q_val = 0;
    float f_val = 0, r_val = 0;
    const char *p = line;

    while (*p) {
        while (*p == ' ' || *p == '\t')
            p++;
        if (!*p)
            break;

        char letter = (char)toupper((unsigned char)*p);
        if (letter == ';' || letter == '(') {
            break; /* rest of line is a comment */
        }
        p++;

        char *endp;
        float val = strtof(p, &endp);
        if (endp == p) {
            /* No numeric value followed the letter - skip it and keep going
             * rather than aborting the whole line. */
            continue;
        }
        p = endp;

        switch (letter) {
        case 'G': {
            int g = (int)val;
            if (g == 20 || g == 21)
                have_units_g = g;
            else if (g == 90 || g == 91)
                have_dist_g = g;
            else
                have_g = g;
            break;
        }
        case 'M':
            have_m = (int)val;
            break;
        case 'X':
            have_x = true;
            x = val;
            break;
        case 'Y':
            have_y = true;
            y = val;
            break;
        case 'I':
            have_i = true;
            i_off = val;
            break;
        case 'J':
            have_j = true;
            j_off = val;
            break;
        case 'P':
            have_p = true;
            p_val = val;
            break;
        case 'S':
            have_s = true;
            s_val = val;
            break;
        case 'Q':
            have_q = true;
            q_val = val;
            break;
        case 'F':
            have_f = true;
            f_val = val;
            break;
        case 'R':
            have_r = true;
            r_val = val;
            break;
        default:
            break;
        }
    }

    /* G20/G21 (inch/mm units) apply to every distance word on this same
     * line, including the arc center offsets and feedrate. */
    if (have_units_g == 20)
        s_inches = true;
    else if (have_units_g == 21)
        s_inches = false;
    if (s_inches) {
        const float MM_PER_INCH = 25.4f;
        if (have_x) x *= MM_PER_INCH;
        if (have_y) y *= MM_PER_INCH;
        if (have_i) i_off *= MM_PER_INCH;
        if (have_j) j_off *= MM_PER_INCH;
        if (have_f) f_val *= MM_PER_INCH;
    }
    if (have_f)
        s_feed_mm_per_min = f_val;

    if (have_s) {
        /* Standard GRBL laser-mode convention: S is power, not PRR - sent
         * to the ATMEGA, which owns the actual power setpoint.
         * atmega_link_set_power() only enqueues the request now (this
         * task never blocks on the ATMEGA - see atmega_link.h), so "ok"
         * for this line returns immediately regardless of ATMEGA speed.
         * Position/power ordering is still enforced, just relocated:
         * whichever move/stream/stroke command is submitted below is
         * marked gate_on_power=true, so dac_task (not this task) waits
         * for the power to settle immediately before writing it to the
         * DAC. See laser_ctrl.h's gate_on_power comment and dac_task.c. */
        atmega_link_set_power(s_val / GRBL_S_MAX * 100.0f);
    }

    if (have_dist_g == 90)
        s_relative = false;
    else if (have_dist_g == 91)
        s_relative = true;

    /* A $J= jog payload carries no motion word of its own - the X/Y on it
     * are always a travel move, and a jog never fires the laser. Forcing
     * G0 here reuses the ordinary travel path below (which also means
     * jogs respect the same limits and pacing as any other move). */
    if (jog)
        have_g = 0;

    if (have_g == 92) {
        if (have_x)
            s_pos_x_mm = x;
        if (have_y)
            s_pos_y_mm = y;
    } else if (have_g == 4) {
        /* Dwell: blocks this task (and so the sender, which waits for our
         * "ok" before sending the next line) for P seconds. Does not
         * block core 0 - queued DAC/engrave/PRR commands already
         * submitted keep running normally. Flush first so any pending
         * planned marks are at least handed off to the queue before the
         * (semantically unrelated) wait begins. */
        motion_planner_flush();
        if (have_p && p_val > 0.0f)
            vTaskDelay(pdMS_TO_TICKS((uint32_t)(p_val * 1000.0f)));
    } else if (have_g == 0 || have_g == 1) {
        s_last_motion_g = have_g;
        if (have_x || have_y) {
            float tx = s_pos_x_mm, ty = s_pos_y_mm;
            bool will_mark = (have_g == 1) && s_laser_intent && !s_preview_mode;

            if (s_relative) {
                if (have_x) tx += x;
                if (have_y) ty += y;
            } else {
                if (have_x) tx = x;
                if (have_y) ty = y;
            }

            if (s_fill_recording) {
                /* Recording a fill region: remember the vertex, don't
                 * actually move anything until M65. */
                fill_add_point(tx, ty);
                s_pos_x_mm = tx;
                s_pos_y_mm = ty;
            } else if (will_mark) {
                int reps = (have_r && r_val >= 1.0f) ? (int)r_val : 1;

                if (reps > 1) {
                    /* Repeat count bypasses the look-ahead planner (see
                     * the word-table comment above) - flush first so
                     * ordering against any already-buffered planned
                     * marks is preserved. */
                    int rep;
                    float start_x = s_pos_x_mm, start_y = s_pos_y_mm;

                    motion_planner_flush();
                    for (rep = 0; rep < reps; rep++)
                        mark_stroke(start_x, start_y, tx, ty, have_s && rep == 0);
                } else {
                    motion_planner_enqueue(s_pos_x_mm, s_pos_y_mm, tx, ty,
                                            s_feed_mm_per_min, have_s);
                }
                s_pos_x_mm = tx;
                s_pos_y_mm = ty;
            } else {
                /* G0 rapid, or G1 with laser off/preview mode: a genuinely
                 * interpolated/paced travel move now (see
                 * submit_plain_move()'s comment), no wobble/skywrite (not
                 * applicable to a non-marking jump). Flush any pending
                 * planned marks first so they decelerate to a stop and
                 * execute before this jump, in the right order. */
                motion_planner_flush();
                submit_plain_move(s_pos_x_mm, s_pos_y_mm, tx, ty, have_s);
                s_pos_x_mm = tx;
                s_pos_y_mm = ty;
                set_engrave_actual(false);
            }
        } else {
            /* No X/Y on this line: just (re-)apply the laser-mode state
             * without moving (e.g. a bare "G1 S500" changing power while
             * stationary, or a G0/G1 mode-only switch). */
            update_engrave_from_laser_mode();
        }
    } else if (have_g == 2 || have_g == 3) {
        s_last_motion_g = have_g;
        if (s_fill_recording)
            ESP_LOGW(TAG, "arcs are not supported inside a fill region (M64/M65), ignoring");
        else {
            /* Arcs don't go through the look-ahead planner (see
             * apply_arc()'s comment) - flush any pending planned marks
             * first so ordering is preserved. */
            motion_planner_flush();
            apply_arc(have_g == 2, have_x, x, have_y, y, have_i, i_off, have_j, j_off);
        }
        update_engrave_from_laser_mode();
    }

    if (have_m == 3 || have_m == 4)
        apply_engrave_intent(true);
    else if (have_m == 5)
        apply_engrave_intent(false);
    else if (have_m == 2 || have_m == 30) {
        /* Program end: stop firing and return to absolute mode, mirroring
         * real GRBL's end-of-program behavior. Position is left as-is.
         * Flush first so any pending planned marks actually execute
         * rather than being silently discarded. */
        motion_planner_flush();
        apply_engrave_intent(false);
        s_relative = false;
    } else if (have_m == 62 || have_m == 63) {
        /* Repurposing GRBL's digital-output M-codes for the guide laser -
         * only one output channel exists, so P is accepted but ignored. */
        atmega_link_set_guide(have_m == 62);
    } else if (have_m == 64) {
        /* Fill recording bypasses the planner entirely (see mark_stroke()
         * usage in generate_hatch_and_outline()) - flush first so
         * ordering against any already-buffered planned marks holds. */
        motion_planner_flush();
        if (s_fill_recording)
            ESP_LOGW(TAG, "M64 while already recording a fill region - restarting it");
        fill_reset();
        s_fill_recording = true;
    } else if (have_m == 65) {
        s_fill_recording = false;
        if (s_fill_n >= 3)
            generate_hatch_and_outline();
        else
            ESP_LOGW(TAG, "M65: fill region had fewer than 3 points, nothing to fill");
        fill_reset();
    } else if (have_m == 68) {
        /* Unlike M65, deliberately does NOT fill_reset() afterward - M68
         * exists specifically to be called repeatedly (looping a live
         * preview indefinitely), so the recorded shape stays available
         * for the next M68 call. Only a fresh M64 (or Ctrl-X) clears it. */
        s_fill_recording = false;
        if (s_fill_n >= 2)
            preview_loop_outline(have_p ? p_val : 100.0f);
        else
            ESP_LOGW(TAG, "M68: no recorded shape (record one with M64 first)");
    } else if (have_m == 66) {
        motion_planner_flush(); /* pending planned marks execute under the old mode, not the new one */
        s_preview_mode = true;
        atmega_link_set_guide(true);
        update_engrave_from_laser_mode(); /* forces actual engrave off */
    } else if (have_m == 67) {
        s_preview_mode = false;
        atmega_link_set_guide(false);
        update_engrave_from_laser_mode();
    } else if (have_m == 10 || have_m == 11) {
        /* Custom: arm / disarm. This line's "ok" returns immediately -
         * atmega_link_set_armed() only enqueues the request. The ATMEGA's
         * own arm sequence takes ~2s (see atmega_link.c), handled entirely
         * by that module's background task so this task (and '?' status
         * queries, and reading the next line) stays responsive throughout. */
        atmega_link_set_armed(have_m == 10);
    }

    if (have_q)
        apply_prr_hz(q_val);
}

static void handle_line(char *line)
{
    /* Strip trailing CR/LF/whitespace. */
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' ||
                        line[len - 1] == ' '))
        line[--len] = '\0';

    if (len == 0) {
        grbl_write_str("ok\r\n");
        return;
    }

    if (line[0] == '$') {
        if (strcmp(line, "$$") == 0) {
            send_settings_dump();
            return;
        }

        /* "$J=<motion>" (jog) has to be recognized before the generic
         * "$<n>=<value>" setting-write branch below, which would otherwise
         * parse it as setting number atoi("J") == 0 and silently discard
         * the move - senders use $J= for their jog buttons and "move to"
         * (see rayforge's dialect move_to/jog), so that looked like the
         * machine ignoring every manual move. */
        if (strncasecmp(line, "$J=", 3) == 0) {
            /* Per the GRBL jog spec the payload's G90/G91, G20/G21 and F
             * apply to this jog only and must not disturb the modal state
             * a running job depends on, so they're saved and restored
             * around it rather than left applied. */
            bool saved_relative = s_relative;
            bool saved_inches = s_inches;
            float saved_feed = s_feed_mm_per_min;
            int saved_motion_g = s_last_motion_g;

            process_gcode_line(line + 3, true);

            s_relative = saved_relative;
            s_inches = saved_inches;
            s_feed_mm_per_min = saved_feed;
            s_last_motion_g = saved_motion_g;
            grbl_write_str("ok\r\n");
            return;
        }

        char *eq = strchr(line, '=');
        if (eq) {
            int number = atoi(line + 1);
            float value = strtof(eq + 1, NULL);

            if (!calib_set_param(number, value))
                ESP_LOGW(TAG, "unrecognized setting $%d (accepted anyway)", number);
            /* Cheap and always-correct: just re-read all three motion
             * limits regardless of which $ number changed, rather than
             * special-casing 180/181/182 here. */
            motion_planner_set_params(calib_max_accel_mm_s2(), calib_junction_deviation_mm(),
                                       calib_max_velocity_mm_s());
            grbl_write_str("ok\r\n");
            return;
        }

        /* $X (unlock), $H (home), and anything else "$"-prefixed with no
         * '=': this controller has no alarm state and no real homing
         * sequence, so just acknowledge. */
        grbl_write_str("ok\r\n");
        return;
    }

    process_gcode_line(line, false);
    grbl_write_str("ok\r\n");
}

static void grbl_task_fn(void *arg)
{
    char line[GRBL_LINE_MAX];
    size_t line_len = 0;

    ESP_LOGI(TAG, "GRBL task running on core %d", xPortGetCoreID());

    motion_planner_init(motion_block_execute_cb);
    motion_planner_set_params(calib_max_accel_mm_s2(), calib_junction_deviation_mm(),
                               calib_max_velocity_mm_s());

    grbl_write_str("\r\nGrbl 1.1h ['$' for help]\r\n");

    for (;;) {
        /* stdin is bound to USB-Serial-JTAG (see sdkconfig.defaults);
         * fgetc blocks until a byte arrives, which is fine here since this
         * task has nothing else to do between commands. */
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        uint8_t byte = (uint8_t)c;

        /* Realtime single-char commands take effect immediately, even
         * mid-line, and are never themselves part of a buffered line. */
        if (byte == '?') {
            send_status_report();
            continue;
        }
        if (byte == '!' || byte == '~') {
            grbl_write_str("ok\r\n"); /* feed hold / resume: no motion queue to pause */
            continue;
        }
        if (byte == 0x18) { /* Ctrl-X soft reset */
            s_relative = false;
            s_last_motion_g = 1;
            line_len = 0;
            s_fill_recording = false;
            fill_reset();
            s_preview_mode = false;
            /* Abort, don't complete, any pending planned marks - a soft
             * reset is an emergency stop signal, not "finish what was
             * queued" (unlike the flush() call sites above). */
            motion_planner_clear();
            apply_engrave_intent(false);
            grbl_write_str("\r\nGrbl 1.1h ['$' for help]\r\n");
            continue;
        }

        if (byte == '\n' || byte == '\r') {
            if (line_len > 0) {
                line[line_len] = '\0';
                handle_line(line);
                line_len = 0;
            }
            continue;
        }

        if (line_len < GRBL_LINE_MAX - 1)
            line[line_len++] = (char)byte;
        /* else: silently drop overflow bytes until the next line terminator */
    }
}

bool grbl_task_start(void)
{
    /* No driver install needed: stdin/stdout are already wired to
     * USB-Serial-JTAG by the primary console config in sdkconfig.defaults. */
    return xTaskCreatePinnedToCore(grbl_task_fn, "grbl_task", GRBL_TASK_STACK_SIZE,
                                    NULL, GRBL_TASK_PRIORITY, NULL,
                                    GRBL_TASK_CORE) == pdPASS;
}
