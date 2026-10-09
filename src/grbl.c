/*
 * GRBL 1.1 front end: line parser, modal state, $ commands, status reports.
 *
 * Reset / abort concurrency design
 * --------------------------------
 * Only the GRBL task ever touches the parser state (s_m). The RX task
 * (host_serial.c) acts on realtime bytes by itself:
 *
 *   0x18  on_reset():  s_reset_gen++            (1st: so the GRBL task sees it)
 *                      galvo_out_hold(false) if held
 *                      motion_abort(&x,&y)      (laser off NOW, wakes a blocked
 *                                                motion_submit() with false)
 *         then host_serial pushes a one-byte mark (0x18) into the RX stream.
 *
 *   GRBL task: while s_reset_gen != s_gen_seen it discards every byte it
 *   reads (those were received before the reset) up to the mark; replies for
 *   a line that was executing during the reset are suppressed; submit()
 *   refuses to enqueue once the generation changed. At the mark it runs
 *   motion_abort() a second time (this catches the one segment that may have
 *   slipped in between its generation check and the first abort), adopts the
 *   returned position, resets the modal state, clears the line buffer,
 *   acknowledges all consumed bytes and prints the banner. Bytes after the
 *   mark are normal input again. s_m is therefore never touched by the RX
 *   task, and nothing before the mark can execute after the abort.
 *
 *   0x85 (jog cancel) aborts motion from the RX task and hands the position
 *   over through s_jc_*; the GRBL task adopts it before its next line.
 */

#include "grbl.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "atmega_link.h"
#include "calib.h"
#include "dac_task.h"
#include "ad3552r_board.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "galvo_out.h"
#include "gcode_parse.h"
#include "host_serial.h"
#include "laser_io.h"
#include "motion.h"
#include "motion_task.h"

static const char *TAG = "grbl";

#define LINE_MAX_LEN    256
#define RX_WINDOW       1024
#define GRBL_TASK_STACK 6144
#define GRBL_TASK_PRIO  9
#define E_ABORT         (-1)     /* line interrupted by a reset / abort */
#define MM_PER_INCH     25.4f
#define WCO_PERIOD      10

struct modal {
    uint8_t motion;              /* 0..3 = G0..G3 */
    bool relative;               /* G91 */
    bool inches;                 /* G20 */
    uint8_t laser;               /* 0 = M5, 3 = M3, 4 = M4 */
    uint8_t wcs;                 /* 0..5 = G54..G59 */
    bool preview;                /* M66 */
    int tool;
    float feed;                  /* mm/min, 0 = never set */
    float s;
    float x, y;                  /* commanded end of the last submitted segment, machine mm */
};

static struct modal s_m;
static float s_wcs_off[6][2];
static float s_g92[2];

static bool s_fault;
/* $231: guide laser switched on automatically for a frame (moves under
 * M3 S0, which is what Rayforge's Frame sends - real jobs use M4). */
static bool s_auto_guide;
static bool s_clamp_warned;
static uint32_t s_clamp_count;

/* Hand-over state between tasks (see header comment). */
static volatile uint32_t s_reset_gen;
static uint32_t s_gen_seen;               /* GRBL task only */
static volatile float s_jc_x, s_jc_y;
static volatile bool s_jc_pending;
static volatile bool s_jog_active;
static volatile uint32_t s_acked;         /* bytes the host may consider free */
static uint32_t s_consumed;               /* GRBL task only */
static volatile float s_pub_wco_x, s_pub_wco_y, s_pub_feed, s_pub_s;
static volatile bool s_wco_dirty = true;
static int s_wco_count;

/* ------------------------------------------------------------ helpers -- */

static float fix0(float v)
{
    return fabsf(v) < 0.0005f ? 0.0f : v;
}

static float to_mm(float v)
{
    return s_m.inches ? v * MM_PER_INCH : v;
}

static void publish(void)
{
    float wx = s_wcs_off[s_m.wcs][0] + s_g92[0];
    float wy = s_wcs_off[s_m.wcs][1] + s_g92[1];

    if (wx != s_pub_wco_x || wy != s_pub_wco_y)
        s_wco_dirty = true;
    s_pub_wco_x = wx;
    s_pub_wco_y = wy;
    s_pub_feed = s_m.feed;
    s_pub_s = s_m.s;
}

static void modal_reset(void)
{
    if ((s_m.preview || s_auto_guide) && !s_fault)
        atmega_link_set_guide(false);
    s_auto_guide = false;
    s_m.motion = 0;
    s_m.relative = false;
    s_m.inches = false;
    s_m.laser = 0;
    s_m.wcs = 0;
    s_m.preview = false;
    s_m.s = 0.0f;
    s_clamp_warned = false;
    publish();
    s_wco_dirty = true;
}

static void print_banner(void)
{
    host_puts("\r\nGrbl 1.1h ['$' for help]\r\n");
}

static void work_limits(float *x0, float *x1, float *y0, float *y1)
{
    float w = calib_get(CAL_FIELD_W), h = calib_get(CAL_FIELD_H);

    if (calib_get(CAL_ORIGIN_CENTER) > 0.5f) {
        *x0 = -w / 2; *x1 = w / 2; *y0 = -h / 2; *y1 = h / 2;
    } else {
        *x0 = 0; *x1 = w; *y0 = 0; *y1 = h;
    }
}

static void clamp_target(float *x, float *y)
{
    float x0, x1, y0, y1, cx = *x, cy = *y;

    work_limits(&x0, &x1, &y0, &y1);
    if (cx < x0) cx = x0;
    if (cx > x1) cx = x1;
    if (cy < y0) cy = y0;
    if (cy > y1) cy = y1;
    if (cx != *x || cy != *y) {
        s_clamp_count++;
        if (!s_clamp_warned) {
            s_clamp_warned = true;
            ESP_LOGW(TAG, "target (%.3f,%.3f) outside work area, clamped (counted, logged once per job)",
                     (double)*x, (double)*y);
        }
        *x = cx;
        *y = cy;
    }
}

/* Waits for the motion pipeline to drain. False if a reset interrupted it. */
static bool wait_idle(void)
{
    while (!s_fault && motion_busy()) {
        if (s_reset_gen != s_gen_seen)
            return false;
        vTaskDelay(1);
    }
    return s_reset_gen == s_gen_seen;
}

static bool submit(uint8_t type, float x, float y, uint8_t power, float speed, float dwell_us)
{
    struct motion_seg seg = {
        .type = type, .power = power, .x = x, .y = y,
        .speed_mm_s = speed, .dwell_us = dwell_us,
    };

    if (s_reset_gen != s_gen_seen)
        return false;
    if (!motion_submit(&seg, UINT32_MAX))
        return false;
    if (type != MOTION_DWELL) {
        s_m.x = x;
        s_m.y = y;
    }
    return true;
}

static uint8_t current_power(void)
{
    if (s_m.preview || (s_m.laser != 3 && s_m.laser != 4) || !(s_m.s > 0.0f))
        return 0;
    return gcode_power_byte(s_m.s, calib_get(CAL_S_MAX), calib_get(CAL_POWER_MIN),
                            calib_get(CAL_POWER_MAX));
}

/* --------------------------------------------------------- G-code ------ */

static int exec_gcode(const char *line, bool jog)
{
    struct gcode_block b;
    int e = gcode_parse_line(line, &b);
    int motion = -1;
    int g_dwell = 0, g_g10 = 0, g92 = 0, g53 = 0, end_prog = 0;
    int new_wcs = -1;
    bool have_x = false, have_y = false, have_i = false, have_j = false, have_r = false;
    float ax = 0, ay = 0, iv = 0, jv = 0, rv = 0, pv = 0, lv = 0;
    bool has_p = false, has_l = false;
    int arm = -1, guide = -1, preview = -1;

    if (e)
        return e;
    if (b.n == 0)
        return 0;

    /* Pass 1: classify. Units are applied after we know G20/G21 of this block. */
    int units = -1, dist = -1;

    for (int k = 0; k < b.n; k++) {
        float v = b.w[k].value;

        switch (b.w[k].letter) {
        case 'G': {
            int g = (int)lroundf(v * 10.0f);

            switch (g) {
            case 0: case 10: case 20: case 30: motion = g / 10; break;
            case 40: g_dwell = 1; break;
            case 100: g_g10 = 1; break;
            case 170: case 180: case 190:
            case 280: case 281: case 300: case 301:
            case 400: case 430: case 490: case 610: case 640: case 800:
            case 911: case 940:
                break;
            case 200: units = 1; break;
            case 210: units = 0; break;
            case 530: g53 = 1; break;
            case 540: case 550: case 560: case 570: case 580: case 590:
                new_wcs = (g - 540) / 10;
                break;
            case 900: dist = 0; break;
            case 910: dist = 1; break;
            case 920: g92 = 1; break;
            case 921: case 922: case 923: g92 = 2; break;
            default: return GCODE_ERR_UNSUPPORTED;
            }
            break;
        }
        case 'M': {
            int m = (int)lroundf(v);

            switch (m) {
            case 0: case 1: case 6: case 7: case 8: case 9:
            case 64: case 65: case 68: case 69:
                break;
            case 2: case 30: end_prog = 1; break;
            case 3: case 4: case 5: break;      /* applied in pass 2 */
            case 10: arm = 1; break;
            case 11: arm = 0; break;
            case 62: guide = 1; break;
            case 63: guide = 0; break;
            case 66: preview = 1; break;
            case 67: preview = 0; break;
            default: return GCODE_ERR_UNSUPPORTED;
            }
            break;
        }
        case 'X': have_x = true; ax = v; break;
        case 'Y': have_y = true; ay = v; break;
        case 'I': have_i = true; iv = v; break;
        case 'J': have_j = true; jv = v; break;
        case 'R': have_r = true; rv = v; break;
        case 'P': has_p = true; pv = v; break;
        case 'L': has_l = true; lv = v; break;
        default: break;                          /* F S T Z A B C ...: handled below / ignored */
        }
    }

    if (jog) {
        if (!have_x && !have_y)
            return GCODE_ERR_BAD_TARGET;
        motion = 0;
    }

    /* Pass 2: modal updates in GRBL order. */
    if (units >= 0)
        s_m.inches = (units == 1);
    if (dist >= 0)
        s_m.relative = (dist == 1);
    if (new_wcs >= 0)
        s_m.wcs = (uint8_t)new_wcs;

    for (int k = 0; k < b.n; k++) {
        float v = b.w[k].value;

        switch (b.w[k].letter) {
        case 'F':
            if (v > 0.0f)
                s_m.feed = to_mm(v);     /* a $J feed is undone by the $J wrapper */
            break;
        case 'S':
            s_m.s = v > 0.0f ? v : 0.0f;
            break;
        case 'T':
            s_m.tool = (int)v;
            break;
        case 'M': {
            int m = (int)lroundf(v);

            if (m == 3 || m == 4 || m == 5)
                s_m.laser = (uint8_t)m;
            break;
        }
        default:
            break;
        }
    }
    if (!s_fault) {
        /* Arm, guide and preview act on the ATmega at once, so they must not
         * overtake motion that is still queued (GRBL treats M62/M63 the same
         * way): M11 would disarm in the middle of the previous job, M67 would
         * drop the guide laser halfway through a frame. Wait for the queue
         * to drain first; same handling as G4. */
        if ((arm >= 0 || guide >= 0 || preview >= 0) && !wait_idle())
            return E_ABORT;
        if (arm >= 0)
            atmega_link_set_armed(arm == 1);
        if (guide >= 0)
            atmega_link_set_guide(guide == 1);
        if (preview >= 0) {
            s_m.preview = (preview == 1);
            atmega_link_set_guide(s_m.preview);
        }
        if (guide >= 0 || preview >= 0)
            s_auto_guide = false;           /* the user took over the guide laser */

        /* Auto guide ends when the line leaves "M3 with S0" (M5 at the end of
         * Rayforge's frame, or M4 / a real power for a job). Wait for the
         * frame's queued moves first so the guide stays on to the end. */
        if (s_auto_guide && !(s_m.laser == 3 && !(s_m.s > 0.0f))) {
            if (!wait_idle())
                return E_ABORT;
            atmega_link_set_guide(s_m.preview);
            s_auto_guide = false;
        }
    }

    float wco_x = s_wcs_off[s_m.wcs][0] + s_g92[0];
    float wco_y = s_wcs_off[s_m.wcs][1] + s_g92[1];

    /* Non-modal commands. */
    if (g_g10) {
        int idx = s_m.wcs;

        if (!has_l)
            return GCODE_ERR_UNSUPPORTED;
        if (has_p && pv > 0.5f) {
            idx = (int)lroundf(pv) - 1;
            if (idx < 0 || idx > 5)
                return GCODE_ERR_UNSUPPORTED;
        }
        if (lroundf(lv) == 2) {
            if (have_x) s_wcs_off[idx][0] = to_mm(ax);
            if (have_y) s_wcs_off[idx][1] = to_mm(ay);
        } else if (lroundf(lv) == 20) {
            if (have_x) s_wcs_off[idx][0] = s_m.x - s_g92[0] - to_mm(ax);
            if (have_y) s_wcs_off[idx][1] = s_m.y - s_g92[1] - to_mm(ay);
        } else {
            return GCODE_ERR_UNSUPPORTED;
        }
        have_x = have_y = false;
        publish();
        return 0;
    }
    if (g92 == 1) {
        if (have_x) s_g92[0] = s_m.x - s_wcs_off[s_m.wcs][0] - to_mm(ax);
        if (have_y) s_g92[1] = s_m.y - s_wcs_off[s_m.wcs][1] - to_mm(ay);
        publish();
        return 0;
    }
    if (g92 == 2) {
        s_g92[0] = s_g92[1] = 0.0f;
        publish();
        return 0;
    }
    publish();

    if (g_dwell) {
        if (s_fault)
            return 0;
        if (!wait_idle())
            return E_ABORT;
        if (has_p && pv > 0.001f) {
            if (!submit(MOTION_DWELL, s_m.x, s_m.y, 0, 0.0f, pv * 1.0e6f))
                return E_ABORT;
        }
        return 0;
    }

    /* Motion. */
    if (motion < 0)
        motion = s_m.motion;
    else if (!jog)
        s_m.motion = (uint8_t)motion;

    bool arc = (motion == 2 || motion == 3);
    bool moving = have_x || have_y || (arc && (have_i || have_j || have_r));

    if (moving) {
        float tx, ty;
        float sx = s_m.x, sy = s_m.y;

        if (s_fault)
            return 9;
        if (s_m.relative && !g53) {
            tx = sx + (have_x ? to_mm(ax) : 0.0f);
            ty = sy + (have_y ? to_mm(ay) : 0.0f);
        } else {
            tx = have_x ? to_mm(ax) + (g53 ? 0.0f : wco_x) : sx;
            ty = have_y ? to_mm(ay) + (g53 ? 0.0f : wco_y) : sy;
        }
        clamp_target(&tx, &ty);

        if (motion == 0) {
            if (fabsf(tx - sx) > 1e-5f || fabsf(ty - sy) > 1e-5f) {
                if (!submit(MOTION_JUMP, tx, ty, 0, 0.0f, 0.0f))
                    return E_ABORT;
                if (jog)
                    s_jog_active = true;
            }
        } else {
            if (!s_auto_guide && s_m.laser == 3 && !(s_m.s > 0.0f) && !s_m.preview &&
                calib_get(CAL_AUTO_GUIDE) > 0.5f) {
                atmega_link_set_guide(true);      /* framing: show it with the guide laser */
                s_auto_guide = true;
            }
            float speed = gcode_mark_speed(s_m.feed, calib_get(CAL_MAX_MARK_RATE),
                                           calib_get(CAL_DEFAULT_MARK_SPEED));
            uint8_t pw = current_power();

            if (motion == 1) {
                if (fabsf(tx - sx) > 1e-5f || fabsf(ty - sy) > 1e-5f) {
                    if (!submit(MOTION_MARK, tx, ty, pw, speed, 0.0f))
                        return E_ABORT;
                }
            } else {
                struct gcode_arc a;
                float px, py;
                bool cw = (motion == 2);
                float tol = calib_get(CAL_ARC_TOL);
                int ae;

                if (have_r && !have_i && !have_j)
                    ae = gcode_arc_init_r(&a, sx, sy, tx, ty, to_mm(rv), cw, tol);
                else
                    ae = gcode_arc_init(&a, sx, sy, tx, ty, have_i ? to_mm(iv) : 0.0f,
                                        have_j ? to_mm(jv) : 0.0f, cw, tol);
                if (ae)
                    return ae;
                while (gcode_arc_next(&a, &px, &py)) {
                    clamp_target(&px, &py);
                    if (!submit(MOTION_MARK, px, py, pw, speed, 0.0f))
                        return E_ABORT;
                }
            }
        }
    }

    if (end_prog) {
        if (!wait_idle())
            return E_ABORT;
        s_m.laser = 0;
        s_m.relative = false;
        s_clamp_warned = false;
        if (s_auto_guide) {
            atmega_link_set_guide(s_m.preview);
            s_auto_guide = false;
        }
        if (s_clamp_count) {
            ESP_LOGW(TAG, "%u targets were clamped to the work area", (unsigned)s_clamp_count);
            s_clamp_count = 0;
        }
        publish();
    }
    return 0;
}

/* ------------------------------------------------------ $ commands ----- */

static void out_line(const char *s)
{
    host_puts(s);
}

static bool is_ro_setting(int n)
{
    static const int ro[] = { 0, 1, 2, 4, 5, 6, 10, 11, 13, 20, 21, 22, 23, 24, 25,
                              26, 27, 31, 32, 102, 112, 122, 132 };

    for (size_t i = 0; i < sizeof(ro) / sizeof(ro[0]); i++)
        if (ro[i] == n)
            return true;
    return false;
}

static void print_gc(void)
{
    static const char *const g_motion[] = { "G0", "G1", "G2", "G3" };
    char fbuf[24], sbuf[24];

    snprintf(fbuf, sizeof(fbuf), "%g", (double)s_m.feed);
    snprintf(sbuf, sizeof(sbuf), "%g", (double)s_m.s);
    host_printf("[GC:%s G%d G17 %s %s G94 M%d M9 T%d F%s S%s]\r\n", g_motion[s_m.motion & 3],
                54 + s_m.wcs, s_m.inches ? "G20" : "G21", s_m.relative ? "G91" : "G90",
                s_m.laser, s_m.tool, fbuf, sbuf);
}

static void print_params(void)
{
    for (int i = 0; i < 6; i++)
        host_printf("[G%d:%.3f,%.3f,0.000]\r\n", 54 + i, (double)fix0(s_wcs_off[i][0]),
                    (double)fix0(s_wcs_off[i][1]));
    host_puts("[G28:0.000,0.000,0.000]\r\n[G30:0.000,0.000,0.000]\r\n");
    host_printf("[G92:%.3f,%.3f,0.000]\r\n", (double)fix0(s_g92[0]), (double)fix0(s_g92[1]));
    host_puts("[TLO:0.000]\r\n[PRB:0.000,0.000,0.000:0]\r\n");
}

static void print_stats(void)
{
    struct galvo_out_stats gs;
    struct atmega_status as;

    if (s_fault) {
        host_puts("[MSG:motion hardware fault]\r\n");
    } else {
        galvo_out_get_stats(&gs);
        host_printf("[MSG:galvo chunks=%u underruns=%u barriers=%u ticks=%llu tick_us=%.2f clipped=%u]\r\n",
                    (unsigned)gs.chunks, (unsigned)gs.underruns, (unsigned)gs.barriers,
                    (unsigned long long)gs.ticks, (double)galvo_out_tick_us(), (unsigned)gs.clipped);
    }
    atmega_link_get_status(&as);
    host_printf("[MSG:atmega link=%d armed=%d ready=%d guide=%d power=%u stat=0x%02x vdet=%umV err=%u]\r\n",
                as.link_ok, as.armed, as.ready, as.guide_on, (unsigned)as.power,
                (unsigned)as.stat_bits, (unsigned)as.vdetect_mv, (unsigned)as.errors);
    host_printf("[MSG:host rx_dropped=%u clamped=%u]\r\n", (unsigned)host_rx_dropped(),
                (unsigned)s_clamp_count);
    {
        struct laser_io_state io;

        laser_io_get_state(&io);
        host_printf("[MSG:io gate=%d kill=%d gate_active_low=%d sync_hz=%lu]\r\n",
                    io.gate_on, io.killed, io.active_low, (unsigned long)io.prr_hz);
        host_printf("[MSG:dac config_repairs=%lu]\r\n",
                    (unsigned long)ad3552r_board_guard_repairs());
    }
}

static int dollar_setting(const char *line)
{
    const char *p = line + 1;
    float num, val;
    int n;

    if (gcode_parse_number(&p, &num))
        return 3;
    while (*p == ' ')
        p++;
    if (*p != '=')
        return 3;
    p++;
    while (*p == ' ')
        p++;
    if (gcode_parse_number(&p, &val))
        return 3;
    n = (int)lroundf(num);
    if (calib_set_param(n, val) != 0) {
        if (is_ro_setting(n)) {          /* GRBL settings we only mirror: accept quietly */
            ESP_LOGW(TAG, "$%d is fixed on this controller, ignored", n);
            return 0;
        }
        return 3;
    }
    if (!s_fault) {
        if (!motion_busy() && !galvo_out_busy()) {
            galvo_out_reload();
            /* PRR / SYNC duty take effect at once (they used to wait for
             * a reboot). */
            if (n == 220 || n == 221)
                laser_io_set_prr(calib_get(CAL_PRR_HZ), calib_get(CAL_PRR_DUTY));
        }
        motion_reload_params();
    }
    return 0;
}

static int exec_jog(const char *payload)
{
    struct modal saved = s_m;
    int e;

    e = exec_gcode(payload, true);
    saved.x = s_m.x;
    saved.y = s_m.y;
    s_m = saved;
    publish();
    return e;
}

/* Holds the gate on for ms with the galvo standing still, polling for a soft
 * reset. Only called while the stream is closed (nothing else drives the
 * gate then); Ctrl-X also kills the gate through motion_abort(). */
static void gate_pulse(uint32_t ms)
{
    uint32_t gen = s_reset_gen;

    laser_io_gate(true);
    for (uint32_t t = 0; t < ms && s_reset_gen == gen; t++)
        vTaskDelay(1);
    laser_io_gate(false);
}

static bool parse_args(const char *p, float *v, int n)
{
    for (int i = 0; i < n; i++) {
        if (gcode_parse_number(&p, &v[i]))
            return false;
        while (*p == ' ')
            p++;
        if (i + 1 < n) {
            if (*p != ',')
                return false;
            p++;
        }
    }
    return *p == '\0';
}

/*
 * $GT=<ms>  gate wiring test: drives EMISSION MODULATION (GPIO4, DB25 pin 19)
 *           active for up to 5 s so it can be checked with a meter/scope.
 *           Refused unless the laser is DISARMED (no MO, so no emission).
 * $LT=<ms>,<S>  static laser test: sets the power for S, then opens the gate
 *           for up to 2 s with the galvo standing still. Refused unless the
 *           laser is armed AND ready; burns one spot.
 */
static int laser_test(const char *p, bool fire)
{
    struct atmega_status st;
    float v[2] = { 0.0f, 0.0f };

    if (!parse_args(p, v, fire ? 2 : 1) || v[0] < 1.0f)
        return 3;
    if (s_fault || motion_busy() || galvo_out_busy()) {
        host_puts("[MSG:test refused: motion in progress]\r\n");
        return 0;
    }
    atmega_link_get_status(&st);
    if (!fire) {
        uint32_t ms = v[0] > 5000.0f ? 5000u : (uint32_t)v[0];

        if (st.armed) {
            host_puts("[MSG:GT refused: laser is armed - send M11 first]\r\n");
            return 0;
        }
        host_printf("[MSG:GT gate (GPIO4 / DB25 pin 19) active for %lu ms]\r\n", (unsigned long)ms);
        gate_pulse(ms);
        host_puts("[MSG:GT done, gate inactive]\r\n");
        return 0;
    }
    {
        uint32_t ms = v[0] > 2000.0f ? 2000u : (uint32_t)v[0];
        uint8_t pw = gcode_power_byte(v[1], calib_get(CAL_S_MAX), calib_get(CAL_POWER_MIN),
                                      calib_get(CAL_POWER_MAX));
        struct laser_io_state io;

        if (!st.link_ok || !st.armed || !st.ready) {
            host_printf("[MSG:LT refused: link=%d armed=%d ready=%d (M10, then wait ~2 s)]\r\n",
                        st.link_ok, st.armed, st.ready);
            return 0;
        }
        if (pw == 0) {
            host_puts("[MSG:LT refused: S maps to power 0]\r\n");
            return 0;
        }
        atmega_link_set_power(pw);
        if (!atmega_link_wait_power(pw, 200)) {
            host_printf("[MSG:LT refused: ATmega did not confirm power %u]\r\n", pw);
            return 0;
        }
        laser_io_get_state(&io);
        host_printf("[MSG:LT power=%u sync_hz=%lu gate %lu ms]\r\n", pw,
                    (unsigned long)io.prr_hz, (unsigned long)ms);
        gate_pulse(ms);
        host_puts("[MSG:LT done, gate inactive]\r\n");
        return 0;
    }
}

static int exec_dollar(char *line)
{
    char *p = line;
    size_t len;

    for (len = 0; p[len]; len++)
        if (p[len] >= 'a' && p[len] <= 'z')
            p[len] = (char)(p[len] - 'a' + 'A');
    while (len && p[len - 1] == ' ')
        p[--len] = '\0';

    if (len == 1) {
        host_puts("[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $SLP $C $X $H $S $RB $RD $PW=word $GT=ms $LT=ms,S ~ ! ? ctrl-x]\r\n");
        return 0;
    }
    if (strcmp(p, "$$") == 0) {
        calib_dump(out_line);
        return 0;
    }
    if (strncmp(p, "$J=", 3) == 0)
        return exec_jog(p + 3);
    if (strcmp(p, "$I") == 0) {
        host_printf("[VER:1.1h.WaveMaster:]\r\n[OPT:V,%d,%d]\r\n[MSG:machine:WaveMaster Galvo]\r\n",
                    (int)MOTION_QUEUE_LEN, RX_WINDOW);
        return 0;
    }
    if (strcmp(p, "$G") == 0) {
        print_gc();
        return 0;
    }
    if (strcmp(p, "$#") == 0) {
        print_params();
        return 0;
    }
    if (strcmp(p, "$X") == 0 || strcmp(p, "$H") == 0 || strcmp(p, "$C") == 0 ||
        strcmp(p, "$SLP") == 0)
        return 0;
    if (strcmp(p, "$N") == 0) {
        host_puts("$N0=\r\n$N1=\r\n");
        return 0;
    }
    if (strncmp(p, "$N", 2) == 0 && strchr(p, '='))
        return 0;                                /* startup blocks are not stored */
    if (strcmp(p, "$RB") == 0) {
        uint16_t cx, cy;

        if (s_fault || motion_busy() || galvo_out_busy() || !dac_task_readback(&cx, &cy))
            host_puts("[MSG:RB unavailable (busy or SPI error)]\r\n");
        else
            host_printf("[MSG:RB X=0x%04X Y=0x%04X]\r\n", cx, cy);
        return 0;
    }
    if (strncmp(p, "$PW=", 4) == 0) {
        /* Power-word wiring test: latch a raw 0-255 word on DB25 pins 1-8
         * (via the ATmega) without motion or gate, so the bits can be
         * checked with a meter. The next job re-applies its own power. */
        const char *q = p + 4;
        float v;

        if (gcode_parse_number(&q, &v) || *q || v < 0.0f || v > 255.0f)
            return 3;
        if (s_fault || motion_busy() || galvo_out_busy()) {
            host_puts("[MSG:PW refused: motion in progress]\r\n");
            return 0;
        }
        {
            uint8_t w = (uint8_t)(v + 0.5f);
            bool ok;

            atmega_link_set_power(w);
            ok = atmega_link_wait_power(w, 300);
            host_printf("[MSG:PW word=%u (0x%02X) bits D7..D0=%u%u%u%u%u%u%u%u %s]\r\n", w, w,
                        (w >> 7) & 1, (w >> 6) & 1, (w >> 5) & 1, (w >> 4) & 1,
                        (w >> 3) & 1, (w >> 2) & 1, (w >> 1) & 1, w & 1,
                        ok ? "latched" : "NOT confirmed by the ATmega");
        }
        return 0;
    }
    if (strncmp(p, "$GT=", 4) == 0)
        return laser_test(p + 4, false);
    if (strncmp(p, "$LT=", 4) == 0)
        return laser_test(p + 4, true);
    if (strcmp(p, "$RD") == 0) {
        const uint8_t *a = NULL;
        uint16_t v[16];
        int n = (s_fault || motion_busy() || galvo_out_busy()) ? -1 : dac_task_regdump(&a, v, 16);

        if (n <= 0) {
            host_puts("[MSG:RD unavailable (busy or SPI error)]\r\n");
        } else {
            char line[160];
            int o = snprintf(line, sizeof(line), "[MSG:RD");

            for (int i = 0; i < n && o < (int)sizeof(line) - 12; i++)
                o += snprintf(line + o, sizeof(line) - o, " %02X=%02X", a[i], v[i] & 0xFF);
            snprintf(line + o, sizeof(line) - o, "]\r\n");
            host_puts(line);
        }
        return 0;
    }
    if (strcmp(p, "$S") == 0) {
        print_stats();
        return 0;
    }
    if (strchr(p, '='))
        return dollar_setting(p);
    return 3;
}

static int process_line(char *line)
{
    char *p = line;

    while (*p == ' ' || *p == '\t')
        p++;
    if (*p == '$')
        return exec_dollar(p);
    return exec_gcode(p, false);
}

/* --------------------------------------------------- status report ----- */

static void report_status(void)
{
    const char *state;
    float mx, my;
    bool busy = false;
    int feed;
    char wco[48] = "";

    if (s_fault) {
        state = "Alarm";
        mx = s_m.x;
        my = s_m.y;
    } else {
        busy = motion_busy();
        if (s_jog_active && !busy)
            s_jog_active = false;
        state = galvo_out_is_held() ? "Hold" : (s_jog_active ? "Jog" : (busy ? "Run" : "Idle"));
        galvo_out_position(&mx, &my);
    }
    feed = (int)lroundf(s_pub_feed > 0.0f ? s_pub_feed : calib_get(CAL_DEFAULT_MARK_SPEED) * 60.0f);

    if (s_wco_dirty || ++s_wco_count >= WCO_PERIOD) {
        s_wco_dirty = false;
        s_wco_count = 0;
        snprintf(wco, sizeof(wco), "|WCO:%.3f,%.3f,0.000", (double)fix0(s_pub_wco_x),
                 (double)fix0(s_pub_wco_y));
    }
    {
        int rxfree = RX_WINDOW - (int)(host_rx_accepted() - s_acked);

        if (rxfree < 0)
            rxfree = 0;
        if (rxfree > RX_WINDOW)
            rxfree = RX_WINDOW;
        host_printf("<%s|MPos:%.3f,%.3f,0.000|FS:%d,%d|Bf:%d,%d%s>\r\n", state,
                    (double)fix0(mx), (double)fix0(my), feed, (int)lroundf(s_pub_s),
                    s_fault ? 0 : (int)motion_queue_free(), rxfree, wco);
    }
}

/* ------------------------------------------- realtime (RX task ctx) ---- */

static void on_status(void)
{
    report_status();
}

static void on_hold(void)
{
    if (!s_fault)
        galvo_out_hold(true);
}

static void on_resume(void)
{
    if (!s_fault)
        galvo_out_hold(false);
}

static void on_reset(void)
{
    s_reset_gen++;                       /* first: the GRBL task starts discarding */
    /* Stop / Ctrl-X is an emergency stop: drop EMISSION ENABLE (MO) and the
     * power word on the ATmega too, not just the gate. Asynchronous, so it
     * does not delay the abort below; also done in fault mode. The next job
     * re-arms by itself ($226). */
    atmega_link_set_armed(false);
    if (!s_fault) {
        float x, y;

        if (galvo_out_is_held())
            galvo_out_hold(false);
        motion_abort(&x, &y);
        s_jog_active = false;
    }
}

static void on_jog_cancel(void)
{
    float x, y;

    if (s_fault || !s_jog_active)
        return;
    motion_abort(&x, &y);
    s_jc_x = x;
    s_jc_y = y;
    s_jc_pending = true;
    s_jog_active = false;
}

/* ---------------------------------------------------------- task ------- */

static void reply(int err)
{
    if (err == 0)
        host_puts("ok\r\n");
    else
        host_printf("error:%d\r\n", err);
}

static void handle_reset_mark(void)
{
    s_gen_seen++;
    if (!s_fault) {
        float x, y;

        motion_abort(&x, &y);            /* cleanup: see header comment */
        s_m.x = x;
        s_m.y = y;
    }
    s_jc_pending = false;
    s_jog_active = false;
    modal_reset();
    s_acked = s_consumed;
    print_banner();
}

static void run_line(char *line, bool overflow)
{
    int err;

    if (s_jc_pending) {
        s_jc_pending = false;
        s_m.x = s_jc_x;
        s_m.y = s_jc_y;
    }
    err = overflow ? GCODE_ERR_OVERFLOW : process_line(line);
    if (s_reset_gen != s_gen_seen)
        return;                          /* reset happened meanwhile: no reply */
    if (err == E_ABORT)
        err = 0;
    reply(err);
    s_acked = s_consumed;
}

static void grbl_task(void *arg)
{
    char line[LINE_MAX_LEN];
    size_t len = 0;
    bool overflow = false, last_cr = false;
    uint8_t buf[64];

    (void)arg;
    print_banner();
    for (;;) {
        size_t n = host_rx_read(buf, sizeof(buf), 100);

        for (size_t i = 0; i < n; i++) {
            uint8_t c = buf[i];

            if (c == HOST_RX_RESET_MARK) {
                handle_reset_mark();
                len = 0;
                overflow = false;
                last_cr = false;
                continue;
            }
            s_consumed++;
            if (s_reset_gen != s_gen_seen)
                continue;                /* received before a reset: discard */
            if (c == '\n' && last_cr) {  /* second half of CRLF */
                last_cr = false;
                continue;
            }
            last_cr = (c == '\r');
            if (c == '\n' || c == '\r') {
                line[len] = '\0';
                run_line(line, overflow);
                len = 0;
                overflow = false;
                continue;
            }
            if (overflow)
                continue;
            if (len >= LINE_MAX_LEN - 1) {
                overflow = true;
                continue;
            }
            line[len++] = (char)c;
        }
    }
}

void grbl_set_fault(const char *reason)
{
    s_fault = true;
    ESP_LOGE(TAG, "motion hardware unavailable (%s): answering host, motion disabled",
             reason ? reason : "?");
}

bool grbl_task_start(void)
{
    static const struct host_rt_handlers h = {
        .status = on_status,
        .hold = on_hold,
        .resume = on_resume,
        .reset = on_reset,
        .jog_cancel = on_jog_cancel,
    };

    memset(&s_m, 0, sizeof(s_m));
    s_m.inches = false;
    modal_reset();
    if (!s_fault) {
        float x, y;

        galvo_out_position(&x, &y);
        s_m.x = x;
        s_m.y = y;
    }
    host_serial_set_handlers(&h);
    return xTaskCreatePinnedToCore(grbl_task, "grbl", GRBL_TASK_STACK, NULL, GRBL_TASK_PRIO,
                                   NULL, 1) == pdPASS;
}
