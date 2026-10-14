#ifndef ZB_MOTION_H_
#define ZB_MOTION_H_

#include <stdbool.h>
#include <stdint.h>

/* Z-Bro motion, homing, endstop, driver (docs/ZBRO_PLAN.md). All positions in mm. Thread-safe API;
 * a dedicated "zb_motion" task executes commands, callers never block except where noted. */

enum zb_state { ZB_IDLE, ZB_MOVING, ZB_HOMING, ZB_CAL, ZB_FAULT };

/* Error codes shared with the COM protocol (EVT ERR <code>). */
enum zb_err { ZB_E_NONE = 0, ZB_E_BUSY = 1, ZB_E_NOT_HOMED = 2, ZB_E_RANGE = 3, ZB_E_ARG = 4,
              ZB_E_ENDSTOP = 5, ZB_E_HOME_FAIL = 6, ZB_E_FAULT = 7, ZB_E_UNKNOWN = 8 };

enum zb_evt_type { ZB_EVT_HOMED, ZB_EVT_DONE, ZB_EVT_ERR };
/* Called from the motion task (not an ISR) on completion of HOME/MOVE/JOG/STOP or on error. */
typedef void (*zb_event_cb)(enum zb_evt_type type, float pos_mm, int err);

struct zb_status { float pos; enum zb_state state; bool homed; bool endstop; float speed; };

bool zb_motion_start(zb_event_cb cb);          /* init pins, driver, endstop ISR, RMT; spawn task */
void zb_motion_status(struct zb_status *s);    /* lock-free snapshot, position live during motion */

/* Commands: return ZB_E_NONE if accepted (completion arrives via the event callback), else the error. */
int  zb_home(void);
int  zb_move_abs(float mm, float speed_mm_s);  /* speed 0 = max_speed */
int  zb_move_rel(float dmm, float speed_mm_s);
int  zb_stop(void);                            /* controlled decel (uses decel) */
void zb_estop(void);                           /* halt now, STEP low, axis becomes unhomed; callable from any task */

/* Driver. */
void zb_driver_enable(bool on);                /* ENABLE pin, honours sleep_idle on disable */
void zb_driver_set_microstep(int ms);          /* 1/2/4/8/16; only while idle */

/* Primitives for calibration (zb_cal.c). All blocking, called from the cal task while state == ZB_CAL.
 * Return ZB_E_NONE or an error; they obey zb_cal_abort_requested(). */
bool zb_cal_enter(void);                       /* needs IDLE+homed; sets state ZB_CAL */
void zb_cal_leave(void);
void zb_cal_request_abort(void);
bool zb_cal_abort_requested(void);
/* Move with explicit parameters (bypasses the saved settings), blocks until done. stop_on_endstop: stop
 * the moment the endstop triggers and report the position in *trigger_mm (steps counted exactly). */
int  zb_cal_move(float target_mm, float speed, float accel, float decel, bool stop_on_endstop, float *trigger_mm);
/* Slow, precise probe towards the endstop; returns trigger position in mm using exact step counts. */
int  zb_cal_probe(float speed_mm_s, float *trigger_mm);
float zb_cal_position(void);

#endif
