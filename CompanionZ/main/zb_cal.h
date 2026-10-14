#ifndef ZB_CAL_H_
#define ZB_CAL_H_

#include <stdbool.h>

/* Z-Bro calibration (docs/ZBRO_PLAN.md section 5). Runs in its own task. */

/* Start tests ("endstop","speed","accel","decel","homing"); n == 0 or tests == NULL runs all.
 * False if already running, not homed/idle, or an unknown test name. */
bool zb_cal_start(const char *const *tests, int n);
void zb_cal_abort(void);
/* malloc'd JSON, shape of GET /api/cal (plan section 6); caller frees. */
char *zb_cal_json(void);
/* Optional: called (from the cal task) whenever the JSON state changed, for WS push. */
typedef void (*zb_cal_cb)(void);
void zb_cal_set_cb(zb_cal_cb cb);

#endif
