#ifndef ZB_CAL_H_
#define ZB_CAL_H_

#include <stdbool.h>
#include <stddef.h>

#include "zb_motion.h"

/* Z-Bro calibration (docs/ZBRO_PLAN.md section 5). Runs in its own task. */

/* Start tests ("endstop","speed","accel","decel","homing"); n == 0 or tests == NULL runs all.
 * Returns 0, or ZB_E_ARG (unknown test), ZB_E_BUSY (running / axis busy), ZB_E_NOT_HOMED, ZB_E_FAULT. */
int  zb_cal_start(const char *const *tests, int n);
void zb_cal_abort(void);
/* malloc'd JSON, shape of GET /api/cal (plan section 6); caller frees. */
char *zb_cal_json(void);
/* Compact JSON (no trials, no allocation) for :7700 and the fleet glue; 0 if it does not fit in n. */
#define ZB_CAL_COMPACT_MAX 900
size_t zb_cal_json_compact(char *buf, size_t n);
/* Counter bumped on every state change except progress ticks (gates EVT CAL pushes). */
unsigned zb_cal_phase(void);
/* Optional: called (from the cal task) whenever the JSON state changed, for WS push. */
typedef void (*zb_cal_cb)(void);
void zb_cal_set_cb(zb_cal_cb cb);

#endif
