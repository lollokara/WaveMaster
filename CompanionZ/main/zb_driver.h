#ifndef ZB_DRIVER_H_
#define ZB_DRIVER_H_

#include <stdbool.h>

/* A4988 pin handling. zb_driver_enable / zb_driver_set_microstep are declared in zb_motion.h. */

/* Drives ENABLE high (motor off) as the very first action, then sets up SLEEP/RESET/MS/DIR/STEP-idle.
 * Safe to call more than once. main() may call it first thing after boot. */
void zb_driver_init(void);
/* Direction pin; positive = true. dir_invert is applied here. */
void zb_driver_set_dir(bool positive);
bool zb_driver_is_enabled(void);
int  zb_driver_microstep(void);

#endif
