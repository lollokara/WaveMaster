#ifndef ZB_ENDSTOP_H_
#define ZB_ENDSTOP_H_

#include <stdbool.h>
#include <stdint.h>

/* Endstop on GPIO5: IRAM-safe ISR latches the first active edge together with a snapshot of a step
 * counter; the debounce (es_debounce_us) is applied by zb_endstop_poll(). */

/* capture_fn runs in the ISR on trigger (must be IRAM_ATTR, no locks) and returns the step position. */
void zb_endstop_init(int32_t (*capture_fn)(void));
bool zb_endstop_active(void);                          /* raw level after es_invert, no debounce */
void zb_endstop_rearm(void);                           /* clear the latch; latch at once if already active */
/* True once a trigger is latched and the pin stayed active for the debounce time. *captured gets the
 * counter value taken at the first edge. A latch that turned out to be a bounce is dropped. */
bool zb_endstop_poll(int32_t *captured);

#endif
