#ifndef ZB_ENDSTOP_H_
#define ZB_ENDSTOP_H_

#include <stdbool.h>
#include <stdint.h>

/* Endstop on GPIO5: IRAM-safe ISR latches the first active edge together with a snapshot of a step
 * counter; the debounce (es_debounce_us) is applied by zb_endstop_poll(). */

void zb_endstop_init(volatile int32_t *capture_src);   /* capture_src: read in the ISR on trigger */
bool zb_endstop_active(void);                          /* raw level after es_invert, no debounce */
void zb_endstop_rearm(void);                           /* clear the latch; latch at once if already active */
/* True once a trigger is latched and the pin stayed active for the debounce time. *captured gets the
 * counter value taken at the first edge. A latch that turned out to be a bounce is dropped. */
bool zb_endstop_poll(int32_t *captured);

#endif
