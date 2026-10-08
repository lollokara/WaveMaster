#ifndef GRBL_H_
#define GRBL_H_

#include <stdbool.h>

/*
 * GRBL 1.1 protocol front end (core 1).
 *
 * Threads:
 *   host_rx task (host_serial.c, prio 10)  realtime bytes: '?' '!' '~' 0x18 0x85
 *   grbl task    (this file,    prio 9)    line parser / executor
 *
 * The parser state (modal state, position) belongs to the grbl task alone.
 * The RX task only touches: the motion/galvo abort calls, a handful of
 * volatile hand-over variables and a reset generation counter - see the
 * "reset / abort" comment in grbl.c.
 */

/* Registers the realtime handlers and starts the GRBL task. Works without
 * motion hardware (see grbl_set_fault) so the host still gets answers. */
bool grbl_task_start(void);

/* Call before grbl_task_start() if the motion hardware could not be brought
 * up: status reports then say Alarm, motion commands answer error:9, and no
 * motion/galvo function is ever called. */
void grbl_set_fault(const char *reason);

#endif /* GRBL_H_ */
