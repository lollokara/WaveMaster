#ifndef GRBL_TASK_H_
#define GRBL_TASK_H_

#include <stdbool.h>

/* Starts the GRBL-protocol UART task pinned to core 1. Must be called
 * after laser_ctrl_init(). Speaks a minimal subset of the GRBL 1.1 serial
 * protocol (line-based G-code + a few realtime single-char commands) over
 * UART0, so GRBL-compatible senders (e.g. rayforge) can drive it. */
bool grbl_task_start(void);

#endif /* GRBL_TASK_H_ */
