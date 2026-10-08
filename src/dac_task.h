#ifndef DAC_TASK_H_
#define DAC_TASK_H_

#include <stdbool.h>
#include <stdint.h>

/* Starts the DAC/SPI task pinned to core 0. Must be called after
 * laser_ctrl_init(). Returns true if the AD3552R initialized and the task
 * was created successfully. */
bool dac_task_start(void);

/* Fixed cost of emitting one point of a paced stream (the two X/Y SPI
 * register writes), in microseconds, on top of whatever per-point delay
 * the caller asks for. Derived from the per-transaction throughput
 * actually measured at boot rather than assumed, and rounded up.
 *
 * Callers that tell a host how long a burst will take must add this to
 * their commanded delay: at typical preview rates it dominates. For the
 * M68 preview loop's 20us/point it is roughly 3x the commanded delay, so
 * ignoring it made grbl_task under-report a 2.3s burst as "~0s" - which in
 * turn made tools/square_loop.py re-issue M68 while the previous 160KB
 * point buffer was still allocated, and the second allocation failed
 * ("M68: out of memory"), dropping whole revolutions of the preview.
 *
 * Returns 0 until the boot speed test has run. */
uint32_t dac_task_us_per_point(void);

#endif /* DAC_TASK_H_ */
