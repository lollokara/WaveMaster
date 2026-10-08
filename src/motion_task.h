#ifndef MOTION_TASK_H_
#define MOTION_TASK_H_

/*
 * Core-1 task that owns the trajectory generator (motion.h).
 *
 *   grbl.c  --motion_submit()-->  [segment queue]  --> motion_task
 *       --> motion_gen (ticks) --> galvo_out_tick() --> [chunk ring] --> dac_task (core 0)
 *
 * Flow control: motion_submit() blocks while the segment queue is full, so
 * grbl.c withholds "ok" and the host's character-counting stream pauses -
 * that is the whole backpressure chain, end to end.
 *
 * Underrun policy (the host cannot always keep up with a fast galvo):
 *  - Start-up prebuffer: when motion begins from idle, the task does not
 *    start generating until the queue holds MOTION_PREBUFFER_SEGS segments
 *    or no new segment has arrived for MOTION_PREBUFFER_IDLE_MS.
 *  - While a polyline is open (motion_gen_has_pending()) and the queue is
 *    empty, it waits for the next segment only as long as the output ring
 *    still has more than MOTION_UNDERRUN_MARGIN_US of motion buffered
 *    (galvo_out_buffered_us()). If the ring would run dry first, it calls
 *    motion_gen_finish() - a clean polyline end (laser off, mark delay) -
 *    instead of letting the galvo stop with the laser on. The next segment
 *    then starts a new polyline (with a fresh laser-on delay).
 *  - When the queue is empty and nothing is pending, it calls
 *    galvo_out_flush() so a partially filled chunk is streamed promptly.
 */

#include <stdbool.h>
#include <stddef.h>
#include "motion.h"

#define MOTION_QUEUE_LEN           512
#define MOTION_PREBUFFER_SEGS      64
#define MOTION_PREBUFFER_IDLE_MS   20
#define MOTION_UNDERRUN_MARGIN_US  3000.0f

/* Creates the queue and task (core 1). Call after calib_init() and
 * dac_task_start(). Initial position is (0,0) machine mm. */
bool motion_task_start(void);

/* Enqueue a segment. Blocks up to wait_ms while the queue is full (pass
 * UINT32_MAX to wait forever). Returns false on timeout or while an abort is
 * in progress. Called from grbl.c only. */
bool motion_submit(const struct motion_seg *seg, uint32_t wait_ms);

/* Abort: empties the queue, resets the generator, calls galvo_out_abort()
 * (laser off immediately, pending output dropped). Blocks until the motion
 * task has acknowledged (bounded, ~200 ms). On return *x and *y hold the
 * position the galvo was actually left at (machine mm). */
void motion_abort(float *x, float *y);

/* True while anything is queued, being generated, or still streaming. */
bool motion_busy(void);

/* Free slots in the segment queue (for the status report's Bf: field). */
size_t motion_queue_free(void);

/* Re-read motion parameters from calib.h (call after a $ setting change). */
void motion_reload_params(void);

#endif /* MOTION_TASK_H_ */
