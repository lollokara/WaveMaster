#ifndef DAC_TASK_H_
#define DAC_TASK_H_

/*
 * Core-0 owner of the SPI bus / AD3552R. Consumes chunks produced by
 * galvo_out.c and clocks them out as one continuous hardware-paced stream
 * (SPI clock = point rate, CS held low across chunks), driving the laser
 * gate in lock-step. Also services sync barriers (power / PRR changes,
 * arm-and-wait) between chunks.
 *
 * The chunk structures and the producer/consumer queues are private to
 * galvo_out.c + dac_task.c (see galvo_chunk.h).
 */

#include <stdbool.h>
#include <stdint.h>

/* Initialises the AD3552R and starts the task pinned to core 0. */
bool dac_task_start(void);

/* Diagnostic: read back the two DAC output registers. Only valid while the
 * stream is closed (idle); returns false otherwise or on SPI error. Runs on
 * the caller's task but serialises with dac_task internally. */
bool dac_task_readback(uint16_t *code_x, uint16_t *code_y);

#endif /* DAC_TASK_H_ */
