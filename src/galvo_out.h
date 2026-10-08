#ifndef GALVO_OUT_H_
#define GALVO_OUT_H_

/*
 * Output stage: tick stream -> DAC codes + laser gate, packed into chunks
 * for dac_task (core 0) to clock out with hardware pacing.
 *
 * Producer side (core 1, motion_task only - single producer):
 *   galvo_out_tick()   one sample per tick: machine-mm position + intended
 *                      laser power (0 = off). Converts mm -> DAC codes
 *                      (calib: origin, swap/invert, radial correction,
 *                      volts/mm, offset, clamp to +-10 V), applies the
 *                      laser-on / laser-off delays to the gate edges, and
 *                      implements the power mode:
 *                        $223=0 "analog": power != 0 -> gate on; the power
 *                          word goes to the ATmega. A change between two
 *                          non-zero powers is a sync barrier: dac_task
 *                          lets the stream drain (laser off), applies the
 *                          new power over the ATmega link, then resumes.
 *                        $223=1 "pulse density": ATmega power fixed at
 *                          $225; power p gates the laser on for
 *                          round(p/255 * $229) ticks out of every $229.
 *                      Blocks when the chunk pool is full (backpressure).
 *   galvo_out_set_prr() queue a pulse-repetition-rate change (barrier).
 *   galvo_out_flush()   hand the partially filled chunk to dac_task now.
 *
 * Any thread:
 *   galvo_out_abort()   gate off immediately, drop everything not yet on
 *                       the wire, stream stops. Safe from core 1 tasks.
 *   galvo_out_hold()    feed hold: stop starting new chunks (gate off at
 *                       the next chunk boundary), resume with false.
 *
 * Gate timing: the gate GPIO is switched from the SPI transaction ISR at
 * chunk start and by a hardware timer for edges inside a chunk, both timed
 * from the same moment the chunk starts clocking out, so laser edges stay
 * locked to the DAC samples (see galvo_out.c / dac_task.c).
 */

#include <stdbool.h>
#include <stdint.h>

/* Call once after dac_task_start(). Reads the tick period from calib and
 * programs the SPI clock for it. */
void galvo_out_init(void);

/* Actual tick period in microseconds after SPI clock quantisation. The
 * trajectory generator must use THIS value, not $200, so speeds are exact. */
float galvo_out_tick_us(void);

/* Re-apply settings that affect the output stage (tick period, delays,
 * transform, power mode). Only call while idle (!galvo_out_busy()). */
void galvo_out_reload(void);

void galvo_out_tick(float x_mm, float y_mm, uint8_t power);
void galvo_out_set_prr(float hz);
void galvo_out_flush(void);

/* Microseconds of motion committed but not yet clocked out (ready chunks +
 * in-flight transactions + the partially filled chunk). */
float galvo_out_buffered_us(void);

/* True while any chunk is filling, queued or in flight. */
bool galvo_out_busy(void);

void galvo_out_abort(void);
void galvo_out_hold(bool hold);
bool galvo_out_is_held(void);

/* Machine-mm position of the last tick actually clocked out to the DAC
 * (for status reports). */
void galvo_out_position(float *x_mm, float *y_mm);

/* Diagnostics for '$S' style status: underrun count, chunks streamed,
 * barrier count. */
struct galvo_out_stats {
    uint32_t chunks;
    uint32_t underruns;
    uint32_t barriers;
    uint64_t ticks;
};
void galvo_out_get_stats(struct galvo_out_stats *s);

#endif /* GALVO_OUT_H_ */
