#ifndef AD3552R_BOARD_H_
#define AD3552R_BOARD_H_

#include <stdbool.h>
#include <stddef.h>
#include "ad3552r.h"

/* Board pin map, from the WaveMaster schematic */
#define BOARD_PIN_D3   10
#define BOARD_PIN_D2   11
#define BOARD_PIN_D1   12
#define BOARD_PIN_D0   13
#define BOARD_PIN_DIR   8
#define BOARD_PIN_RST   9
/* Wired to the AD3552R's QSPI pin (pin 31): hardware mode-select input,
 * NOT settable by register alone. Low = classic/dual SPI, high = quad SPI
 * (datasheet Table 14: quad mode is "enabled by the QSPI pin only"). Must
 * be driven low for single-lane bring-up before any SPI activity. */
#define BOARD_PIN_SPI_OP1 7
#define BOARD_PIN_SCK   5
#define BOARD_PIN_CS    6

extern struct ad3552r_desc *g_dac;

struct ad3552r_desc *ad3552r_board_init(void);
void ad3552r_board_set_quad(struct ad3552r_desc *dac, bool enable);

/* Write a target output voltage to a DAC channel (0 or 1), converting via
 * the channel's configured scale/offset (see ad3552r_get_scale/_get_offset).
 * Returns the ad3552r driver's int32_t error code (0 on success). */
int32_t ad3552r_board_write_volts(uint8_t ch, float volts);

/* Exposes the same volts->code conversion write_volts() uses internally,
 * for verifying other write paths (e.g. dac_task's streaming speed test)
 * against a register readback without duplicating the scale/offset math. */
int32_t ad3552r_board_volts_to_code(uint8_t ch, float volts, uint16_t *out_code);

/* Buffers a whole precomputed X/Y path (n_points volts pairs, one SPI
 * burst for the entire buffer instead of one transaction per point) and
 * clocks it out via the AD3552R's STREAM_MODE register. See the
 * implementation comment in ad3552r_board.c for how the addressing works.
 * Returns the ad3552r driver's int32_t error code (0 on success). */
int32_t ad3552r_board_stream_xy(const float *x_volts, const float *y_volts,
                                 size_t n_points);

/* Restores normal (non-streaming) single-instruction mode after a
 * streaming session. Only needs calling if a streaming write has
 * happened at least once; harmless (returns 0 immediately) otherwise. */
int32_t ad3552r_board_stream_end(void);

/*
 * Hardware-paced streaming.
 *
 * The per-transaction write path costs ~85us/point (measured: 32.3us per
 * single-channel register write, two per point), so software pacing - a
 * write followed by a busy-wait - simply cannot hit a fast target rate; a
 * 100Hz/480-point preview needs 20.8us/point and gets ~85us instead.
 *
 * These three calls get there by pacing in hardware instead. In the
 * AD3552R's streaming mode the address pointer loops every 6 bytes, so one
 * X+Y point is 6 bytes = 12 quad-mode clock cycles, and the DAC updates its
 * outputs as those bytes arrive. The SPI clock therefore *is* the point
 * rate: pick the clock and the hardware emits points at exactly that
 * spacing, perfectly uniformly, with the CPU asleep for the whole burst
 * (transfers this long go through the interrupt-driven SPI path - see
 * no_os_spi_esp32.c's IRQ_XFER_MIN_BYTES).
 *
 * Usage: build the wire buffer once, begin, emit it as many times as
 * wanted, end.
 */

/* Wire-format size of one X+Y point in streaming mode: CH1's 3-byte
 * register then CH0's. Exposed so callers can size/replicate stream
 * buffers. */
#define AD3552R_STREAM_BYTES_PER_POINT 6

/* Converts n_points of volts into the 6-bytes-per-point wire format the
 * emit call expects. Caller frees *out_buf. */
int32_t ad3552r_board_build_stream_buf(const float *x_volts, const float *y_volts,
                                        size_t n_points, uint8_t **out_buf);

/* Switches the bus to quad + streaming mode and sets the clock so points
 * come out target_us_per_point apart. Writes back the rate actually
 * achievable (the clock quantizes to an integer divider) to
 * out_us_per_point.
 *
 * Returns 1 if hardware pacing was engaged, 0 if the requested rate is
 * outside what the clock divider can reach (the caller should fall back to
 * software pacing), or a negative ad3552r/errno code on failure. */
int32_t ad3552r_board_paced_begin(float target_us_per_point, float *out_us_per_point);

/* Clocks one pass of a buffer built by ad3552r_board_build_stream_buf().
 * Only valid between begin and end. */
int32_t ad3552r_board_paced_emit(const uint8_t *buf, size_t n_points);

/* Restores the normal clock, single-lane mode and non-streaming mode. */
void ad3552r_board_paced_end(void);

#endif /* AD3552R_BOARD_H_ */
