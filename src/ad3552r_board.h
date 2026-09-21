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

#endif /* AD3552R_BOARD_H_ */
