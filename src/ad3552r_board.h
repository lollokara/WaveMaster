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

/* Restores normal (non-streaming) single-instruction mode after a
 * streaming session. Harmless if no stream was configured. */
int32_t ad3552r_board_stream_end(void);

/* Wire-format size of one X+Y point in streaming mode: CH1's 3-byte
 * register then CH0's. In quad mode that is 12 SPI clocks, so the SPI clock
 * (12 / tick) sets the point rate. */
#define AD3552R_STREAM_BYTES_PER_POINT 6

/* Opens a streaming session: swaps the SPI device to clock_hz, raises the
 * QSPI pin, switches to 4 lines, applies the streaming register config and
 * sends the write instruction (addressed to CH1) with CS kept active. The
 * bus stays acquired; the caller queues data-only QIO transactions on
 * ad3552r_board_spi_dev() (all but the last with SPI_TRANS_CS_KEEP_ACTIVE)
 * and, after the last, non-keep-active one has completed, calls
 * ad3552r_board_stream_close(). */
int32_t ad3552r_board_stream_open(uint32_t clock_hz);
void ad3552r_board_stream_close(void);

/* Analog/interface config guard (see ad3552r_board.c): snapshot after init,
 * checked and restored before every stream session. */
void ad3552r_board_guard_snapshot(void);
uint32_t ad3552r_board_guard_repairs(void);

/* Diagnostics: reads the guarded registers plus INTERFACE_CONFIG_B,
 * STREAM_MODE and TRANSFER_REGISTER. Returns the count (addrs points to a
 * static table) or a negative error. Only while the stream is closed. */
int32_t ad3552r_board_dump_regs(const uint8_t **addrs, uint16_t *vals, size_t max);

/* spi_device_handle_t of the current device (valid after stream_open). */
void *ad3552r_board_spi_dev(void);
/* Clock the device really runs at (Hz), 0 if unknown. */
uint32_t ad3552r_board_spi_actual_hz(void);
/* Clock the SPI peripheral would produce for a request (Hz). */
uint32_t ad3552r_board_quantize_clock(uint32_t hz);

/* Per-channel constants for the fast volts->code conversion
 * code = volts*1000*inv_scale_mv - offset_code (as the driver defines it). */
int32_t ad3552r_board_code_map(uint8_t ch, float *inv_scale_mv, float *offset_code);

#endif /* AD3552R_BOARD_H_ */
