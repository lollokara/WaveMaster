#include "ad3552r_board.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_delay.h"
#include "no_os_util.h"
#include "esp_log.h"
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>

static const char *TAG = "ad3552r_board";

struct ad3552r_desc *g_dac = NULL;

static struct ad3552r_esp32_spi_pins s_pins = {
    .sck = BOARD_PIN_SCK,
    .cs = BOARD_PIN_CS,
    .d0 = BOARD_PIN_D0,
    .d1 = BOARD_PIN_D1,
    .d2 = BOARD_PIN_D2,
    .d3 = BOARD_PIN_D3,
    .dir = BOARD_PIN_DIR,
};

static struct no_os_gpio_init_param s_reset_gpio_param = {
    .number = BOARD_PIN_RST,
};

/* Kept alive (not a local var) so streaming can toggle it dynamically -
 * see set_qspi_pin() below. */
static struct no_os_gpio_desc *s_qspi_pin = NULL;

static void hold_qspi_pin_low(void)
{
    struct no_os_gpio_init_param qspi_pin_param = { .number = BOARD_PIN_SPI_OP1 };

    /* AD3552R's QSPI pin is a hardware mode-select input, not settable by
     * register: low selects classic/dual SPI, high selects quad SPI.
     * Must be driven low before any SPI activity for single-lane bring-up. */
    no_os_gpio_get_optional(&s_qspi_pin, &qspi_pin_param);
    if (s_qspi_pin)
        no_os_gpio_direction_output(s_qspi_pin, NO_OS_GPIO_LOW);
}

/* AD3552R's SPI protocol is phase-sequential even within one transfer
 * (instruction/address phase always host-driven, then a data phase
 * that's host-driven for writes or purely device-driven for reads -
 * never simultaneous), which is why quad-SDR mode's data phase timing
 * works the same as this board's other lines: this call only needs to
 * happen once before the whole streamed burst, not per phase. */
static void set_qspi_pin(bool high)
{
    if (s_qspi_pin)
        no_os_gpio_set_value(s_qspi_pin, high ? NO_OS_GPIO_HIGH : NO_OS_GPIO_LOW);
}

struct ad3552r_desc *ad3552r_board_init(void)
{
    static struct ad3552r_init_param param;
    struct ad3552r_desc *dac;
    int32_t err;

    hold_qspi_pin_low();

    param = (struct ad3552r_init_param){
        .chip_id = AD3552R_ID,
        .spi_param = {
            .device_id = 0,
            .chip_select = 0,
            .mode = NO_OS_SPI_MODE_0,
            /* Datasheet max fSCLK is 66MHz; 60MHz confirmed clean on real
             * hardware (init + scratchpad/product-ID checks pass, and
             * streamed quad-SPI writes verified correct via register
             * readback - see the stream path). */
            .max_speed_hz = 60000000,
            .bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST,
            .extra = &s_pins,
        },
        .reset_gpio_param_optional = &s_reset_gpio_param,
        .sdo_drive_strength = 1,
        .channels = {
            [0] = { .en = 1, .range = AD3552R_CH_OUTPUT_RANGE_NEG_10__10V },
            [1] = { .en = 1, .range = AD3552R_CH_OUTPUT_RANGE_NEG_10__10V },
        },
        /* CRC framing requires simultaneous tx+rx SPI transactions, which
         * this board's shared-direction level-shifter (single DIR line for
         * D0-D3) cannot reliably support - confirmed empirically: raw
         * half-duplex (phase-separated) register reads work perfectly,
         * but genuine full-duplex CRC frames do not. Disabled for now;
         * revisit if data-integrity checking on the wire becomes a
         * requirement (galvo control at these speeds/distances is low risk). */
        .crc_en = 0,
        .single_transfer = 0,
    };

    err = ad3552r_init(&dac, &param);
    if (err) {
        ESP_LOGE(TAG, "ad3552r_init failed: %ld", (long)err);
        return NULL;
    }

    ESP_LOGI(TAG, "AD3552R init OK (single-lane, CRC disabled)");
    g_dac = dac;

    {
        int32_t si, sd, oi, od;
        ad3552r_get_scale(dac, 0, &si, &sd);
        ad3552r_get_offset(dac, 0, &oi, &od);
        ESP_LOGI(TAG, "CH0 scale=%ld.%06ld offset=%ld.%06ld",
                 (long)si, (long)sd, (long)oi, (long)od);
    }

    return dac;
}

static int32_t volts_to_code(uint8_t ch, float volts, uint16_t *out_code)
{
    int32_t err, scale_int, scale_dec, offset_int, offset_dec;
    float scale_mv, offset_code, code_f;

    err = ad3552r_get_scale(g_dac, ch, &scale_int, &scale_dec);
    if (!err)
        err = ad3552r_get_offset(g_dac, ch, &offset_int, &offset_dec);
    if (err)
        return err;

    /* ad3552r_get_scale/_get_offset follow the Linux IIO convention this
     * ported driver was written against: scale is in millivolts-per-code,
     * and offset is in the *code* domain (added before scaling), i.e.
     * Vout(mV) = (code + offset) * scale - NOT Vout = code*scale + offset.
     * Confirmed on real hardware: for the +-10V range, offset reads back
     * as exactly -32768 (= Vmin(mV) / scale = -10000 / 0.305176), which
     * only makes sense in code-domain units, not millivolts. Inverting
     * for code = f(volts): code = Vout(mV)/scale - offset.
     *
     * Deliberately float, not double: ESP32-S3 has a hardware FPU for
     * single precision only, so double arithmetic here is done in slow
     * software emulation. Measured impact on the buffered-streaming
     * write path (5000 points, 10000 calls to this function): building
     * the point buffer with double math took ~118ms (~11.8us/call);
     * switching to float removed it as the dominant cost - see
     * STATUS.md's Performance section. float's ~7 significant digits is
     * far more precision than a 16-bit DAC code needs anyway. */
    scale_mv = scale_int + scale_dec / 1000000.0f;
    offset_code = offset_int + offset_dec / 1000000.0f;
    code_f = (volts * 1000.0f) / scale_mv - offset_code;
    if (code_f < 0)
        code_f = 0;
    if (code_f > 65535)
        code_f = 65535;
    *out_code = (uint16_t)(code_f + 0.5f);

    return 0;
}

int32_t ad3552r_board_volts_to_code(uint8_t ch, float volts, uint16_t *out_code)
{
    return volts_to_code(ch, volts, out_code);
}

int32_t ad3552r_board_write_volts(uint8_t ch, float volts)
{
    int32_t err;
    uint16_t code;

    err = volts_to_code(ch, volts, &code);
    if (err)
        return err;

    /* NOT ad3552r_set_ch_value(g_dac, AD3552R_CH_CODE, ch, code) - found,
     * the hard way, to be a silent no-op on this (non-Xilinx) platform.
     * The vendored driver's ad3552r_set_ch_value()'s AD3552R_CH_CODE case
     * is entirely gated behind "#ifdef XILINX_PLATFORM" (ad3552r.c), which
     * is never defined for this ESP-IDF build; with no code outside that
     * #ifdef and no break statement, the switch falls through into the
     * AD3552R_CH_RFB case instead, which just stores the value as the
     * feedback-resistor software field and recalculates gain/offset -
     * *not* a DAC write at all. It returns 0 (success), does no SPI
     * transaction, and doesn't touch the DAC's output register - which is
     * why every plain move/paced-stream point ever written through this
     * function appeared to succeed (correct computed voltage logged, no
     * error) while the physical output never changed. Confirmed via the
     * "$RB" debug readback (see grbl_task.c/dac_task.c): CH0/CH1 stayed
     * frozen at whatever the boot-time streaming self-test last wrote
     * (which uses a completely different path - ad3552r_transfer(), not
     * ad3552r_set_ch_value() - and was never affected by this bug) no
     * matter what position was actually commanded afterward.
     *
     * Fix: write the DAC output register directly, bypassing the broken
     * abstraction entirely - the exact same register (and function)
     * ad3552r_board_read_code() reads back from, and the same one
     * the stream path's raw ad3552r_transfer() calls already
     * targeted correctly all along. */
    return ad3552r_write_reg(g_dac, AD3552R_REG_ADDR_CH_DAC_24B(ch), code);
}

/*
 * Streaming session (used by dac_task.c). See ad3552r_board.h.
 *
 * The AD3552R's STREAM_MODE length auto-resets after the first streaming
 * transaction, so the config must be applied immediately before the stream
 * and with nothing else in between: the config is attached to an
 * ad3552r_transfer() carrying the write instruction, and the shim is armed
 * to send only that instruction (CS kept low, bus left acquired) instead of
 * the data message.
 */
#define STREAM_BYTES_PER_POINT AD3552R_STREAM_BYTES_PER_POINT

static bool s_stream_mode_configured = false;
static uint32_t s_stream_saved_clock_hz = 0;
static uint8_t s_stream_marker[1];

int32_t ad3552r_board_stream_open(uint32_t clock_hz)
{
    struct ad3552_transfer_config stream_cfg = {0};
    struct ad3552_transfer_data xfer = {0};
    int32_t err;

    s_stream_saved_clock_hz = g_dac->spi->clock_hz;
    err = ad3552r_esp32_spi_set_clock(g_dac->spi, clock_hz);
    if (err)
        return err;

    set_qspi_pin(true);
    ad3552r_esp32_spi_set_lines(g_dac->spi, 4);

    stream_cfg.single_instr = 0;
    stream_cfg.stream_mode_length = STREAM_BYTES_PER_POINT;
    stream_cfg.addr_asc = 0;
    xfer.spi_cfg = &stream_cfg;
    xfer.addr = AD3552R_REG_ADDR_CH_DAC_24B(1); /* CH1, the higher address */
    xfer.data = s_stream_marker;
    xfer.len = 1;
    xfer.is_read = 0;
    s_stream_mode_configured = true;

    ad3552r_esp32_spi_stream_arm(s_stream_marker);
    err = ad3552r_transfer(g_dac, &xfer);
    ad3552r_esp32_spi_stream_arm(NULL);
    if (err) {
        ad3552r_esp32_spi_set_lines(g_dac->spi, 1);
        set_qspi_pin(false);
        ad3552r_esp32_spi_set_clock(g_dac->spi, s_stream_saved_clock_hz);
        ad3552r_board_stream_end();
    }
    return err;
}

void ad3552r_board_stream_close(void)
{
    ad3552r_esp32_spi_stream_release(g_dac->spi);
    ad3552r_esp32_spi_set_lines(g_dac->spi, 1);
    set_qspi_pin(false);
    ad3552r_esp32_spi_set_clock(g_dac->spi, s_stream_saved_clock_hz);
    ad3552r_board_stream_end();
}

void *ad3552r_board_spi_dev(void)
{
    return ad3552r_esp32_spi_dev(g_dac->spi);
}

uint32_t ad3552r_board_spi_actual_hz(void)
{
    return ad3552r_esp32_spi_actual_hz(g_dac->spi);
}

uint32_t ad3552r_board_quantize_clock(uint32_t hz)
{
    return ad3552r_esp32_spi_quantize_hz(hz);
}

int32_t ad3552r_board_code_map(uint8_t ch, float *inv_scale_mv, float *offset_code)
{
    int32_t si, sd, oi, od, err;

    err = ad3552r_get_scale(g_dac, ch, &si, &sd);
    if (!err)
        err = ad3552r_get_offset(g_dac, ch, &oi, &od);
    if (err)
        return err;
    *inv_scale_mv = 1.0f / (si + sd / 1000000.0f);
    *offset_code = oi + od / 1000000.0f;
    return 0;
}

/* Restores normal (non-streaming) single-instruction mode after a
 * streaming session, so subsequent one-off register reads/writes (e.g.
 * status polling, CLI-style debugging) behave as expected again. */
int32_t ad3552r_board_stream_end(void)
{
    struct ad3552_transfer_config normal_cfg = {0};
    struct ad3552_transfer_data xfer = {0};
    uint8_t dummy[1] = {0};
    int32_t err;

    if (!s_stream_mode_configured)
        return 0;

    normal_cfg.single_instr = 1;
    normal_cfg.stream_mode_length = 0;
    normal_cfg.addr_asc = 0;

    /* ad3552r_read_reg()/_write_reg() never attach a spi_cfg to their
     * internal transfer struct, so they can't be used to apply this -
     * only a manually built ad3552_transfer_data with .spi_cfg set
     * reaches _update_spi_cfg(). A 1-byte scratchpad read is a harmless
     * real transaction to carry the config change on. */
    xfer.addr = AD3552R_REG_ADDR_SCRATCH_PAD;
    xfer.data = dummy;
    xfer.len = 1;
    xfer.is_read = 1;
    xfer.spi_cfg = &normal_cfg;

    err = ad3552r_transfer(g_dac, &xfer);
    s_stream_mode_configured = false;

    return err;
}

void ad3552r_board_set_quad(struct ad3552r_desc *dac, bool enable)
{
    int32_t err;
    uint16_t mode = enable ? AD3552R_QUAD_SPI : AD3552R_SPI;
    uint16_t val = mode << 6; /* AD3552R_MASK_MULTI_IO_MODE = BIT(7)|BIT(6) */

    /* Register-side switch happens over whatever line-mode is currently
     * active, so do it before flipping our own bus's line count. */
    err = ad3552r_write_reg(dac, AD3552R_REG_ADDR_TRANSFER_REGISTER, val);
    if (err) {
        ESP_LOGE(TAG, "quad mode register write failed: %ld", (long)err);
        return;
    }

    ad3552r_esp32_spi_set_lines(dac->spi, enable ? 4 : 1);
    ESP_LOGI(TAG, "SPI bus now in %s mode", enable ? "QUAD" : "single-lane");
}
