#ifndef NO_OS_SPI_H_
#define NO_OS_SPI_H_

#include <stdint.h>

enum no_os_spi_mode {
    NO_OS_SPI_MODE_0,
    NO_OS_SPI_MODE_1,
    NO_OS_SPI_MODE_2,
    NO_OS_SPI_MODE_3,
};

enum no_os_spi_bit_order {
    NO_OS_SPI_BIT_ORDER_MSB_FIRST,
    NO_OS_SPI_BIT_ORDER_LSB_FIRST,
};

/* ESP32-specific pin map, passed as init_param.extra */
struct ad3552r_esp32_spi_pins {
    int sck;
    int cs;
    int d0; /* SDI/SDIO0, MOSI in single-lane mode */
    int d1; /* SDO/SDIO1, MISO in single-lane mode */
    int d2; /* SDIO2, quad mode only */
    int d3; /* SDIO3, quad mode only */
    int dir; /* level-shifter direction pin, shared by d0-d3 */
};

struct no_os_spi_init_param {
    uint32_t device_id;
    uint32_t max_speed_hz;
    enum no_os_spi_mode mode;
    uint8_t chip_select;
    enum no_os_spi_bit_order bit_order;
    void *platform_ops; /* unused by this shim, kept for source compatibility */
    void *extra;        /* struct ad3552r_esp32_spi_pins * */
};

struct no_os_spi_desc {
    struct no_os_spi_init_param init;
    void *spi_dev;      /* spi_device_handle_t, opaque here to avoid pulling
                            in ESP-IDF headers for driver-only consumers */
    void *spi_dev_fd;   /* second spi_device_handle_t on the same bus,
                            configured full-duplex (not HALFDUPLEX), used
                            only when a transaction needs simultaneous
                            tx+rx (e.g. CRC-framed transfers) */
    int dir_gpio;
    uint8_t active_lines; /* 1, 2, or 4 */
    uint32_t clock_hz;    /* clock the device is currently configured for */
    struct ad3552r_esp32_spi_pins pins; /* kept so the device can be re-added
                                          at a different clock - see
                                          ad3552r_esp32_spi_set_clock() */
};

struct no_os_spi_msg {
    uint8_t *tx_buff;
    uint8_t *rx_buff;
    uint32_t bytes_number;
    uint8_t cs_change; /* 1 = release CS after this message, 0 = hold */
};

int32_t no_os_spi_init(struct no_os_spi_desc **desc,
                        const struct no_os_spi_init_param *param);
int32_t no_os_spi_remove(struct no_os_spi_desc *desc);
int32_t no_os_spi_transfer(struct no_os_spi_desc *desc,
                            struct no_os_spi_msg *msgs, uint32_t len);

/* ESP32-specific extensions, not part of the no-OS API surface */
void ad3552r_esp32_spi_set_lines(struct no_os_spi_desc *desc, uint8_t lines);
int32_t ad3552r_esp32_spi_raw(struct no_os_spi_desc *desc, const uint8_t *tx,
                               uint8_t *rx, uint32_t len, uint8_t host_drives);

/*
 * Reconfigure the bus clock by removing and re-adding the SPI device (the
 * ESP-IDF master driver fixes the clock per device, and offers no
 * per-transaction override).
 *
 * Exists because in the AD3552R's streaming mode the SPI clock *is* the
 * output pacing: the device's address pointer loops every 6 bytes, so one
 * point costs 12 quad-mode clock cycles and the DAC updates as the bytes
 * arrive. Choosing the clock therefore sets the point rate exactly, in
 * hardware, with no per-point CPU involvement - see
 * ad3552r_board_paced_begin().
 *
 * Only a single device is ever configured on this bus at a time: adding a
 * second one was previously found to break the first one's transfers even
 * while unused (see no_os_spi_init()), so this swaps rather than adds.
 * Returns 0 on success.
 */
int32_t ad3552r_esp32_spi_set_clock(struct no_os_spi_desc *desc, uint32_t hz);

/* The clock the hardware actually settled on for the current device, which
 * is the requested value rounded to an achievable divider. Callers pacing
 * off the clock must use this, not what they asked for. Returns 0 if it
 * cannot be determined. */
uint32_t ad3552r_esp32_spi_actual_hz(struct no_os_spi_desc *desc);

/* ---- Streaming support (galvo output stage) ---------------------------
 *
 * Callbacks run in the SPI interrupt, right before a queued transaction
 * whose spi_transaction_t.user is non-NULL is started (pre) and right after
 * it completes (post). Must be IRAM-safe. Transactions issued by this shim
 * itself always have user == NULL. */
typedef void (*ad3552r_spi_stream_cb_t)(void *user);
void ad3552r_esp32_spi_set_stream_cbs(ad3552r_spi_stream_cb_t pre,
                                       ad3552r_spi_stream_cb_t post);

/* The clock the SPI peripheral will really use for a requested rate. */
uint32_t ad3552r_esp32_spi_quantize_hz(uint32_t hz);

/* Current spi_device_handle_t (changes whenever set_clock() swaps it). */
void *ad3552r_esp32_spi_dev(struct no_os_spi_desc *desc);

/* The next no_os_spi_transfer() of two messages whose second message's tx
 * pointer equals marker sends only the first message (CS kept active) and
 * returns with the bus acquired. Pair with ad3552r_esp32_spi_stream_release()
 * after the last (non keep-active) transaction has completed. */
void ad3552r_esp32_spi_stream_arm(const uint8_t *marker);
void ad3552r_esp32_spi_stream_release(struct no_os_spi_desc *desc);

#endif /* NO_OS_SPI_H_ */
