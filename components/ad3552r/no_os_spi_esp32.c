#include "no_os_spi.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "no_os_spi_esp32";

/* Shared by every transaction; set right before each spi_device_transmit
 * call since transfers in this driver are always synchronous/single-threaded. */
static int s_dir_gpio = -1;
static volatile int s_host_drives = 1;

static void IRAM_ATTR pre_transfer_cb(spi_transaction_t *t)
{
    if (s_dir_gpio >= 0)
        gpio_set_level((gpio_num_t)s_dir_gpio, s_host_drives ? 1 : 0);
}

int32_t no_os_spi_init(struct no_os_spi_desc **desc,
                        const struct no_os_spi_init_param *param)
{
    struct ad3552r_esp32_spi_pins *pins = (struct ad3552r_esp32_spi_pins *)param->extra;
    struct no_os_spi_desc *d;
    esp_err_t err;

    d = calloc(1, sizeof(*d));
    if (!d)
        return -ENOMEM;

    d->init = *param;
    d->active_lines = 1;
    d->dir_gpio = pins->dir;

    gpio_config_t dir_cfg = {
        .pin_bit_mask = 1ULL << pins->dir,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&dir_cfg);
    gpio_set_level((gpio_num_t)pins->dir, 1); /* default: host drives */

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = pins->d0,
        .miso_io_num = pins->d1,
        .sclk_io_num = pins->sck,
        .quadwp_io_num = pins->d2,
        .quadhd_io_num = pins->d3,
        /* Large enough for buffered continuous/streaming writes (see
         * ad3552r_board_stream_xy()), which pack a whole precomputed path
         * into one SPI burst instead of one transaction per point. */
        .max_transfer_sz = 32768,
    };
    err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        free(d);
        return -EIO;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = (int)param->max_speed_hz,
        .mode = (uint8_t)param->mode,
        .spics_io_num = pins->cs,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .pre_cb = pre_transfer_cb,
    };
    ESP_LOGI(TAG, "requesting clock_speed_hz=%d", dev_cfg.clock_speed_hz);
    err = spi_bus_add_device(SPI2_HOST, &dev_cfg, (spi_device_handle_t *)&d->spi_dev);
    if (err != ESP_OK) {
        free(d);
        return -EIO;
    }

    /* A second (full-duplex) device on this bus was tried here to support
     * CRC-framed simultaneous tx+rx transfers, but merely having a second
     * device configured measurably broke the primary half-duplex device's
     * transfers (confirmed on hardware: reads that worked stopped working
     * as soon as the second device existed, even unused). Since CRC is
     * disabled (see ad3552r_board.c), it isn't needed - removed. */
    d->spi_dev_fd = NULL;

    s_dir_gpio = pins->dir;
    *desc = d;

    return 0;
}

int32_t no_os_spi_remove(struct no_os_spi_desc *desc)
{
    if (desc->spi_dev_fd)
        spi_bus_remove_device((spi_device_handle_t)desc->spi_dev_fd);
    if (desc->spi_dev)
        spi_bus_remove_device((spi_device_handle_t)desc->spi_dev);
    free(desc);
    return 0;
}

void ad3552r_esp32_spi_set_lines(struct no_os_spi_desc *desc, uint8_t lines)
{
    desc->active_lines = lines;
}

static uint32_t line_flags(uint8_t lines)
{
    if (lines == 4)
        return SPI_TRANS_MODE_QIO;
    if (lines == 2)
        return SPI_TRANS_MODE_DIO;
    return 0;
}

/*
 * Polling transmit instead of spi_device_transmit(): the latter queues the
 * transaction and blocks on a FreeRTOS semaphore signaled by the SPI ISR,
 * which for transfers this short (a handful of bytes) costs far more than
 * the SCLK bit-time itself. Polling busy-waits the CPU instead of
 * sleeping, trading a spinning core for a much shorter, more
 * deterministic per-transaction latency - the right tradeoff here since
 * dac_task has nothing else useful to do while a write is in flight
 * anyway. Measured effect on real hardware: see dac_task.c's boot-time
 * speed test result in the boot log.
 */
static int32_t do_transaction(struct no_os_spi_desc *desc, const uint8_t *tx,
                               uint8_t *rx, uint32_t len, uint8_t keep_cs)
{
    spi_transaction_t t = {0};
    spi_device_handle_t dev;
    esp_err_t err;

    dev = (spi_device_handle_t)desc->spi_dev;
    t.flags = line_flags(desc->active_lines);
    if (tx) {
        /* Half-duplex TX phase: only .length (bits) is used. */
        t.length = len * 8;
        t.tx_buffer = tx;
        s_host_drives = 1;
    } else if (rx) {
        /* Half-duplex RX phase: length must be 0 and rxlength set
         * instead, otherwise ESP-IDF rejects the transaction with
         * ESP_ERR_INVALID_ARG. */
        t.rxlength = len * 8;
        t.rx_buffer = rx;
        s_host_drives = 0;
    }
    if (keep_cs)
        t.flags |= SPI_TRANS_CS_KEEP_ACTIVE;

    err = spi_device_polling_transmit(dev, &t);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "spi_device_polling_transmit failed: %s (len=%lu, flags=0x%lx)",
                 esp_err_to_name(err), (unsigned long)len, (unsigned long)t.flags);
    /* Always leave DIR back in host-drive state once the bus is idle,
     * so instruction phases of the *next* transfer always start correctly. */
    gpio_set_level((gpio_num_t)desc->dir_gpio, 1);

    return err == ESP_OK ? 0 : -EIO;
}

/*
 * Fast path: when every message in the array is tx-only (true for every
 * register *write* in single-instruction mode - the instruction byte and
 * the data bytes are both host-driven the whole time), concatenate them
 * into one buffer and issue a single polling transaction instead of one
 * transaction per message. This also skips spi_device_acquire_bus()
 * entirely, since CS_KEEP_ACTIVE across separate transactions is no
 * longer needed when it's just one transaction. Reads still need the
 * slower multi-transaction path below (the data phase's direction differs
 * from the instruction phase's).
 */
#define COALESCE_BUF_MAX 8

static bool try_coalesced_write(struct no_os_spi_desc *desc,
                                 struct no_os_spi_msg *msgs, uint32_t len,
                                 int32_t *out_err)
{
    uint8_t buf[COALESCE_BUF_MAX];
    uint32_t total = 0, i;
    static bool logged = false;

    for (i = 0; i < len; i++) {
        if (msgs[i].rx_buff || !msgs[i].tx_buff)
            return false;
        if (total + msgs[i].bytes_number > COALESCE_BUF_MAX)
            return false;
        memcpy(buf + total, msgs[i].tx_buff, msgs[i].bytes_number);
        total += msgs[i].bytes_number;
    }

    if (!logged) {
        ESP_LOGI(TAG, "coalesced-write fast path active (%lu msgs, %lu bytes)",
                 (unsigned long)len, (unsigned long)total);
        logged = true;
    }

    *out_err = do_transaction(desc, buf, NULL, total, 0);
    return true;
}

/*
 * Stage-timestamp profiling for large (streaming) transfers, to find
 * exactly where time goes for a bulk write - see STATUS.md's "Performance"
 * section for the ~10x gap this was added to investigate. Only triggers
 * when a message is bigger than PROF_THRESHOLD bytes (i.e. the streaming
 * path, not per-point single-register writes), and only logs once so it
 * doesn't spam every subsequent streaming call.
 */
#define PROF_THRESHOLD 256
#define PROF_MAX_STAGES 8

static void log_profile(const int64_t *ts, const char **labels, int n, uint32_t bytes)
{
    int i;

    ESP_LOGI(TAG, "--- streaming transfer profile (%lu bytes) ---", (unsigned long)bytes);
    for (i = 1; i < n; i++)
        ESP_LOGI(TAG, "  %-28s: %6lld us", labels[i], (long long)(ts[i] - ts[i - 1]));
    ESP_LOGI(TAG, "  %-28s: %6lld us", "TOTAL", (long long)(ts[n - 1] - ts[0]));
}

int32_t no_os_spi_transfer(struct no_os_spi_desc *desc,
                            struct no_os_spi_msg *msgs, uint32_t len)
{
    uint32_t i;
    int32_t err;
    bool coalesced;
    bool profile = false;
    static bool profile_logged = false;
    int64_t ts[PROF_MAX_STAGES];
    const char *labels[PROF_MAX_STAGES];
    int n = 0;
    uint32_t total_bytes = 0;

    for (i = 0; i < len; i++)
        total_bytes += msgs[i].bytes_number;
    if (total_bytes > PROF_THRESHOLD && !profile_logged) {
        profile = true;
        profile_logged = true;
    }

    if (profile) {
        ts[n] = esp_timer_get_time(); labels[n] = "entry"; n++;
    }

    coalesced = try_coalesced_write(desc, msgs, len, &err);
    if (coalesced) {
        if (profile) {
            ts[n] = esp_timer_get_time(); labels[n] = "coalesced path (unexpected)"; n++;
            log_profile(ts, labels, n, total_bytes);
        }
        return err;
    }

    if (profile) {
        ts[n] = esp_timer_get_time(); labels[n] = "after coalesce-check (rejected, as expected)"; n++;
    }

    /* Mixed tx/rx array (a register read): SPI_TRANS_CS_KEEP_ACTIVE (used
     * between messages below to hold CS across the instruction and data
     * phases) is only accepted by the ESP-IDF driver while the bus has
     * been explicitly acquired for this device - otherwise every such
     * transaction is rejected outright with ESP_ERR_INVALID_ARG before it
     * ever reaches the wire. */
    err = spi_device_acquire_bus((spi_device_handle_t)desc->spi_dev, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_acquire_bus failed: %s", esp_err_to_name(err));
        return -EIO;
    }

    if (profile) {
        ts[n] = esp_timer_get_time(); labels[n] = "after spi_device_acquire_bus"; n++;
    }

    for (i = 0; i < len; i++) {
        struct no_os_spi_msg *m = &msgs[i];
        uint8_t keep_cs = !m->cs_change;

        err = do_transaction(desc, m->tx_buff, m->rx_buff, m->bytes_number, keep_cs);
        if (profile && n < PROF_MAX_STAGES - 1) {
            ts[n] = esp_timer_get_time();
            labels[n] = (m->bytes_number > 64) ? "after do_transaction (bulk data)"
                                                : "after do_transaction (small msg)";
            n++;
        }
        if (err)
            break;
    }

    spi_device_release_bus((spi_device_handle_t)desc->spi_dev);

    if (profile) {
        ts[n] = esp_timer_get_time(); labels[n] = "after spi_device_release_bus"; n++;
        log_profile(ts, labels, n, total_bytes);
    }

    return err;
}

int32_t ad3552r_esp32_spi_raw(struct no_os_spi_desc *desc, const uint8_t *tx,
                               uint8_t *rx, uint32_t len, uint8_t host_drives)
{
    return do_transaction(desc, host_drives ? tx : NULL,
                           host_drives ? NULL : rx, len, 0);
}
