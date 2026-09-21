#include "dac_task.h"
#include "ad3552r_board.h"
#include "laser_ctrl.h"
#include "engrave_gpio.h"
#include "prr_pwm.h"
#include "atmega_link.h"
#include "calib.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char *TAG = "dac_task";

#define DAC_TASK_STACK_SIZE 4096
#define DAC_TASK_PRIORITY   10
#define DAC_TASK_CORE       0

/* One-shot speed tests run at boot, measuring real DAC write throughput
 * over the actual SPI bus/driver path (not a theoretical bit-rate
 * calculation) - reported as MUPS (mega updates per second) to match the
 * datasheet's own throughput unit. See STATUS.md for the full writeup of
 * what was tried and why per-transaction writes can't approach the
 * datasheet's 33 MUPS figure (that number is for the AD3552R's own
 * quad+DDR streaming hardware mode, not reachable via ESP-IDF's generic
 * spi_master driver for arbitrary transfers). */
#define SPEED_TEST_WRITES 5000
#define SPEED_TEST_STREAM_POINTS 5000

static void run_dac_speed_test(void)
{
    int64_t t0, t1;
    double secs, ups, mups;
    int32_t err;
    uint32_t i;
    float *x, *y;

    /* Per-transaction single-channel writes: one SPI burst per point. */
    t0 = esp_timer_get_time();
    for (i = 0; i < SPEED_TEST_WRITES; i++) {
        err = ad3552r_board_write_volts(0, (i & 1) ? 5.0f : -5.0f);
        if (err) {
            ESP_LOGE(TAG, "speed test write failed at i=%lu: %ld",
                     (unsigned long)i, (long)err);
            return;
        }
    }
    t1 = esp_timer_get_time();

    secs = (double)(t1 - t0) / 1e6;
    ups = SPEED_TEST_WRITES / secs;
    mups = ups / 1e6;
    ESP_LOGI(TAG, "per-transaction speed test: %d single-channel writes in %.4f s "
             "= %.1f updates/sec = %.5f MUPS (single-lane SPI, no CRC - see ad3552r_board.c for clock rate)",
             SPEED_TEST_WRITES, secs, ups, mups);

    /* Buffered streaming: one SPI burst for the whole precomputed path,
     * both X and Y channels per point (see ad3552r_board_stream_xy()). */
    x = malloc(SPEED_TEST_STREAM_POINTS * sizeof(float));
    y = malloc(SPEED_TEST_STREAM_POINTS * sizeof(float));
    if (!x || !y) {
        ESP_LOGE(TAG, "speed test: out of memory for stream buffers");
        free(x);
        free(y);
        return;
    }
    for (i = 0; i < SPEED_TEST_STREAM_POINTS; i++) {
        x[i] = (i & 1) ? 5.0f : -5.0f;
        y[i] = (i & 1) ? -5.0f : 5.0f;
    }

    /* First call: the one-time diagnostic logging inside
     * ad3552r_board_stream_xy()/no_os_spi_transfer() (buffer-build timing,
     * the stage-profile dump) fires during this call and is itself several
     * hundred microseconds of USB-CDC log output per line, which would
     * otherwise contaminate the timing below. Run it once, unmeasured,
     * to get that one-time cost out of the way first. */
    err = ad3552r_board_stream_xy(x, y, SPEED_TEST_STREAM_POINTS);
    if (err) {
        ESP_LOGE(TAG, "streaming speed test (warmup) failed: %ld", (long)err);
        free(x);
        free(y);
        return;
    }

    /* Second call: identical work, but now the one-time diagnostic logs
     * are done firing (guarded by static flags), so this measures the
     * real steady-state cost. */
    t0 = esp_timer_get_time();
    err = ad3552r_board_stream_xy(x, y, SPEED_TEST_STREAM_POINTS);
    t1 = esp_timer_get_time();
    ad3552r_board_stream_end();
    free(x);
    free(y);

    if (err) {
        ESP_LOGE(TAG, "streaming speed test failed: %ld", (long)err);
        return;
    }

    secs = (double)(t1 - t0) / 1e6;
    ups = SPEED_TEST_STREAM_POINTS / secs;
    mups = ups / 1e6;
    ESP_LOGI(TAG, "buffered streaming speed test (steady-state, warmup excluded): "
             "%d X+Y points in %.4f s = %.1f points/sec = %.5f MUPS "
             "(quad SPI, no CRC - see ad3552r_board.c for clock rate)",
             SPEED_TEST_STREAM_POINTS, secs, ups, mups);

    /* Correctness check: the streamed burst should leave the DAC output
     * registers holding the *last* point's codes. Read them back and
     * compare against what write_volts() (the already-verified
     * per-transaction path) computes for the same voltages, to catch a
     * wrong stream address/order bug rather than just trusting "no SPI
     * error" as proof the values landed correctly. */
    {
        uint16_t ch0_reg = 0, ch1_reg = 0, ch0_expect = 0, ch1_expect = 0;
        int32_t e1, e2, e3, e4;
        float last_x = (SPEED_TEST_STREAM_POINTS - 1) & 1 ? 5.0f : -5.0f;
        float last_y = (SPEED_TEST_STREAM_POINTS - 1) & 1 ? -5.0f : 5.0f;

        e1 = ad3552r_read_reg(g_dac, AD3552R_REG_ADDR_CH_DAC_24B(0), &ch0_reg);
        e2 = ad3552r_read_reg(g_dac, AD3552R_REG_ADDR_CH_DAC_24B(1), &ch1_reg);
        e3 = ad3552r_board_volts_to_code(0, last_x, &ch0_expect);
        e4 = ad3552r_board_volts_to_code(1, last_y, &ch1_expect);

        if (e1 || e2 || e3 || e4)
            ESP_LOGE(TAG, "stream verify: a read/convert call failed (e1=%ld e2=%ld e3=%ld e4=%ld)",
                     (long)e1, (long)e2, (long)e3, (long)e4);
        else if (ch0_reg == ch0_expect && ch1_reg == ch1_expect)
            ESP_LOGI(TAG, "stream verify OK: CH0=0x%04x CH1=0x%04x matches last point (%.1fV,%.1fV)",
                     ch0_reg, ch1_reg, (double)last_x, (double)last_y);
        else
            ESP_LOGE(TAG, "stream verify MISMATCH: CH0=0x%04x (want 0x%04x) CH1=0x%04x (want 0x%04x)",
                     ch0_reg, ch0_expect, ch1_reg, ch1_expect);
    }
}

static void dac_task_fn(void *arg)
{
    struct laser_cmd cmd;
    uint32_t moves = 0;
    int32_t err;
    /* Mirrors grbl_task's engrave state, tracked independently here so
     * this task knows whether to apply jump_delay or mark/polygon_delay
     * after a MOVE without needing a round trip back to grbl_task. Always
     * kept in sync by the LASER_CMD_ENGRAVE case below, which is always
     * queued (and thus always processed) before any MOVE/STREAM that
     * depends on the new state, since both go through the same FIFO. */
    bool engrave_on = false;

    ESP_LOGI(TAG, "DAC task running on core %d", xPortGetCoreID());

    run_dac_speed_test();

    for (;;) {
        if (!laser_ctrl_next(&cmd, 1000))
            continue;

        switch (cmd.type) {
        case LASER_CMD_MOVE:
            if (cmd.gate_on_power) {
                /* This line also carried an S-word; grbl_task fired the
                 * power change async and moved on (see grbl_task.c /
                 * atmega_link.h). This is the one place that actually
                 * waits, matching the ATMEGA link's own ACK timeout (see
                 * atmega_link.c) so a slow/missing ATMEGA delays only
                 * this pixel's DAC write, never the serial line reader. */
                if (!atmega_link_wait_power_settled(300))
                    ESP_LOGW(TAG, "power not confirmed before this move - "
                             "position/power may be out of sync for this pixel");
            }
            err = ad3552r_board_write_volts(0, cmd.move.x_volts);
            if (err)
                ESP_LOGE(TAG, "X write failed: %ld", (long)err);
            err = ad3552r_board_write_volts(1, cmd.move.y_volts);
            if (err)
                ESP_LOGE(TAG, "Y write failed: %ld", (long)err);
            moves++;
            if (moves % 100 == 0)
                ESP_LOGI(TAG, "moves=%lu last=(%.3fV,%.3fV)",
                         (unsigned long)moves, cmd.move.x_volts, cmd.move.y_volts);
            /* Settle time before the next queued command executes: jump
             * (non-marking) moves get jump_delay; marking moves get
             * mark_delay plus polygon_delay (applied uniformly per vertex,
             * not angle-aware - see calib.h). This task has nothing else
             * to stay responsive to, so a plain busy-wait is fine, same
             * reasoning as the ATMEGA power-settle wait above. */
            {
                uint32_t delay_us = engrave_on
                    ? (uint32_t)(calib_mark_delay_us() + calib_polygon_delay_us())
                    : (uint32_t)calib_jump_delay_us();
                if (delay_us)
                    esp_rom_delay_us(delay_us);
            }
            break;
        case LASER_CMD_ENGRAVE:
            engrave_gpio_set(cmd.engrave_on);
            engrave_on = cmd.engrave_on;
            ESP_LOGI(TAG, "engrave %s", cmd.engrave_on ? "ON" : "off");
            /* Give the laser time to physically respond before the next
             * queued command (typically the first point of a mark, or the
             * next jump) executes. */
            {
                uint32_t delay_us = (uint32_t)(cmd.engrave_on
                    ? calib_laser_on_delay_us() : calib_laser_off_delay_us());
                if (delay_us)
                    esp_rom_delay_us(delay_us);
            }
            break;
        case LASER_CMD_PRR:
            prr_pwm_set_hz(cmd.prr_hz);
            ESP_LOGI(TAG, "prr = %.1f Hz", cmd.prr_hz);
            break;
        case LASER_CMD_STREAM:
            if (cmd.stream.paced) {
                /* Feed-rate-paced marking run (mark_stroke() in
                 * grbl_task.c, when the G-code line carried an F word):
                 * write points one at a time via the per-transaction path
                 * with a real delay between them, trading throughput for
                 * hitting the requested speed - the opposite trade-off
                 * from the fast quad-SPI burst path below, deliberately,
                 * since arcs/hatch fill have no feed target and want max
                 * speed while a feed-paced run's whole point is to be
                 * slower and controlled. This is constant-velocity pacing
                 * only - no accel/decel ramp or junction-deviation
                 * planning, see STATUS.md. */
                size_t i;
                bool stream_err = false;

                for (i = 0; i < cmd.stream.n_points; i++) {
                    err = ad3552r_board_write_volts(0, cmd.stream.x_volts[i]);
                    if (!err)
                        err = ad3552r_board_write_volts(1, cmd.stream.y_volts[i]);
                    if (err) {
                        ESP_LOGE(TAG, "paced stream write failed at point %u: %ld",
                                 (unsigned)i, (long)err);
                        stream_err = true;
                        break;
                    }
                    if (cmd.stream.point_delay_us)
                        esp_rom_delay_us(cmd.stream.point_delay_us);
                    /* esp_rom_delay_us() is a pure busy-wait - never yields
                     * to the scheduler. A long enough paced burst (found on
                     * real hardware: ~20000 points, ~3s total) can starve
                     * the idle task long enough to trip its watchdog
                     * (task_wdt panic on IDLE0), since this task never
                     * blocks in between. A real (tick-based) 1ms yield
                     * every 256 points lets the idle task run and pat the
                     * watchdog, at the cost of a small amount of pacing
                     * jitter - negligible for what this path is used for
                     * (feed-rate/preview pacing, not marking-quality
                     * timing critical enough to matter at the ~4us level). */
                    if ((i & 0xFFu) == 0xFFu)
                        vTaskDelay(1);
                }
                if (!stream_err)
                    ESP_LOGI(TAG, "paced run: %u points at ~%luus/point",
                             (unsigned)cmd.stream.n_points,
                             (unsigned long)cmd.stream.point_delay_us);
            } else {
                err = ad3552r_board_stream_xy(cmd.stream.x_volts, cmd.stream.y_volts,
                                              cmd.stream.n_points);
                ad3552r_board_stream_end();
                if (err)
                    ESP_LOGE(TAG, "stream write failed: %ld", (long)err);
                else
                    ESP_LOGI(TAG, "streamed %u points", (unsigned)cmd.stream.n_points);
            }
            free(cmd.stream.x_volts);
            free(cmd.stream.y_volts);
            break;
        }
    }
}

bool dac_task_start(void)
{
    if (!ad3552r_board_init())
        return false;

    engrave_gpio_init();
    prr_pwm_init();

    return xTaskCreatePinnedToCore(dac_task_fn, "dac_task", DAC_TASK_STACK_SIZE,
                                    NULL, DAC_TASK_PRIORITY, NULL,
                                    DAC_TASK_CORE) == pdPASS;
}
