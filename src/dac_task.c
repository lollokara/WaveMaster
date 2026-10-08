#include "dac_task.h"
#include "ad3552r_board.h"
#include "laser_ctrl.h"
#include "engrave_gpio.h"
#include "prr_pwm.h"
#include "atmega_link.h"
#include "calib.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"

static const char *TAG = "dac_task";

#define DAC_TASK_STACK_SIZE 4096
#define DAC_TASK_PRIORITY   10
#define DAC_TASK_CORE       0

/* Measured cost of one single-channel ad3552r_board_write_volts() call, in
 * microseconds, filled in by run_dac_speed_test() at boot. Used by
 * dac_task_us_per_point() so grbl_task can estimate how long a paced burst
 * will really take (see dac_task.h) instead of assuming the commanded
 * per-point delay is the whole cost - it isn't, the SPI transaction
 * dominates at typical preview rates. */
static float s_write_us = 0.0f;

/* RAM budget for batching whole revolutions into one hardware-paced SPI
 * transaction (see run_paced_in_hardware()). Trades memory against how
 * often the inter-transaction hold lands: bigger means a rarer artefact.
 * 16KB is comfortable against this board's ~300KB heap and, for a typical
 * preview shape, already pushes the boundary out to every ~50ms. */
#define PACED_BATCH_MAX_BYTES 16384

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
    s_write_us = (float)(secs * 1e6 / SPEED_TEST_WRITES);
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

/*
 * A paced burst busy-waits (esp_rom_delay_us) for its whole duration and
 * never blocks, which starves core 0's idle task. Because the idle task is
 * a TWDT subscriber (CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0), a burst
 * longer than CONFIG_ESP_TASK_WDT_TIMEOUT_S (5s) would trip the watchdog.
 *
 * The obvious fix - periodically calling vTaskDelay(1) to let idle run -
 * was what this code did, and it was the source of a very visible galvo
 * stutter: CONFIG_FREERTOS_HZ is 100, so one tick is 10ms, not the ~1ms
 * the old comment assumed. On a 100Hz preview loop of a 480-point shape
 * (20us/point) that meant a dead stop roughly twice per revolution, each
 * one about as long as a whole revolution should take. Measured on real
 * hardware: a 19680-point burst that should run 394ms took 2290ms, with
 * 76 x 10ms = 760ms of that spent frozen at a single point.
 *
 * Instead, hand the watchdog duty over for the duration of the burst:
 * unsubscribe core 0's idle task and subscribe this task, which then feeds
 * the watchdog with esp_task_wdt_reset() - a few microseconds, and it
 * never blocks or yields. The burst keeps its real protection (a wedged
 * dac_task still trips the TWDT), and the galvo never stops. Restored on
 * the way out, including the error path.
 *
 * Starving idle on core 0 for the burst is acceptable here: dac_task is
 * the only user task pinned to core 0 (grbl_task and atmega_task are both
 * on core 1, see grbl_task.c/atmega_link.c), no tasks are being deleted
 * during a burst so idle has no cleanup to do, and the busy-wait was
 * already starving it between yields anyway.
 */
static TaskHandle_t s_idle0;

static bool paced_wdt_begin(void)
{
    s_idle0 = xTaskGetIdleTaskHandleForCore(DAC_TASK_CORE);
    if (!s_idle0)
        return false;
    /* Nothing to hand over if idle isn't actually a subscriber (the
     * CHECK_IDLE_TASK_CPU0 option can be off) - leave the watchdog alone
     * and let the caller fall back to yielding. */
    if (esp_task_wdt_status(s_idle0) != ESP_OK)
        return false;
    if (esp_task_wdt_add(NULL) != ESP_OK)
        return false;
    if (esp_task_wdt_delete(s_idle0) != ESP_OK) {
        esp_task_wdt_delete(NULL);
        return false;
    }
    return true;
}

static void paced_wdt_end(bool swapped)
{
    if (!swapped)
        return;
    esp_task_wdt_add(s_idle0);
    esp_task_wdt_delete(NULL);
}

/*
 * Runs a paced stream with the SPI clock doing the pacing, instead of a
 * write-then-busy-wait loop. See ad3552r_board.h: in streaming mode one
 * X+Y point is 12 quad-mode clock cycles and the DAC updates as they
 * arrive, so setting the clock sets the point rate exactly - uniformly, in
 * hardware, and with this task asleep for the whole burst rather than
 * spinning. That is what makes the requested rate actually achievable:
 * software pacing bottoms out at the ~85us/point the per-transaction path
 * costs, so a 100Hz/480-point preview (20.8us/point) could only ever run
 * at about a quarter speed.
 *
 * Returns true if it ran the stream (engaged or errored), false if the
 * requested rate is out of the clock's range and the caller should
 * software-pace instead.
 */
static bool run_paced_in_hardware(const struct laser_cmd *cmd)
{
    float achieved_us = 0.0f;
    uint8_t *rep_buf = NULL, *batch_buf = NULL;
    size_t bytes_per_rep, batch_reps, k;
    uint32_t rep = 0;
    bool forever = (cmd->stream.repeats == 0);
    int32_t err;

    err = ad3552r_board_paced_begin((float)cmd->stream.point_delay_us, &achieved_us);
    if (err <= 0)
        return false; /* out of range, or setup failed - fall back */

    err = ad3552r_board_build_stream_buf(cmd->stream.x_volts, cmd->stream.y_volts,
                                          cmd->stream.n_points, &rep_buf);
    if (err || !rep_buf) {
        ESP_LOGE(TAG, "paced stream: could not build wire buffer: %ld", (long)err);
        ad3552r_board_paced_end();
        return true; /* reported; don't also run it the slow way */
    }

    /* Clocking is gapless *within* a transaction but not across one: each
     * emit costs a bus acquire, a separate instruction/address phase and a
     * release (~33us total, measured in the boot-time streaming profile),
     * and the DAC holds its last value throughout. Once per revolution that
     * is a visible step on whichever axis happens to be on its steep slope
     * at the shape's start point - found on the scope as a notch in Y while
     * X, at its peak and naturally flat there, looked clean.
     *
     * Batching whole revolutions into one transaction divides how often
     * that boundary happens, for one memcpy per revolution of extra RAM.
     * PACED_BATCH_MAX_BYTES is the budget: at a typical 480-point shape
     * (2880 bytes/rev) it buys 5 revolutions per transaction, so the hold
     * lands every ~50ms instead of every ~10ms. */
    bytes_per_rep = cmd->stream.n_points * AD3552R_STREAM_BYTES_PER_POINT;
    batch_reps = PACED_BATCH_MAX_BYTES / bytes_per_rep;
    if (batch_reps < 1)
        batch_reps = 1;
    if (!forever && batch_reps > cmd->stream.repeats)
        batch_reps = cmd->stream.repeats;

    if (batch_reps > 1) {
        batch_buf = malloc(batch_reps * bytes_per_rep);
        if (batch_buf) {
            for (k = 0; k < batch_reps; k++)
                memcpy(batch_buf + k * bytes_per_rep, rep_buf, bytes_per_rep);
        } else {
            /* Not fatal - fall back to one revolution per transaction. */
            batch_reps = 1;
        }
    }

    while (forever || rep < cmd->stream.repeats) {
        const uint8_t *src = batch_buf ? batch_buf : rep_buf;
        size_t this_reps = batch_reps;

        /* Don't overrun a finite request: the batch buffer is just N
         * identical copies, so a short final batch is simply a prefix of
         * it. Without this, asking for 7 revolutions with a batch of 5
         * would emit 10. */
        if (!forever && (size_t)(cmd->stream.repeats - rep) < this_reps)
            this_reps = cmd->stream.repeats - rep;

        err = ad3552r_board_paced_emit(src, cmd->stream.n_points * this_reps);
        if (err) {
            ESP_LOGE(TAG, "paced stream emit failed at rep %lu: %ld",
                     (unsigned long)rep, (long)err);
            break;
        }
        rep += this_reps;
        /* Same rule as the software path: only ever stop between whole
         * repetitions, so an interrupted preview ends on a closed shape. */
        if (laser_ctrl_pending())
            break;
    }

    ad3552r_board_paced_end();
    free(rep_buf);
    free(batch_buf);

    ESP_LOGI(TAG, "hardware-paced run: %u points x %lu reps (%u rev/transaction) "
             "at %.2fus/point (requested %luus) = %.1fHz",
             (unsigned)cmd->stream.n_points, (unsigned long)rep,
             (unsigned)batch_reps, (double)achieved_us,
             (unsigned long)cmd->stream.point_delay_us,
             (double)(achieved_us > 0.0f
                      ? 1.0e6f / (achieved_us * (float)cmd->stream.n_points) : 0.0f));

    return true;
}

uint32_t dac_task_us_per_point(void)
{
    /* Two single-channel writes (X then Y) per streamed point. Rounded up
     * so callers' duration estimates never come out optimistic. */
    return (uint32_t)(2.0f * s_write_us + 0.999f);
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
            if (cmd.stream.paced && run_paced_in_hardware(&cmd)) {
                /* Handled entirely by run_paced_in_hardware() - the SPI
                 * clock did the pacing. */
            } else if (cmd.stream.paced) {
                /* Software-paced fallback, for target rates the clock
                 * divider cannot reach (see ad3552r_board_paced_begin()).
                 * Writes points one at a time via the per-transaction path
                 * with a real busy-wait between them. This is
                 * constant-velocity pacing only - no accel/decel ramp or
                 * junction-deviation planning, see STATUS.md. */
                size_t i;
                /* The two SPI register writes are themselves part of the
                 * period, so the wait is the remainder, not the whole
                 * thing. Sleeping the full period on top of the write cost
                 * ran every software-paced stream slow by that cost -
                 * measured 273us/point against a 208us request before this
                 * subtraction. dac_task_us_per_point() is the boot-measured
                 * figure, so this self-corrects rather than assuming. */
                uint32_t overhead_us = dac_task_us_per_point();
                uint32_t wait_us = cmd.stream.point_delay_us > overhead_us
                    ? cmd.stream.point_delay_us - overhead_us : 0;
                bool stream_err = false;
                /* Replay the same buffer repeats times back-to-back, or
                 * indefinitely when repeats == 0 (M68's preview loop - see
                 * laser_ctrl.h). Looping here rather than having the
                 * producer build the repeated path keeps the shape's heap
                 * cost to one repetition, and leaves no gap between
                 * repetitions for the galvo to park in. */
                uint32_t rep = 0;
                bool forever = (cmd.stream.repeats == 0);
                /* See paced_wdt_begin() above: keeps the task watchdog fed
                 * without ever stalling the galvo. */
                bool wdt_swapped = paced_wdt_begin();

                while (!stream_err && (forever || rep < cmd.stream.repeats)) {
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
                        if (wait_us)
                            esp_rom_delay_us(wait_us);
                        /* Feed the watchdog every 256 points. When the swap
                         * above succeeded this is a non-blocking reset costing
                         * a few microseconds, so the pacing stays smooth; if it
                         * failed we fall back to the old tick yield, which
                         * stutters but at least keeps the watchdog quiet. */
                        if ((i & 0xFFu) == 0xFFu) {
                            if (wdt_swapped)
                                esp_task_wdt_reset();
                            else
                                vTaskDelay(1);
                        }
                    }
                    rep++;
                    /* Only ever break between whole repetitions, so an
                     * interrupted preview still ends on a closed shape
                     * rather than partway round it. Any newly queued
                     * command ends the loop - including the ENGRAVE that
                     * Ctrl-X's soft reset submits (grbl_task.c), which is
                     * what makes an indefinite preview abortable.
                     *
                     * Abort latency is therefore up to one repetition:
                     * ~41ms for a typical preview shape, and bounded at
                     * ~2.7s for the largest/slowest one M68 will build
                     * (PREVIEW_MAX_POINTS at its 1Hz floor). Acceptable
                     * because the marking laser is forced off for the
                     * whole of preview mode - nothing is being cut while
                     * this waits to notice. */
                    if (laser_ctrl_pending())
                        break;
                }
                paced_wdt_end(wdt_swapped);
                if (!stream_err)
                    ESP_LOGI(TAG, "paced run: %u points x %lu reps at ~%luus/point",
                             (unsigned)cmd.stream.n_points,
                             (unsigned long)rep,
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
