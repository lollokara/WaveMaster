#include "atmega_link.h"
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "atmega_link";

/* Wiring/protocol matches ../arduino-laser-control/src/main.cpp exactly:
 * UART0 default pins (GPIO43 TX / GPIO44 RX), 250000 baud 8N1. Free on
 * this board since the PC/GRBL link uses USB-Serial-JTAG instead (see
 * grbl_task.c), not physical UART0. */
#define ATMEGA_UART_PORT UART_NUM_0
#define ATMEGA_BAUD      250000

#define CMD_SET_ARM      0x01
#define CMD_SET_FIRING   0x02
#define CMD_SET_GUIDE    0x03
#define CMD_SET_POWER    0x04
#define CMD_SET_PWM_EMIT 0x05
#define CMD_SET_PWM_SYNC 0x06
#define CMD_GET_STATUS   0x07
#define CMD_PULSE_LATCH  0x08

/*
 * Every command to the ATMEGA is async/fire-and-forget from the caller's
 * point of view: atmega_link_set_guide()/_set_power()/_set_armed() all
 * just enqueue a request and return immediately. A dedicated background
 * task (atmega_task_fn) does the actual UART round-trip, so nothing on
 * this link can ever freeze grbl_task (which owns '?' status queries,
 * reading new G-code lines, and Ctrl-X soft reset - all of which must
 * stay responsive no matter what the ATMEGA is doing).
 *
 * This matters differently for different commands:
 *
 * - ARM (M10): the ATMEGA's set_armed(1) path does digitalWrite() then a
 *   *blocking* delay(2000) before it gets back around to reading serial
 *   and ACKing (see arduino-laser-control/src/main.cpp) - up to ~2.5s.
 *   Fully fire-and-forget is fine here: arming isn't tied to any specific
 *   galvo position.
 *
 * - GUIDE (M62/M63): also fully fire-and-forget - a rare on/off toggle,
 *   not tied to per-pixel timing.
 *
 * - POWER (S-word): async in the same way (grbl_task never blocks on it),
 *   but position/power ordering still has to be enforced somewhere: for
 *   dithered/raster engraving, the galvo must not reach the next position
 *   until the current pixel's power has actually taken effect, or the two
 *   drift out of sync (visible as smearing/ghosting). The DAC path
 *   (~1us/point once streaming) and the power path (a full UART round
 *   trip to a separate microcontroller) run at wildly different speeds.
 *   Rather than block grbl_task (which would refreeze the exact problem
 *   ARM's async fix solved), this link exposes
 *   atmega_link_wait_power_settled(): a semaphore that atmega_task_fn
 *   gives every time it finishes processing a POWER command (success or
 *   timeout - either way, the ATMEGA has stopped changing that value).
 *   dac_task - which has nothing else to be responsive to, unlike
 *   grbl_task - waits on this (bounded, same ACK_TIMEOUT_MS ceiling)
 *   immediately before executing the move that was queued alongside that
 *   S value. See grbl_task.c (gate_on_power) and dac_task.c for the
 *   producer/consumer sides of this.
 *
 * All three command types share one physical UART, so a mutex serializes
 * access - only atmega_task_fn ever talks to the ATMEGA, so in practice
 * this just protects against nothing else stepping on the bus, but keeps
 * send_cmd() safe to call from anywhere.
 */
#define ACK_TIMEOUT_MS       300
#define ARM_ON_ACK_TIMEOUT_MS 2500

#define ATMEGA_TASK_STACK_SIZE 4096
#define ATMEGA_TASK_PRIORITY   5 /* below grbl_task (9) and dac_task (10) */
#define ATMEGA_TASK_CORE       1
#define ATMEGA_QUEUE_LEN       16

enum atmega_cmd_type {
    ATMEGA_CMD_GUIDE,
    ATMEGA_CMD_POWER,
    ATMEGA_CMD_ARM,
};

struct atmega_cmd {
    enum atmega_cmd_type type;
    union {
        bool guide_on;
        float power_percent;
        bool armed;
    };
};

static bool s_ready = false;
static QueueHandle_t s_cmd_queue;
static SemaphoreHandle_t s_uart_mutex;
static SemaphoreHandle_t s_power_settled_sem;

/* Sends a command frame and waits for the single-byte ACK the ATMEGA
 * echoes back (its own command byte on success, 0xFF on NACK/unknown
 * command). Only ever called from atmega_task_fn in practice, but takes
 * s_uart_mutex itself so that isn't a hard requirement. */
static bool send_cmd(const uint8_t *frame, size_t len, uint32_t timeout_ms)
{
    uint8_t resp;
    int n;
    bool ok;

    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(timeout_ms + 50)) != pdTRUE) {
        ESP_LOGW(TAG, "UART busy, dropping cmd 0x%02x", frame[0]);
        return false;
    }

    uart_flush_input(ATMEGA_UART_PORT);
    uart_write_bytes(ATMEGA_UART_PORT, (const char *)frame, len);

    n = uart_read_bytes(ATMEGA_UART_PORT, &resp, 1, pdMS_TO_TICKS(timeout_ms));
    if (n != 1) {
        ESP_LOGW(TAG, "no ACK for cmd 0x%02x within %lums", frame[0],
                 (unsigned long)timeout_ms);
        ok = false;
    } else if (resp == 0xFF) {
        ESP_LOGW(TAG, "NACK for cmd 0x%02x", frame[0]);
        ok = false;
    } else if (resp != frame[0]) {
        ESP_LOGW(TAG, "unexpected ACK 0x%02x for cmd 0x%02x", resp, frame[0]);
        ok = false;
    } else {
        ok = true;
    }

    xSemaphoreGive(s_uart_mutex);
    return ok;
}

static void do_set_guide(bool on)
{
    uint8_t frame[2] = { CMD_SET_GUIDE, on ? 1 : 0 };

    send_cmd(frame, sizeof(frame), ACK_TIMEOUT_MS);
    ESP_LOGI(TAG, "guide laser %s", on ? "ON" : "off");
}

static void do_set_power(float percent)
{
    uint8_t value;
    uint8_t frame[2];
    bool ok;

    if (percent < 0.0f)
        percent = 0.0f;
    if (percent > 100.0f)
        percent = 100.0f;
    value = (uint8_t)(percent / 100.0f * 255.0f + 0.5f);

    frame[0] = CMD_SET_POWER;
    frame[1] = value;
    ok = send_cmd(frame, sizeof(frame), ACK_TIMEOUT_MS);
    ESP_LOGI(TAG, "power = %.1f%% (byte 0x%02x) %s", (double)percent, value,
             ok ? "confirmed" : "NOT confirmed");

    /* Given (not taken) regardless of ok: either the ATMEGA confirmed the
     * new value, or the timeout window has already elapsed - either way,
     * it has stopped changing, which is all a waiting dac_task needs to
     * know. A binary semaphore that's already "given" just stays given -
     * fine here since exactly one give() happens per POWER command and
     * dac_task's wait (see atmega_link_wait_power_settled) takes exactly
     * once per gated move, kept 1:1 by grbl_task always enqueuing them as
     * a matching pair (see grbl_task.c's have_s handling). */
    xSemaphoreGive(s_power_settled_sem);
}

static void do_set_armed(bool armed)
{
    uint8_t frame[2] = { CMD_SET_ARM, armed ? 1 : 0 };

    /* This is the call that can take up to ~2.5s (see the header comment) -
     * safe here since this only ever runs on atmega_task_fn's own stack,
     * never on grbl_task's. */
    send_cmd(frame, sizeof(frame), armed ? ARM_ON_ACK_TIMEOUT_MS : ACK_TIMEOUT_MS);
    ESP_LOGI(TAG, "system %s", armed ? "ARMED" : "disarmed");
}

static void atmega_task_fn(void *arg)
{
    struct atmega_cmd cmd;

    for (;;) {
        if (xQueueReceive(s_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE)
            continue;

        switch (cmd.type) {
        case ATMEGA_CMD_GUIDE:
            do_set_guide(cmd.guide_on);
            break;
        case ATMEGA_CMD_POWER:
            do_set_power(cmd.power_percent);
            break;
        case ATMEGA_CMD_ARM:
            do_set_armed(cmd.armed);
            break;
        }
    }
}

void atmega_link_init(void)
{
    uart_config_t cfg = {
        .baud_rate = ATMEGA_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_driver_install(ATMEGA_UART_PORT, 256, 256, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return;
    }
    if (uart_param_config(ATMEGA_UART_PORT, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed");
        return;
    }
    /* TX/RX swapped from UART0's IOMUX defaults (GPIO43=TX/GPIO44=RX).
     * History: this swap was tried once before, appeared to make things
     * worse once the level shifter had power, so it was reverted back to
     * the (presumed) physically-correct default assignment - but ACK
     * timeouts persisted even after fixing the level-shifter power issue,
     * and scope probing showed no traffic at all on the expected line
     * with the default assignment. Re-trying the swap to see whether the
     * physical wiring is in fact crossed relative to what "default"
     * assumes. If this doesn't fix it either, the fault is elsewhere
     * (framing/baud, not pin direction). */
    if (uart_set_pin(ATMEGA_UART_PORT, 44, 43, UART_PIN_NO_CHANGE,
                      UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed");
        return;
    }

    s_uart_mutex = xSemaphoreCreateMutex();
    s_power_settled_sem = xSemaphoreCreateBinary();
    s_cmd_queue = xQueueCreate(ATMEGA_QUEUE_LEN, sizeof(struct atmega_cmd));
    if (!s_uart_mutex || !s_power_settled_sem || !s_cmd_queue) {
        ESP_LOGE(TAG, "mutex/semaphore/queue creation failed");
        return;
    }

    if (xTaskCreatePinnedToCore(atmega_task_fn, "atmega_task", ATMEGA_TASK_STACK_SIZE,
                                 NULL, ATMEGA_TASK_PRIORITY, NULL, ATMEGA_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        return;
    }

    s_ready = true;
    ESP_LOGI(TAG, "ATMEGA link up on UART%d default pins @ %d baud "
             "(fully async on core %d, power-settle event for dac_task)",
             ATMEGA_UART_PORT, ATMEGA_BAUD, ATMEGA_TASK_CORE);
}

void atmega_link_set_guide(bool on)
{
    struct atmega_cmd cmd = { .type = ATMEGA_CMD_GUIDE, .guide_on = on };

    if (!s_ready || xQueueSend(s_cmd_queue, &cmd, 0) != pdTRUE)
        ESP_LOGW(TAG, "guide command dropped (not ready or queue full)");
}

void atmega_link_set_power(float percent)
{
    struct atmega_cmd cmd = { .type = ATMEGA_CMD_POWER, .power_percent = percent };

    if (!s_ready || xQueueSend(s_cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "power command dropped (not ready or queue full)");
        /* Nobody will ever give the semaphore for this dropped request -
         * give it ourselves so a dac_task waiting on it doesn't stall for
         * the full timeout for no reason. */
        if (s_power_settled_sem)
            xSemaphoreGive(s_power_settled_sem);
    }
}

bool atmega_link_wait_power_settled(uint32_t timeout_ms)
{
    if (!s_ready)
        return false;

    return xSemaphoreTake(s_power_settled_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void atmega_link_set_armed(bool armed)
{
    struct atmega_cmd cmd = { .type = ATMEGA_CMD_ARM, .armed = armed };

    if (!s_ready || xQueueSend(s_cmd_queue, &cmd, 0) != pdTRUE)
        ESP_LOGW(TAG, "arm command dropped (not ready or queue full)");
}
