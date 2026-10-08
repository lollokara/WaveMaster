/*
 * ESP32 side of the ATmega328P companion link (protocol v2, see
 * ArduinoCompanion/README.md).
 *
 * Design:
 *  - UART0, TX GPIO44 / RX GPIO43 (swapped vs. the IOMUX defaults), 250000
 *    baud 8N1 through a level shifter.
 *  - One link task (core 1, prio 5) is the only user of the UART. The public
 *    setters only update the "desired" state under a spinlock and notify the
 *    task, so they never block and may be called from any core.
 *  - The task is level-triggered: it compares the desired state with the last
 *    status reply and sends whatever differs (disarm > arm > power > guide),
 *    otherwise polls GET_STATUS every 200 ms (this is also the ATmega's 1 s
 *    heartbeat). A SET_POWER therefore goes out immediately after the notify;
 *    the reply carries the latched power word and is the confirmation that
 *    atmega_link_wait_power() waits for (about 0.8 ms at 250 kbaud).
 *  - Arming is never re-issued implicitly: if the ATmega reports disarmed
 *    although the caller still wants it armed (heartbeat trip, ATmega reset)
 *    the desired state is dropped to disarmed / power 0 instead of silently
 *    re-arming the laser.
 *  - Waiters block on an event group that the task sets after every valid
 *    reply or link-state change; they return false quickly while the link is
 *    down. Designed for one waiter at a time (dac_task).
 */
#include "atmega_link.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

static const char *TAG = "atmega_link";

#define ATMEGA_UART_PORT   UART_NUM_0
#define ATMEGA_PIN_TX      44
#define ATMEGA_PIN_RX      43
#define ATMEGA_BAUD        250000

#define LINK_TASK_STACK    3072
#define LINK_TASK_PRIO     5
#define LINK_TASK_CORE     1

#define POLL_PERIOD_US       200000   /* heartbeat / status poll */
#define POLL_FAST_PERIOD_US   50000   /* while armed and waiting for ready */
#define REPLY_TIMEOUT_US      20000
#define TXN_ATTEMPTS          3
#define LINK_OK_WINDOW_US    500000
#define FAIL_BACKOFF_MS       20

#define FRAME_SOF        0xA5
#define MAX_PAYLOAD      8
#define CMD_SET_ARM      0x01
#define CMD_SET_GUIDE    0x02
#define CMD_SET_POWER    0x03
#define CMD_GET_STATUS   0x04
#define CMD_ACK_FLAG     0x80
#define CMD_NAK          0xFF
#define PROTOCOL_VERSION 2

#define FLAG_ARMED       (1 << 0)
#define FLAG_READY       (1 << 1)
#define FLAG_GUIDE       (1 << 2)
#define FLAG_HB_TRIPPED  (1 << 3)
#define FLAG_5V_PRESENT  (1 << 4)

#define EVT_CHANGED      (1 << 0)

enum action { ACT_NONE, ACT_ARM0, ACT_ARM1, ACT_POWER, ACT_GUIDE, ACT_POLL };

struct reply_status {
    uint8_t flags, power, stat, version, err;
    uint16_t mv;
};

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_task;
static EventGroupHandle_t s_events;

/* Everything below is protected by s_lock. */
static bool s_des_armed, s_des_guide, s_arm_pending;
static uint8_t s_des_power;
static bool s_have_status;
static int64_t s_last_ok_us;
static struct reply_status s_rs;
static uint32_t s_errors;

/* Link-task private. */
static bool s_prev_link_ok;

static uint8_t crc8_update(uint8_t crc, uint8_t b)
{
    crc ^= b;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    return crc;
}

static bool link_ok_locked(int64_t now_us)
{
    return s_have_status && (now_us - s_last_ok_us) < LINK_OK_WINDOW_US;
}

static void count_error(void)
{
    portENTER_CRITICAL(&s_lock);
    s_errors++;
    portEXIT_CRITICAL(&s_lock);
}

static TickType_t us_to_ticks_ceil(int64_t us)
{
    if (us <= 0)
        return 0;
    int64_t ms = (us + 999) / 1000;
    TickType_t t = pdMS_TO_TICKS(ms);
    return t ? t : 1;
}

/* Read exactly n bytes before the deadline. */
static bool read_exact(uint8_t *buf, size_t n, int64_t deadline_us)
{
    size_t got = 0;
    while (got < n) {
        int64_t left = deadline_us - esp_timer_get_time();
        if (left <= 0)
            return false;
        int r = uart_read_bytes(ATMEGA_UART_PORT, buf + got, n - got,
                                us_to_ticks_ceil(left));
        if (r > 0)
            got += (size_t)r;
        else if (r < 0)
            return false;
    }
    return true;
}

/* One request/reply exchange. Returns true on a valid ACK for cmd. */
static bool txn_once(uint8_t cmd, const uint8_t *pl, uint8_t len,
                     struct reply_status *out)
{
    uint8_t tx[4 + MAX_PAYLOAD];
    uint8_t crc = 0;

    tx[0] = FRAME_SOF;
    tx[1] = cmd;
    tx[2] = len;
    if (len)
        memcpy(&tx[3], pl, len);
    for (int i = 1; i < 3 + len; i++)
        crc = crc8_update(crc, tx[i]);
    tx[3 + len] = crc;

    uart_flush_input(ATMEGA_UART_PORT);
    if (uart_write_bytes(ATMEGA_UART_PORT, tx, 4 + len) < 0)
        return false;

    int64_t deadline = esp_timer_get_time() + REPLY_TIMEOUT_US;

    for (;;) {
        uint8_t b, hdr[2], body[MAX_PAYLOAD + 1];

        /* Hunt for SOF. */
        do {
            if (!read_exact(&b, 1, deadline))
                return false;
        } while (b != FRAME_SOF);

        if (!read_exact(hdr, 2, deadline))
            return false;
        if (hdr[1] > MAX_PAYLOAD)
            continue; /* garbage, resync */
        if (!read_exact(body, (size_t)hdr[1] + 1, deadline))
            return false;

        crc = crc8_update(crc8_update(0, hdr[0]), hdr[1]);
        for (int i = 0; i < hdr[1]; i++)
            crc = crc8_update(crc, body[i]);
        if (crc != body[hdr[1]])
            return false; /* retry */

        if (hdr[0] == CMD_NAK) {
            ESP_LOGW(TAG, "NAK for cmd 0x%02x, err %u", cmd,
                     hdr[1] ? body[0] : 0);
            return false;
        }
        if (hdr[0] != (uint8_t)(cmd | CMD_ACK_FLAG) || hdr[1] != 8)
            continue; /* stale reply to an earlier request */

        out->flags = body[0];
        out->power = body[1];
        out->stat = body[2];
        out->mv = (uint16_t)((body[3] << 8) | body[4]);
        out->version = body[5];
        out->err = body[6];
        return true;
    }
}

static bool txn(uint8_t cmd, const uint8_t *pl, uint8_t len,
                struct reply_status *out)
{
    for (int i = 0; i < TXN_ATTEMPTS; i++) {
        if (txn_once(cmd, pl, len, out))
            return true;
        count_error();
    }
    return false;
}

static void apply_status(const struct reply_status *rs)
{
    static bool version_warned;

    portENTER_CRITICAL(&s_lock);
    s_rs = *rs;
    s_have_status = true;
    s_last_ok_us = esp_timer_get_time();
    if ((rs->flags & FLAG_ARMED) && s_des_armed)
        s_arm_pending = false;
    portEXIT_CRITICAL(&s_lock);

    if (rs->flags & FLAG_HB_TRIPPED)
        ESP_LOGW(TAG, "ATmega heartbeat timeout tripped: it disarmed and zeroed power");
    if (rs->version != PROTOCOL_VERSION && !version_warned) {
        version_warned = true;
        ESP_LOGW(TAG, "ATmega protocol version %u, expected %u", rs->version,
                 PROTOCOL_VERSION);
    }
    xEventGroupSetBits(s_events, EVT_CHANGED);
}

static void check_link_transition(void)
{
    bool ok;

    portENTER_CRITICAL(&s_lock);
    ok = link_ok_locked(esp_timer_get_time());
    portEXIT_CRITICAL(&s_lock);

    if (ok != s_prev_link_ok) {
        s_prev_link_ok = ok;
        if (ok)
            ESP_LOGW(TAG, "link UP");
        else
            ESP_LOGW(TAG, "link LOST (no valid reply for %d ms)",
                     LINK_OK_WINDOW_US / 1000);
        xEventGroupSetBits(s_events, EVT_CHANGED);
    }
}

static enum action decide(int64_t now, int64_t next_poll)
{
    enum action act = ACT_NONE;

    portENTER_CRITICAL(&s_lock);
    if (!link_ok_locked(now)) {
        if (now >= next_poll)
            act = ACT_POLL;
    } else {
        bool r_armed = s_rs.flags & FLAG_ARMED;
        bool r_guide = s_rs.flags & FLAG_GUIDE;

        if (r_armed && !s_des_armed)
            act = ACT_ARM0;
        else if (s_des_armed && !r_armed && s_arm_pending)
            act = ACT_ARM1;
        else if (s_rs.power != s_des_power)
            act = ACT_POWER;
        else if (r_guide != s_des_guide)
            act = ACT_GUIDE;
        else if (s_des_armed && !r_armed) {
            /* Remote disarmed on its own (heartbeat trip / reset): do not
             * silently re-arm, follow it to the safe state. */
            s_des_armed = false;
            s_des_power = 0;
            ESP_LOGW(TAG, "ATmega is disarmed but arm was still requested; "
                          "dropping to disarmed");
            act = (s_rs.power != 0) ? ACT_POWER : ACT_NONE;
        } else if (now >= next_poll)
            act = ACT_POLL;
    }
    portEXIT_CRITICAL(&s_lock);
    return act;
}

static void link_task(void *arg)
{
    (void)arg;
    int64_t next_poll = esp_timer_get_time();

    for (;;) {
        int64_t now = esp_timer_get_time();
        enum action act = decide(now, next_poll);

        if (act == ACT_NONE) {
            check_link_transition();
            int64_t wait = next_poll - esp_timer_get_time();
            ulTaskNotifyTake(pdTRUE, wait > 0 ? us_to_ticks_ceil(wait) : 1);
            continue;
        }

        uint8_t cmd, pl = 0, len = 1;
        bool want_ready_fast = false;

        portENTER_CRITICAL(&s_lock);
        switch (act) {
        case ACT_ARM0:  cmd = CMD_SET_ARM;   pl = 0; break;
        case ACT_ARM1:  cmd = CMD_SET_ARM;   pl = 1; break;
        case ACT_POWER: cmd = CMD_SET_POWER; pl = s_des_power; break;
        case ACT_GUIDE: cmd = CMD_SET_GUIDE; pl = s_des_guide ? 1 : 0; break;
        default:        cmd = CMD_GET_STATUS; len = 0; break;
        }
        portEXIT_CRITICAL(&s_lock);

        struct reply_status rs;
        if (txn(cmd, &pl, len, &rs)) {
            apply_status(&rs);
            want_ready_fast = (rs.flags & FLAG_ARMED) && !(rs.flags & FLAG_READY);
            next_poll = esp_timer_get_time() +
                        (want_ready_fast ? POLL_FAST_PERIOD_US : POLL_PERIOD_US);
        } else {
            ESP_LOGD(TAG, "no valid reply to cmd 0x%02x", cmd);
            next_poll = esp_timer_get_time() + POLL_PERIOD_US;
            vTaskDelay(pdMS_TO_TICKS(FAIL_BACKOFF_MS));
        }
        check_link_transition();
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

    if (s_task)
        return;

    s_events = xEventGroupCreate();
    if (!s_events) {
        ESP_LOGE(TAG, "event group creation failed");
        return;
    }
    if (uart_driver_install(ATMEGA_UART_PORT, 256, 256, 0, NULL, 0) != ESP_OK ||
        uart_param_config(ATMEGA_UART_PORT, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "UART driver setup failed");
        return;
    }
    /* TX/RX are swapped relative to UART0's IOMUX defaults (43 TX / 44 RX):
     * the board wiring needs TX on GPIO44 and RX on GPIO43. */
    if (uart_set_pin(ATMEGA_UART_PORT, ATMEGA_PIN_TX, ATMEGA_PIN_RX,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed");
        return;
    }

    if (xTaskCreatePinnedToCore(link_task, "atmega_link", LINK_TASK_STACK, NULL,
                                LINK_TASK_PRIO, &s_task, LINK_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "link task creation failed");
        s_task = NULL;
        return;
    }
    ESP_LOGI(TAG, "ATmega link started: UART%d TX=%d RX=%d @ %d baud",
             ATMEGA_UART_PORT, ATMEGA_PIN_TX, ATMEGA_PIN_RX, ATMEGA_BAUD);
}

static void kick(void)
{
    if (s_task)
        xTaskNotifyGive(s_task);
}

void atmega_link_set_armed(bool armed)
{
    portENTER_CRITICAL(&s_lock);
    s_des_armed = armed;
    s_arm_pending = armed;
    if (!armed)
        s_des_power = 0; /* the ATmega zeroes power on disarm */
    portEXIT_CRITICAL(&s_lock);
    kick();
}

void atmega_link_set_guide(bool on)
{
    portENTER_CRITICAL(&s_lock);
    s_des_guide = on;
    portEXIT_CRITICAL(&s_lock);
    kick();
}

void atmega_link_set_power(uint8_t power)
{
    portENTER_CRITICAL(&s_lock);
    s_des_power = power;
    portEXIT_CRITICAL(&s_lock);
    kick();
}

/* cond: 0 = power match, 1 = ready */
static bool wait_cond(int cond, uint8_t power, uint32_t timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    if (!s_events)
        return false;

    for (;;) {
        bool ok, hit = false, desired_armed;

        xEventGroupClearBits(s_events, EVT_CHANGED);
        portENTER_CRITICAL(&s_lock);
        int64_t now = esp_timer_get_time();
        ok = link_ok_locked(now);
        desired_armed = s_des_armed;
        if (ok)
            hit = cond ? (s_rs.flags & FLAG_READY) != 0 : s_rs.power == power;
        portEXIT_CRITICAL(&s_lock);

        if (hit)
            return true;
        if (!ok || (cond && !desired_armed))
            return false; /* nothing will change soon: fail fast */

        int64_t left = deadline - esp_timer_get_time();
        if (left <= 0)
            return false;
        xEventGroupWaitBits(s_events, EVT_CHANGED, pdFALSE, pdFALSE,
                            us_to_ticks_ceil(left));
    }
}

bool atmega_link_wait_power(uint8_t power, uint32_t timeout_ms)
{
    return wait_cond(0, power, timeout_ms);
}

bool atmega_link_wait_ready(uint32_t timeout_ms)
{
    return wait_cond(1, 0, timeout_ms);
}

void atmega_link_get_status(struct atmega_status *out)
{
    portENTER_CRITICAL(&s_lock);
    out->link_ok = link_ok_locked(esp_timer_get_time());
    out->armed = s_rs.flags & FLAG_ARMED;
    out->ready = s_rs.flags & FLAG_READY;
    out->guide_on = s_rs.flags & FLAG_GUIDE;
    out->power = s_rs.power;
    out->stat_bits = s_rs.stat & 0x07;
    out->vdetect_mv = s_rs.mv;
    out->errors = s_errors;
    portEXIT_CRITICAL(&s_lock);
}
