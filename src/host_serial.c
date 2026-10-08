#include "host_serial.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#define RX_DRIVER_BUF    4096
#define TX_DRIVER_BUF    2048
#define RX_STREAM_BUF    4096
#define RX_TASK_STACK    4096
#define RX_TASK_PRIO     10
#define TX_CHUNK         256
#define TX_DEAD_MS       1000   /* after a timed-out write, drop output this long */
#define PROTO_TX_WAIT_MS 1000
#define LOG_MUTEX_MS     10
#define LOG_TX_WAIT_MS   5
#define LOG_LINE_MAX     200

static StreamBufferHandle_t s_rx_stream;
static SemaphoreHandle_t s_tx_mutex;
static struct host_rt_handlers s_h;
static volatile uint32_t s_accepted;
static volatile uint32_t s_dropped;
static volatile TickType_t s_tx_dead_until;
static volatile bool s_tx_dead;
static volatile bool s_up;

/* ---------------------------------------------------------------- TX --- */

static void tx_locked(const char *s, size_t n, TickType_t wait)
{
    if (s_tx_dead) {
        if ((int32_t)(xTaskGetTickCount() - s_tx_dead_until) < 0)
            return;                         /* host not reading: drop */
        s_tx_dead = false;
    }
    while (n) {
        size_t c = n > TX_CHUNK ? TX_CHUNK : n;

        if (usb_serial_jtag_write_bytes(s, c, wait) == 0) {
            s_tx_dead_until = xTaskGetTickCount() + pdMS_TO_TICKS(TX_DEAD_MS);
            s_tx_dead = true;
            return;
        }
        s += c;
        n -= c;
    }
}

void host_write(const char *s, size_t len)
{
    if (!len)
        return;
    if (!s_up) {
        fwrite(s, 1, len, stdout);          /* before host_serial_init() */
        return;
    }
    xSemaphoreTakeRecursive(s_tx_mutex, portMAX_DELAY);
    tx_locked(s, len, pdMS_TO_TICKS(PROTO_TX_WAIT_MS));
    xSemaphoreGiveRecursive(s_tx_mutex);
}

void host_puts(const char *s)
{
    host_write(s, strlen(s));
}

void host_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    if ((size_t)n >= sizeof(buf))
        n = (int)sizeof(buf) - 1;
    host_write(buf, (size_t)n);
}

/* esp_log -> "[MSG:...]". Runs in the caller's context, so it only uses a
 * stack buffer, a bounded mutex wait and a short write timeout; if the link
 * is busy the line is dropped rather than stalling the logging task. */
static int host_vprintf(const char *fmt, va_list ap)
{
    char buf[LOG_LINE_MAX + 8];
    char *text = buf + 5;                   /* "[MSG:" */
    int n, o = 0;

    if (xPortInIsrContext())
        return 0;
    n = vsnprintf(text, LOG_LINE_MAX, fmt, ap);
    if (n <= 0)
        return n;
    if (n >= LOG_LINE_MAX)
        n = LOG_LINE_MAX - 1;

    /* Strip ANSI escapes, flatten control characters, trim the tail. */
    for (int i = 0; i < n; i++) {
        unsigned char c = (unsigned char)text[i];

        if (c == 0x1b) {
            i++;
            if (i < n && text[i] == '[') {
                while (i + 1 < n && !(text[i + 1] >= '@' && text[i + 1] <= '~'))
                    i++;
                i++;                        /* final byte of the sequence */
            }
            continue;
        }
        if (c < 0x20 || c == 0x7f || c == ']' || c == '\n')
            c = (c == '\n' || c == '\r') ? ' ' : (c == ']' ? ')' : ' ');
        text[o++] = (char)c;
    }
    while (o > 0 && text[o - 1] == ' ')
        o--;
    if (o == 0)
        return n;

    memcpy(buf, "[MSG:", 5);
    buf[5 + o] = ']';
    buf[6 + o] = '\r';
    buf[7 + o] = '\n';

    if (!s_up) {
        fwrite(buf, 1, (size_t)o + 8, stdout);
        return n;
    }
    if (xSemaphoreTakeRecursive(s_tx_mutex, pdMS_TO_TICKS(LOG_MUTEX_MS)) == pdTRUE) {
        tx_locked(buf, (size_t)o + 8, pdMS_TO_TICKS(LOG_TX_WAIT_MS));
        xSemaphoreGiveRecursive(s_tx_mutex);
    }
    return n;
}

/* ---------------------------------------------------------------- RX --- */

static void rx_flush(uint8_t *batch, size_t *len)
{
    if (*len) {
        size_t sent = xStreamBufferSend(s_rx_stream, batch, *len, 0);

        s_accepted += (uint32_t)sent;
        if (sent < *len)
            s_dropped += (uint32_t)(*len - sent);
        *len = 0;
    }
}

static void rx_task(void *arg)
{
    uint8_t in[128];
    uint8_t batch[128];

    (void)arg;
    for (;;) {
        int got = usb_serial_jtag_read_bytes(in, sizeof(in), pdMS_TO_TICKS(100));
        size_t bl = 0;

        if (got <= 0)
            continue;
        s_tx_dead = false;                  /* the host is alive */
        for (int i = 0; i < got; i++) {
            uint8_t c = in[i];

            switch (c) {
            case '?':
                if (s_h.status)
                    s_h.status();
                break;
            case '!':
                if (s_h.hold)
                    s_h.hold();
                break;
            case '~':
                if (s_h.resume)
                    s_h.resume();
                break;
            case 0x85:
                if (s_h.jog_cancel)
                    s_h.jog_cancel();
                break;
            case HOST_RX_RESET_MARK: {
                uint8_t mark = HOST_RX_RESET_MARK;

                rx_flush(batch, &bl);       /* earlier bytes stay ahead of the mark */
                if (s_h.reset)
                    s_h.reset();
                /* The mark must not be lost: wait briefly if the stream is
                 * (pathologically) full - the GRBL task is draining it. */
                if (xStreamBufferSend(s_rx_stream, &mark, 1, pdMS_TO_TICKS(100)) != 1)
                    s_dropped++;
                break;
            }
            default:
                if (c >= 0x80)
                    break;                  /* other GRBL realtime/override bytes: ignored */
                batch[bl++] = c;
                break;
            }
        }
        rx_flush(batch, &bl);
    }
}

size_t host_rx_read(uint8_t *buf, size_t max, uint32_t wait_ms)
{
    return xStreamBufferReceive(s_rx_stream, buf, max, pdMS_TO_TICKS(wait_ms));
}

uint32_t host_rx_accepted(void)
{
    return s_accepted;
}

uint32_t host_rx_dropped(void)
{
    return s_dropped;
}

void host_serial_set_handlers(const struct host_rt_handlers *h)
{
    s_h = *h;
}

bool host_serial_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = TX_DRIVER_BUF,
        .rx_buffer_size = RX_DRIVER_BUF,
    };

    if (s_up)
        return true;
    s_tx_mutex = xSemaphoreCreateRecursiveMutex();
    s_rx_stream = xStreamBufferCreate(RX_STREAM_BUF, 1);
    if (!s_tx_mutex || !s_rx_stream)
        return false;
    if (!usb_serial_jtag_is_driver_installed() &&
        usb_serial_jtag_driver_install(&cfg) != ESP_OK)
        return false;
    if (xTaskCreatePinnedToCore(rx_task, "host_rx", RX_TASK_STACK, NULL,
                                RX_TASK_PRIO, NULL, 1) != pdPASS)
        return false;
    s_up = true;
    esp_log_set_vprintf(host_vprintf);
    return true;
}
