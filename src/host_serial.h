#ifndef HOST_SERIAL_H_
#define HOST_SERIAL_H_

/*
 * Host link over the native USB-Serial-JTAG port.
 *
 *  RX: usb_serial_jtag driver ring buffer -> host_rx task (core 1, prio 10,
 *      the highest on core 1). The task acts on GRBL realtime bytes
 *      ('?' '!' '~' 0x18 0x85) immediately through the handlers below and
 *      appends everything else to a byte stream buffer that grbl.c drains
 *      line by line (host_rx_read()). It never blocks on that buffer: with
 *      the host limited to 1024 unacknowledged bytes (character counting)
 *      it cannot fill; if it ever would, bytes are dropped and counted.
 *
 *      A soft reset (0x18) additionally leaves HOST_RX_RESET_MARK in the
 *      stream, at exactly the position where it was received, so the GRBL
 *      task can discard everything queued before it and nothing after it.
 *      The mark never occurs in normal data (0x18 is not printable G-code).
 *
 *  TX: host_write()/host_printf() serialise whole writes under a mutex, so
 *      ok lines, status reports and log lines from any task or core never
 *      interleave. Not callable from ISRs.
 *
 *  Logging: esp_log output is redirected to GRBL feedback messages,
 *      "[MSG:<text>]\r\n" (ANSI colours stripped, one line per message).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HOST_RX_RESET_MARK 0x18

struct host_rt_handlers {
    void (*status)(void);       /* '?'  */
    void (*hold)(void);         /* '!'  */
    void (*resume)(void);       /* '~'  */
    void (*reset)(void);        /* 0x18 */
    void (*jog_cancel)(void);   /* 0x85 */
};

/* Installs the driver, starts the RX task and redirects esp_log. */
bool host_serial_init(void);

/* Handlers run in the RX task; they must be quick and must not block on
 * anything the GRBL task holds. NULL members are ignored. */
void host_serial_set_handlers(const struct host_rt_handlers *h);

void host_write(const char *s, size_t len);
void host_puts(const char *s);
void host_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Reads up to max bytes of the (non-realtime) input stream, waiting up to
 * wait_ms for the first byte. Returns the count (0 on timeout). */
size_t host_rx_read(uint8_t *buf, size_t max, uint32_t wait_ms);

/* Total non-realtime bytes accepted into the stream so far (wraps). The
 * difference to the bytes the GRBL task has acknowledged is the host's
 * unacknowledged character count. */
uint32_t host_rx_accepted(void);

/* Bytes dropped because the stream buffer was full (should stay 0). */
uint32_t host_rx_dropped(void);

#endif /* HOST_SERIAL_H_ */
