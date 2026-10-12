/* Host test for the two-host logic in src/host_serial.c: arbitration, the
 * Ctrl-X handler/mark pairing, hand-over hold and net_down abort. The real
 * source file is compiled against a tiny FreeRTOS shim (shim/), so only the
 * RTOS is faked. Single-threaded: it checks the logic, not the races. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../../src/host_serial.c"

TickType_t g_tick = 100000;
const char *g_task_name = "host_rx";
static char g_usb[512], g_net[512];
static size_t g_usb_n, g_net_n;
static int fails, checks;

int usb_serial_jtag_write_bytes(const void *s, size_t n, TickType_t wait)
{
    (void)wait;
    if (g_usb_n + n < sizeof(g_usb)) {
        memcpy(g_usb + g_usb_n, s, n);
        g_usb_n += n;
    }
    return (int)n;
}

static int net_write(const char *s, size_t n, TickType_t wait)
{
    (void)wait;
    if (g_net_n + n < sizeof(g_net)) {
        memcpy(g_net + g_net_n, s, n);
        g_net_n += n;
    }
    return (int)n;
}

static int n_reset, n_hold, n_status;
static bool g_busy;
static void h_reset(void) { n_reset++; }
static void h_hold(void) { n_hold++; }
static void h_status(void) { n_status++; host_puts("S"); }
static bool busy_fn(void) { return g_busy; }

#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static size_t drain(uint8_t *out, size_t max) { return xStreamBufferReceive(s_rx_stream, out, max, 0); }
static void clear_all(void)
{
    uint8_t t[256];
    while (drain(t, sizeof(t)))
        ;
    g_usb_n = g_net_n = 0;
    n_reset = n_hold = n_status = 0;
}
static void usb(const char *s) { rx_ingest(&s_usb, (const uint8_t *)s, (int)strlen(s)); }
static void net(const char *s) { host_link_net_rx((const uint8_t *)s, (int)strlen(s)); }
static int logf_(const char *fmt, ...)
{
    va_list ap;
    int r;

    va_start(ap, fmt);
    r = host_vprintf(fmt, ap);
    va_end(ap);
    return r;
}
static int marks(const uint8_t *b, size_t n) { int k = 0; for (size_t i = 0; i < n; i++) k += b[i] == 0x18; return k; }

int main(void)
{
    uint8_t buf[256];
    size_t n;
    struct host_rt_handlers h = { .status = h_status, .hold = h_hold, .reset = h_reset };

    s_up = false;
    CHECK(host_serial_init());
    host_serial_set_handlers(&h);
    host_serial_set_busy_fn(busy_fn);

    /* 1. plain USB line reaches the stream; status reply goes to USB */
    usb("G0 X1\n?");
    n = drain(buf, sizeof(buf));
    CHECK(n == 6 && memcmp(buf, "G0 X1\n", 6) == 0);
    CHECK(n_status == 1 && g_usb_n == 1 && g_usb[0] == 'S');

    /* 2. N1: two Ctrl-X in one block -> one handler call, one mark */
    clear_all();
    usb("\x18\x18");
    n = drain(buf, sizeof(buf));
    CHECK(n_reset == 1 && marks(buf, n) == 1);

    /* 3. N2: '!' before Ctrl-X must not act; bytes before it are dropped, bytes after kept */
    clear_all();
    usb("G1 X5\n!\x18G2\n");
    n = drain(buf, sizeof(buf));
    CHECK(n_hold == 0 && n_reset == 1);
    CHECK(n == 4 && buf[0] == 0x18 && memcmp(buf + 1, "G2\n", 3) == 0);

    /* 4. Ctrl-X then '!' in order: reset first, hold after */
    clear_all();
    usb("\x18!");
    n = drain(buf, sizeof(buf));
    CHECK(n_reset == 1 && n_hold == 1 && marks(buf, n) == 1);

    /* 5. a TCP client attaches; USB is idle -> NET input switches the active host.
     * First let the hand-over hold of the USB G-code from above run out. */
    clear_all();
    host_link_net_up(net_write);
    g_tick += 5000;
    net("G0 X2\n?");
    n = drain(buf, sizeof(buf));
    CHECK(n == 6 && memcmp(buf, "G0 X2\n", 6) == 0);
    CHECK(g_net_n == 1 && g_net[0] == 'S' && g_usb_n == 0);   /* replies follow the active host */

    /* 6. busy machine: USB input is ignored, Ctrl-X still works from USB */
    clear_all();
    g_busy = true;
    usb("G1 X9\n");
    n = drain(buf, sizeof(buf));
    CHECK(n == 0);
    usb("G1 X9\n\x18");
    n = drain(buf, sizeof(buf));
    CHECK(n_reset == 1 && n == 1 && buf[0] == 0x18);
    usb("?");
    CHECK(n_status == 0);                                      /* status of a non-active host is dropped too */
    g_busy = false;

    /* 7. hand-over hold: G-code from the active host (NET) blocks USB for 1 s only */
    clear_all();
    g_tick += 5000;
    net("G0 X3\n");
    drain(buf, sizeof(buf));
    g_tick += 500;
    usb("G0 X4\n");
    CHECK(drain(buf, sizeof(buf)) == 0);                       /* inside the hold: ignored */
    g_tick += 600;
    usb("G0 X4\n");
    CHECK(drain(buf, sizeof(buf)) == 6);                       /* hold over: USB takes over */
    host_puts("U");
    CHECK(g_usb_n == 1 && g_usb[0] == 'U');

    /* 8. net_down while NET is not active: no abort */
    clear_all();
    host_link_net_down();
    CHECK(n_reset == 0);
    CHECK(*s_rx_mutex == 0 && *s_tx_mutex == 0);               /* every lock released */

    /* 9. net_down while NET is active: one reset, USB active again */
    clear_all();
    host_link_net_up(net_write);
    g_tick += 5000;
    net("G0 X5\n");
    drain(buf, sizeof(buf));
    host_link_net_down();
    n = drain(buf, sizeof(buf));
    CHECK(n_reset == 1 && marks(buf, n) == 1);
    CHECK(s_active == &s_usb && s_net.write == NULL);
    CHECK(*s_rx_mutex == 0 && *s_tx_mutex == 0);

    /* 10. log lines from the lwIP / event-loop tasks are dropped, others are written */
    clear_all();
    g_task_name = "tiT";
    logf_("hello");
    CHECK(g_usb_n == 0);
    g_task_name = "sys_evt";
    logf_("hello");
    CHECK(g_usb_n == 0);
    g_task_name = "host_rx";
    logf_("hello");
    CHECK(g_usb_n == strlen("[MSG:hello]\r\n") && memcmp(g_usb, "[MSG:hello]\r\n", g_usb_n) == 0);

    /* 11. the net bytes after a client is gone are ignored (no write callback) */
    clear_all();
    net("G0 X6\n");
    CHECK(drain(buf, sizeof(buf)) == 0);

    printf("%d checks, %d failures\n", checks, fails);
    return fails != 0;
}
