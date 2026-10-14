#include "zbro_link.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "net_wifi.h"
#include "zbro_proto.h"

static const char *TAG = "zbro";

#define LINK_TASK_STACK 4608
#define LINK_TASK_PRIO  4
#define POLL_MS         20
#define PING_MS         1000
#define REPLY_MS        2000     /* the C3 answers every request at once; longer = dead */
#define SILENCE_MS      4500     /* PING is answered every second */
#define CONNECT_MS      3000

enum slot_state { SLOT_IDLE, SLOT_QUEUED, SLOT_SENT, SLOT_DONE };
enum expect { EXP_NONE, EXP_SLOT, EXP_DISCARD, EXP_POS_DISCARD };

/* Single request slot shared between the grbl task and the link task (s_mtx). */
static struct {
    enum slot_state state;
    bool abandon;                    /* caller gave up: drop the reply */
    char line[ZBRO_MAX_LINE + 1];
    char reply[ZBRO_MAX_REPLY];
    int result;
} s_slot;

static SemaphoreHandle_t s_mtx;      /* guards s_slot */
static SemaphoreHandle_t s_call;     /* one blocking caller at a time */
static SemaphoreHandle_t s_done;     /* slot completed */
static SemaphoreHandle_t s_evt;      /* EVT HOMED/DONE/ERR for the armed operation */

static volatile bool s_online;
static volatile float s_pos;
static volatile int s_state = -1;
static volatile bool s_homed, s_endstop;
static char s_ip[16], s_fw[24];

static volatile bool s_evt_armed;
static volatile int s_evt_code;
static volatile bool s_stop_req;
static volatile bool s_stopped;      /* a STOP was requested during the armed operation */
static bool (*s_abort_fn)(void);

/* Link-task-only parser state (too big for the 4 KB stack). */
static struct zbro_lines s_lines;
static struct zbro_reply s_rep;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static bool aborted(void)
{
    return s_abort_fn && s_abort_fn();
}

/* ------------------------------------------------------- link task side -- */

static void slot_finish(int result, const char *reply)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (s_slot.state == SLOT_QUEUED || s_slot.state == SLOT_SENT) {
        if (s_slot.abandon) {
            s_slot.state = SLOT_IDLE;
        } else {
            s_slot.result = result;
            strlcpy(s_slot.reply, reply ? reply : "", sizeof(s_slot.reply));
            s_slot.state = SLOT_DONE;
            xSemaphoreGive(s_done);
        }
    }
    xSemaphoreGive(s_mtx);
}

static bool send_line(int fd, const char *line)
{
    char b[ZBRO_MAX_LINE + 2];
    size_t n = strlcpy(b, line, sizeof(b) - 1);
    size_t off = 0;

    b[n++] = '\n';
    while (off < n) {
        int w = send(fd, b + off, n - off, 0);

        if (w <= 0)
            return false;
        off += (size_t)w;
    }
    return true;
}

static void evt_fire(int code)
{
    if (s_evt_armed) {
        s_evt_code = code;
        xSemaphoreGive(s_evt);
    }
}

/* Returns the new expectation after handling one received line. */
static enum expect handle_line(const char *line, size_t len, enum expect exp)
{
    const struct zbro_reply *r = &s_rep;

    if (!zbro_parse_reply(line, len, &s_rep))
        return exp;
    switch (r->kind) {
    case ZBRO_R_EVT_HOMED:
    case ZBRO_R_EVT_DONE:
        s_pos = r->pos;
        evt_fire(0);
        return exp;
    case ZBRO_R_EVT_ERR:
        evt_fire(r->code);
        return exp;
    case ZBRO_R_EVT_CAL:
        return exp;
    case ZBRO_R_POS:
        s_pos = r->pos;
        s_state = r->state;
        s_homed = r->homed;
        s_endstop = r->endstop;
        /* A POS line answers a POS request, otherwise it is a progress report. */
        if (exp == EXP_POS_DISCARD)
            return EXP_NONE;
        if (exp == EXP_SLOT && strncmp(s_slot.line, "POS", 3) == 0) {
            slot_finish(0, line);
            return EXP_NONE;
        }
        return exp;
    default:
        break;
    }
    if (exp == EXP_SLOT)
        slot_finish(r->kind == ZBRO_R_ERR ? r->code : 0, line);
    return EXP_NONE;
}

static bool wait_readable(int fd, int ms)
{
    fd_set rs;
    struct timeval tv = { .tv_sec = ms / 1000, .tv_usec = (ms % 1000) * 1000 };

    FD_ZERO(&rs);
    FD_SET(fd, &rs);
    return select(fd + 1, &rs, NULL, NULL, &tv) > 0;
}

static int tcp_connect(const char *ip, int port)
{
    struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = htons((uint16_t)port) };
    struct timeval tv = { .tv_sec = 1 };
    fd_set ws;
    int fd, one = 1, idle = 5, intvl = 2, cnt = 3, err = 0;
    socklen_t el = sizeof(err);

    if (inet_pton(AF_INET, ip, &sa.sin_addr) != 1)
        return -1;
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    if (connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0 && errno != EINPROGRESS)
        goto fail;
    FD_ZERO(&ws);
    FD_SET(fd, &ws);
    tv.tv_sec = CONNECT_MS / 1000;
    if (select(fd + 1, NULL, &ws, NULL, &tv) <= 0)
        goto fail;
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &el) != 0 || err)
        goto fail;
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
    return fd;
fail:
    close(fd);
    return -1;
}

/* Reads available bytes and handles every complete line. False = connection lost. */
static bool pump_rx(int fd, enum expect *exp, int64_t *last_rx)
{
    uint8_t buf[96];
    int n = recv(fd, buf, sizeof(buf), 0);

    if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
        return false;
    for (int i = 0; i < n; i++) {
        const char *line;
        size_t len;

        if (zbro_lines_feed(&s_lines, (char)buf[i], &line, &len) == 1) {
            *last_rx = now_ms();
            *exp = handle_line(line, len, *exp);
        }
    }
    return true;
}

/* Runs one TCP session. Returns true if the link came up (HELLO ok). */
static bool session(const char *ip, int port)
{
    enum expect exp = EXP_NONE;
    int64_t t_sent = 0, t_ping = 0, last_rx;
    bool up = false;
    unsigned pings = 0;
    int fd = tcp_connect(ip, port);

    if (fd < 0) {
        ESP_LOGW(TAG, "connect %s:%d failed", ip, port);
        return false;
    }
    zbro_lines_init(&s_lines);
    s_rep.kind = ZBRO_R_OK;
    if (!send_line(fd, "HELLO"))
        goto out;
    last_rx = now_ms();
    for (int64_t t0 = last_rx; now_ms() - t0 < REPLY_MS && !up;) {
        if (wait_readable(fd, POLL_MS)) {
            int n;
            uint8_t c;
            const char *line;
            size_t len;

            while ((n = recv(fd, &c, 1, MSG_DONTWAIT)) == 1) {
                if (zbro_lines_feed(&s_lines, (char)c, &line, &len) == 1) {
                    if (zbro_parse_reply(line, len, &s_rep) && s_rep.kind == ZBRO_R_HELLO) {
                        strlcpy(s_fw, s_rep.text, sizeof(s_fw));
                        up = true;
                    }
                    break;
                }
            }
            if (n == 0)
                break;
        }
    }
    if (!up)
        goto out;

    strlcpy(s_ip, ip, sizeof(s_ip));
    s_online = true;
    ESP_LOGI(TAG, "linked to %s:%d fw %s", ip, port, s_fw);
    last_rx = t_ping = now_ms();
    send_line(fd, "POS");
    exp = EXP_POS_DISCARD;
    t_sent = now_ms();

    while (net_wifi_up()) {
        int64_t t = now_ms();

        if (exp == EXP_NONE) {
            bool ok = true;

            xSemaphoreTake(s_mtx, portMAX_DELAY);
            if (s_stop_req) {
                s_stop_req = false;
                xSemaphoreGive(s_mtx);
                ok = send_line(fd, "STOP");
                exp = EXP_DISCARD;
            } else if (s_slot.state == SLOT_QUEUED) {
                char line[ZBRO_MAX_LINE + 1];

                strlcpy(line, s_slot.line, sizeof(line));
                s_slot.state = SLOT_SENT;
                xSemaphoreGive(s_mtx);
                ok = send_line(fd, line);
                exp = EXP_SLOT;
            } else {
                xSemaphoreGive(s_mtx);
                if (t - t_ping >= PING_MS) {
                    t_ping = t;
                    if (++pings % 5 == 0) {          /* refresh state / homed / endstop at rest */
                        ok = send_line(fd, "POS");
                        exp = EXP_POS_DISCARD;
                    } else {
                        ok = send_line(fd, "PING");
                        exp = EXP_DISCARD;
                    }
                }
            }
            if (!ok)
                break;
            if (exp != EXP_NONE)
                t_sent = t;
        }
        if (wait_readable(fd, POLL_MS) && !pump_rx(fd, &exp, &last_rx))
            break;
        t = now_ms();
        if ((exp != EXP_NONE && t - t_sent > REPLY_MS) || t - last_rx > SILENCE_MS) {
            ESP_LOGW(TAG, "C3 not answering, dropping the link");
            break;
        }
    }
out:
    if (up)
        ESP_LOGW(TAG, "link down");
    s_online = false;
    slot_finish(ZBRO_ERR_LINK, "");
    evt_fire(ZBRO_ERR_LINK);
    close(fd);
    return up;
}

static int udp_open(void)
{
    struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = htons(ZBRO_PORT_UDP),
                              .sin_addr.s_addr = htonl(INADDR_ANY) };
    struct timeval tv = { .tv_sec = 1 };
    int one = 1;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (fd < 0)
        return -1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void link_task(void *arg)
{
    uint32_t backoff = 1000;
    int udp = -1;

    (void)arg;
    for (;;) {
        char pkt[96], ip[16], host[32];
        int port, n;

        if (!net_wifi_up()) {
            if (udp >= 0) {
                close(udp);
                udp = -1;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (udp < 0)
            udp = udp_open();
        if (udp < 0) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        n = recvfrom(udp, pkt, sizeof(pkt) - 1, 0, NULL, NULL);
        if (n <= 0)
            continue;
        while (n > 0 && (pkt[n - 1] == '\n' || pkt[n - 1] == '\r'))
            n--;
        pkt[n] = '\0';
        if (!zbro_parse_beacon(pkt, (size_t)n, ip, sizeof(ip), &port, host, sizeof(host)))
            continue;
        close(udp);                      /* drop queued beacons, listen again after the session */
        udp = -1;
        if (session(ip, port))
            backoff = 1000;
        else if (backoff < 8000)
            backoff *= 2;
        vTaskDelay(pdMS_TO_TICKS(backoff));
    }
}

/* ------------------------------------------------------------- API ------- */

bool zbro_link_start(void)
{
    s_mtx = xSemaphoreCreateMutex();
    s_call = xSemaphoreCreateMutex();
    s_done = xSemaphoreCreateBinary();
    s_evt = xSemaphoreCreateBinary();
    if (!s_mtx || !s_call || !s_done || !s_evt)
        return false;
    return xTaskCreatePinnedToCore(link_task, "zbro", LINK_TASK_STACK, NULL, LINK_TASK_PRIO,
                                   NULL, 1) == pdPASS;
}

bool zbro_online(void)
{
    return s_online;
}

float zbro_pos(void)
{
    return s_pos;
}

void zbro_get_info(struct zbro_info *out)
{
    out->online = s_online;
    strlcpy(out->ip, s_ip, sizeof(out->ip));
    strlcpy(out->fw, s_fw, sizeof(out->fw));
    out->pos = s_pos;
    out->state = s_state;
    out->homed = s_homed;
    out->endstop = s_endstop;
}

void zbro_set_abort_fn(bool (*fn)(void))
{
    s_abort_fn = fn;
}

void zbro_stop(void)
{
    if (!s_evt_armed)
        return;
    s_stopped = true;
    s_stop_req = true;
}

static int request_locked(const char *line, char *reply, size_t n, uint32_t timeout_ms)
{
    int64_t t0 = now_ms();
    int r;

    /* A previous caller may have abandoned a request that is still in flight (bounded by REPLY_MS). */
    for (;;) {
        xSemaphoreTake(s_mtx, portMAX_DELAY);
        if (s_slot.state == SLOT_IDLE)
            break;                   /* keeps s_mtx */
        xSemaphoreGive(s_mtx);
        if (now_ms() - t0 > REPLY_MS + 500)
            return ZBRO_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    strlcpy(s_slot.line, line, sizeof(s_slot.line));
    s_slot.abandon = false;
    s_slot.state = SLOT_QUEUED;
    xSemaphoreTake(s_done, 0);
    xSemaphoreGive(s_mtx);

    for (;;) {
        xSemaphoreTake(s_done, pdMS_TO_TICKS(POLL_MS));

        xSemaphoreTake(s_mtx, portMAX_DELAY);
        if (s_slot.state == SLOT_DONE) {
            r = s_slot.result;
            if (reply && n)
                strlcpy(reply, s_slot.reply, n);
            s_slot.state = SLOT_IDLE;
            xSemaphoreGive(s_mtx);
            return r;
        }
        if (aborted() || (uint32_t)(now_ms() - t0) > timeout_ms) {
            r = aborted() ? ZBRO_ERR_ABORTED : ZBRO_ERR_TIMEOUT;
            if (s_slot.state == SLOT_QUEUED)
                s_slot.state = SLOT_IDLE;
            else
                s_slot.abandon = true;   /* in flight: the link task drops the reply */
            xSemaphoreGive(s_mtx);
            return r;
        }
        xSemaphoreGive(s_mtx);
    }
}

int zbro_request(const char *line, char *reply, size_t n, uint32_t timeout_ms)
{
    int r;

    if (reply && n)
        reply[0] = '\0';
    if (!s_online)
        return ZBRO_ERR_OFFLINE;
    if (strlen(line) > ZBRO_MAX_LINE)
        return ZBRO_E_ARG;
    if (xSemaphoreTake(s_call, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
        return ZBRO_ERR_TIMEOUT;
    r = request_locked(line, reply, n, timeout_ms);
    xSemaphoreGive(s_call);
    return r;
}

/* Sends HOME / MOVE (replied OK at once) and waits for the matching EVT. */
static int run_op(const char *line, int timeout_s)
{
    char rep[ZBRO_MAX_LINE];
    int64_t t0;
    int r;

    if (!s_online)
        return ZBRO_ERR_OFFLINE;
    xSemaphoreTake(s_evt, 0);
    s_evt_code = ZBRO_ERR_LINK;
    s_stopped = false;
    s_stop_req = false;
    s_evt_armed = true;
    r = zbro_request(line, rep, sizeof(rep), 3000);
    if (r) {
        s_evt_armed = false;
        return r;
    }
    t0 = now_ms();
    for (;;) {
        if (xSemaphoreTake(s_evt, pdMS_TO_TICKS(POLL_MS)) == pdTRUE) {
            r = s_evt_code;
            break;
        }
        if (aborted()) {
            r = ZBRO_ERR_ABORTED;
        } else if (now_ms() - t0 > (int64_t)timeout_s * 1000) {
            r = ZBRO_ERR_TIMEOUT;
        } else if (!s_online) {
            r = ZBRO_ERR_LINK;
            break;
        } else {
            continue;
        }
        /* Stop the axis and swallow the resulting EVT so it cannot complete the next operation. */
        s_stop_req = true;
        xSemaphoreTake(s_evt, pdMS_TO_TICKS(1500));
        break;
    }
    if (r == 0 && s_stopped)
        r = ZBRO_ERR_ABORTED;        /* hold / reset stopped it short of the target */
    s_evt_armed = false;
    return r;
}

int zbro_home(int timeout_s)
{
    return run_op("HOME", timeout_s);
}

int zbro_move(float mm, float speed, int timeout_s)
{
    char b[ZBRO_MAX_LINE + 2];
    int n = zbro_fmt_req_move(b, sizeof(b), false, mm, speed);

    if (n < 2)
        return ZBRO_E_ARG;
    b[n - 1] = '\0';                 /* drop the '\n' */
    return run_op(b, timeout_s);
}
