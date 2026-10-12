#include "net_server.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host_serial.h"
#include "lwip/sockets.h"

#define SRV_TASK_STACK 4096
#define TX_TASK_STACK  4096
#define TASK_PRIO      10               /* same as host_rx: '!' and Ctrl-X latency */
#define RX_CHUNK       512
#define SELECT_MS      100
#define TX_RING        8192             /* power of two */
#define TX_SEGMENT     1440             /* one TCP segment */
#define TX_GAP_MS      4                /* at most one send per this long: replies pile up and go out together */
#define TX_STALL_MS    1000             /* nothing sent for this long with data waiting: client is gone */
#define OVERFLOW_MS    1000             /* ring full for this long: client is gone */

static const char *TAG = "net_server";

static volatile int s_fd = -1;
static volatile bool s_started;
static volatile bool s_kick;            /* drop the client from the server task */
static SemaphoreHandle_t s_fd_mutex;    /* held while the TX task uses the socket, and to close it */
static TaskHandle_t s_tx_task;

/* TX ring: single producer (whoever holds the host TX mutex), single consumer
 * (the TX task). The GRBL task, the status handler and the loggers only copy
 * bytes into it, so a slow WiFi link can never block them; the TX task sends
 * what has piled up in as few segments as possible. */
static uint8_t s_ring[TX_RING];
static volatile uint32_t s_head, s_tail;
static volatile TickType_t s_overflow_since;

/* Counters for $S. */
static volatile uint32_t s_kicks_tx, s_kicks_link, s_peer_closed, s_recv_err, s_ring_max;
static volatile uint32_t s_sends, s_sent_bytes;
static volatile int s_last_errno;

/* Called under the host TX mutex; never blocks. */
static int net_write(const char *s, size_t n, TickType_t wait)
{
    uint32_t h = s_head;
    uint32_t t = __atomic_load_n(&s_tail, __ATOMIC_ACQUIRE);
    uint32_t used = h - t;

    (void)wait;
    if (s_fd < 0)
        return 0;
    if (n > TX_RING - used) {
        /* The client has not taken a reply for a long time. Drop this one;
         * if it stays that way, the connection is dead. */
        TickType_t now = xTaskGetTickCount();

        if (!s_overflow_since)
            s_overflow_since = now ? now : 1;
        else if ((TickType_t)(now - s_overflow_since) > pdMS_TO_TICKS(OVERFLOW_MS))
            s_kick = true;
        return (int)n;
    }
    s_overflow_since = 0;
    for (size_t i = 0; i < n; i++)
        s_ring[(h + i) & (TX_RING - 1)] = (uint8_t)s[i];
    __atomic_store_n(&s_head, h + (uint32_t)n, __ATOMIC_RELEASE);
    if (used + n > s_ring_max)
        s_ring_max = used + (uint32_t)n;
    xTaskNotifyGive(s_tx_task);
    return (int)n;
}

static void tx_task(void *arg)
{
    static uint8_t seg[TX_SEGMENT];
    TickType_t stall_since = 0;
    TickType_t last_send = 0;

    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SELECT_MS));
        /* One tiny send() per "ok" costs about 3 ms in lwIP and the WiFi driver and,
         * because this task outranks the GRBL task, nothing would ever pile up:
         * the link topped out near 350 lines/s. Sparse replies (a status, a lone
         * ok) still go out at once; during streaming they go out in batches. */
        {
            TickType_t since = xTaskGetTickCount() - last_send;

            if (since < pdMS_TO_TICKS(TX_GAP_MS))
                vTaskDelay(pdMS_TO_TICKS(TX_GAP_MS) - since);
        }
        for (;;) {
            uint32_t t = s_tail;
            uint32_t avail = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE) - t;
            uint32_t n;
            int fd, r;

            if (avail == 0) {
                stall_since = 0;
                break;
            }
            xSemaphoreTake(s_fd_mutex, portMAX_DELAY);
            fd = s_fd;
            if (fd < 0) {
                xSemaphoreGive(s_fd_mutex);
                break;
            }
            n = avail > TX_SEGMENT ? TX_SEGMENT : avail;
            for (uint32_t i = 0; i < n; i++)
                seg[i] = s_ring[(t + i) & (TX_RING - 1)];
            r = send(fd, seg, n, MSG_DONTWAIT);
            if (r > 0) {
                __atomic_store_n(&s_tail, t + (uint32_t)r, __ATOMIC_RELEASE);
                stall_since = 0;
                last_send = xTaskGetTickCount();
                s_sends++;
                s_sent_bytes += (uint32_t)r;
            } else if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                struct timeval tv = { 0, 50 * 1000 };
                TickType_t now = xTaskGetTickCount();
                fd_set wf;

                FD_ZERO(&wf);
                FD_SET(fd, &wf);
                select(fd + 1, NULL, &wf, NULL, &tv);
                if (!stall_since)
                    stall_since = now ? now : 1;
                else if ((TickType_t)(now - stall_since) > pdMS_TO_TICKS(TX_STALL_MS)) {
                    s_kicks_tx++;
                    s_kick = true;
                    stall_since = 0;
                }
            } else {
                s_last_errno = errno;
                s_kicks_tx++;
                s_kick = true;
                stall_since = 0;
                xSemaphoreGive(s_fd_mutex);
                break;
            }
            xSemaphoreGive(s_fd_mutex);
            if (s_kick)
                break;
        }
    }
}

static void client_options(int fd)
{
    int one = 1, idle = 3, intvl = 1, cnt = 3;

    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
}

static void drop_client(void)
{
    int fd = s_fd;

    if (fd < 0)
        return;
    host_link_net_down();               /* no producer can enqueue after this */
    xSemaphoreTake(s_fd_mutex, portMAX_DELAY);
    s_fd = -1;
    s_tail = s_head;                    /* unsent replies are for a client that is gone */
    s_overflow_since = 0;
    xSemaphoreGive(s_fd_mutex);
    close(fd);
    ESP_LOGI(TAG, "client disconnected");
}

static int listen_socket(void)
{
    struct sockaddr_in a = { 0 };
    int one = 1;
    int ls = socket(AF_INET, SOCK_STREAM, 0);

    if (ls < 0)
        return -1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(NET_SERVER_PORT);
    if (bind(ls, (struct sockaddr *)&a, sizeof(a)) != 0 || listen(ls, 1) != 0) {
        close(ls);
        return -1;
    }
    fcntl(ls, F_SETFL, fcntl(ls, F_GETFL, 0) | O_NONBLOCK);
    return ls;
}

static void server_task(void *arg)
{
    uint8_t buf[RX_CHUNK];
    int ls = -1;

    (void)arg;
    for (;;) {
        fd_set rf;
        struct timeval tv = { 0, SELECT_MS * 1000 };
        int fd, mx;

        if (s_kick) {
            s_kick = false;
            drop_client();              /* link lost or stopped, or the client stopped reading */
        }
        fd = s_fd;
        if (ls < 0) {
            ls = listen_socket();
            if (ls < 0) {
                ESP_LOGE(TAG, "listen on port %d failed", NET_SERVER_PORT);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            ESP_LOGI(TAG, "listening on port %d", NET_SERVER_PORT);
        }
        FD_ZERO(&rf);
        FD_SET(ls, &rf);
        mx = ls;
        if (fd >= 0) {
            FD_SET(fd, &rf);
            if (fd > mx)
                mx = fd;
        }
        if (select(mx + 1, &rf, NULL, NULL, &tv) <= 0)
            continue;

        if (FD_ISSET(ls, &rf)) {
            int nfd = accept(ls, NULL, NULL);

            if (nfd >= 0) {
                if (s_fd >= 0) {
                    static const char busy[] = "[MSG:busy: another host is connected]\r\n";

                    send(nfd, busy, sizeof(busy) - 1, MSG_DONTWAIT);
                    close(nfd);
                } else {
                    client_options(nfd);
                    xSemaphoreTake(s_fd_mutex, portMAX_DELAY);
                    s_tail = s_head;
                    s_overflow_since = 0;
                    s_fd = nfd;
                    xSemaphoreGive(s_fd_mutex);
                    host_link_net_up(net_write);
                    fd = nfd;
                    ESP_LOGI(TAG, "client connected");
                }
            }
        }
        if (fd >= 0 && s_fd == fd && FD_ISSET(fd, &rf)) {
            int n = recv(fd, buf, sizeof(buf), 0);

            if (n > 0) {
                host_link_net_rx(buf, n);
            } else if (n == 0) {
                s_peer_closed++;
                drop_client();
            } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                s_recv_err++;
                s_last_errno = errno;
                drop_client();
            }
        }
    }
}

bool net_server_start(void)
{
    if (s_started)
        return true;
    s_fd_mutex = xSemaphoreCreateMutex();
    if (!s_fd_mutex)
        return false;
    if (xTaskCreatePinnedToCore(tx_task, "net_tx", TX_TASK_STACK, NULL, TASK_PRIO,
                                &s_tx_task, 1) != pdPASS)
        return false;
    if (xTaskCreatePinnedToCore(server_task, "net_srv", SRV_TASK_STACK, NULL,
                                TASK_PRIO, NULL, 1) != pdPASS)
        return false;
    s_started = true;
    return true;
}

void net_server_drop_client(void)
{
    if (s_fd >= 0)
        s_kicks_link++;
    s_kick = true;
}

bool net_server_has_client(void)
{
    return s_fd >= 0;
}

void net_server_stats(char *buf, size_t n)
{
    snprintf(buf, n,
             "[MSG:net client=%d kicks tx=%u link=%u peer_closed=%u recv_err=%u errno=%d ring_max=%u/%u"
             " sends=%u sent=%uB heap_free=%u heap_min=%u]\r\n",
             s_fd >= 0, (unsigned)s_kicks_tx, (unsigned)s_kicks_link, (unsigned)s_peer_closed,
             (unsigned)s_recv_err, s_last_errno, (unsigned)s_ring_max, (unsigned)TX_RING,
             (unsigned)s_sends, (unsigned)s_sent_bytes,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
}
