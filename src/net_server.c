#include "net_server.h"

#include <errno.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host_serial.h"
#include "lwip/sockets.h"

#define SRV_TASK_STACK 4096
#define SRV_TASK_PRIO  10               /* same as host_rx: '!' and Ctrl-X latency */
#define RX_CHUNK       128
#define SELECT_MS      100
#define WRITE_CAP_MS   200              /* a WiFi client that does not drain in this time is gone */

static const char *TAG = "net_server";

static volatile int s_fd = -1;
static volatile bool s_started;
static volatile bool s_kick;            /* drop the client from the server task */

/* Full write or 0. Called under the host TX mutex, so writers never overlap;
 * a socket that does not drain within the wait counts as a dead link. */
static int net_write(const char *s, size_t n, TickType_t wait)
{
    int fd = s_fd;
    size_t off = 0;
    TickType_t t0 = xTaskGetTickCount();

    if (fd < 0)
        return 0;
    /* Only a real reply (long wait) that cannot be delivered drops the client;
     * a log line (5 ms wait) that does not fit is just lost. */
    bool reply = wait >= pdMS_TO_TICKS(WRITE_CAP_MS);

    if (wait > pdMS_TO_TICKS(WRITE_CAP_MS))
        wait = pdMS_TO_TICKS(WRITE_CAP_MS);
    while (off < n) {
        int r = send(fd, s + off, n - off, MSG_DONTWAIT);

        if (r > 0) {
            off += (size_t)r;
            continue;
        }
        if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            s_kick = true;
            return 0;
        }
        {
            TickType_t used = xTaskGetTickCount() - t0;
            struct timeval tv;
            fd_set wf;

            if (used >= wait) {
                if (reply)
                    s_kick = true;      /* replies cannot get through: the host would stall silently */
                return 0;
            }
            tv.tv_sec = 0;
            tv.tv_usec = (suseconds_t)(wait - used) * portTICK_PERIOD_MS * 1000;
            FD_ZERO(&wf);
            FD_SET(fd, &wf);
            if (select(fd + 1, NULL, &wf, NULL, &tv) <= 0) {
                if (reply)
                    s_kick = true;
                return 0;
            }
        }
    }
    return (int)n;
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
    host_link_net_down();               /* no writer can use fd after this */
    s_fd = -1;
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
            drop_client();              /* link lost or stopped, or replies cannot be delivered */
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
                    s_fd = nfd;
                    host_link_net_up(net_write);
                    fd = nfd;
                    ESP_LOGI(TAG, "client connected");
                }
            }
        }
        if (fd >= 0 && s_fd == fd && FD_ISSET(fd, &rf)) {
            int n = recv(fd, buf, sizeof(buf), 0);

            if (n > 0)
                host_link_net_rx(buf, n);
            else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK))
                drop_client();
        }
    }
}

bool net_server_start(void)
{
    if (s_started)
        return true;
    if (xTaskCreatePinnedToCore(server_task, "net_srv", SRV_TASK_STACK, NULL,
                                SRV_TASK_PRIO, NULL, 1) != pdPASS)
        return false;
    s_started = true;
    return true;
}

void net_server_drop_client(void)
{
    s_kick = true;
}

bool net_server_has_client(void)
{
    return s_fd >= 0;
}
