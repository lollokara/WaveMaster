#ifndef ZBRO_LINK_H_
#define ZBRO_LINK_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Z-Bro client (docs/ZBRO_PLAN.md): talks to the ESP32-C3 Z axis over WiFi.
 *
 * One task on core 1 (prio 4) owns the socket. It listens for the UDP :7701
 * beacon, connects TCP :7700, sends HELLO, PINGs once a second and reconnects
 * with backoff. Only active while the WiFi station is up.
 *
 * The blocking calls below are for the grbl task only (they sleep on
 * semaphores). Never call them from dac_task, the motion task or an ISR.
 * zbro_online(), zbro_pos(), zbro_info() and zbro_stop() never block and are
 * safe from the RX / status context.
 *
 * Return values: 0 = ok, 1..8 = error code reported by the C3 (ZBRO_E_* in
 * shared/zbro_proto.h), or one of the ZBRO_ERR_* values below.
 */
#define ZBRO_ERR_OFFLINE 100    /* no link to the C3 */
#define ZBRO_ERR_TIMEOUT 101    /* no answer / move did not finish in time */
#define ZBRO_ERR_ABORTED 102    /* reset, hold or STOP while the call was waiting */
#define ZBRO_ERR_LINK    103    /* link dropped during the call */

struct zbro_info {
    bool online;
    char ip[16];
    char fw[24];
    float pos;
    int state;                  /* 0..4 as zbro_state_name(), -1 unknown */
    bool homed, endstop;
};

bool zbro_link_start(void);
bool zbro_online(void);
float zbro_pos(void);                         /* last reported position, mm */
void zbro_get_info(struct zbro_info *out);

/* The callback tells the blocking calls to give up (grbl: reset generation changed). */
void zbro_set_abort_fn(bool (*fn)(void));

int zbro_home(int timeout_s);                 /* waits for EVT HOMED */
int zbro_move(float mm, float speed, int timeout_s);   /* absolute; speed <= 0 = C3 default; waits for EVT DONE */
void zbro_stop(void);                         /* non-blocking: STOP if a move/home is in progress */

/* Sends one request line (no newline) and returns the C3's reply line in reply. 0 unless the C3 answered ERR. */
int zbro_request(const char *line, char *reply, size_t n, uint32_t timeout_ms);

#endif /* ZBRO_LINK_H_ */
