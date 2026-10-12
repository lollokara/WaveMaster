#ifndef NET_SERVER_H_
#define NET_SERVER_H_

#include <stdbool.h>

/*
 * Raw TCP host link on port 23 (Rayforge: "GRBL (Telnet)"). One client at a
 * time; a second connection is answered with a [MSG:busy] line and closed.
 * Bytes go to host_serial.c as the second transport, so the GRBL protocol is
 * exactly the one spoken over USB.
 *
 * The client socket has TCP_NODELAY and keepalive (about 3 s idle, 1 s
 * interval, 3 probes), so a vanished client is noticed in about 6 s and the
 * running job is aborted (host_link_net_down).
 */

#define NET_SERVER_PORT 23

/* Starts the server task on core 1. Idempotent. */
bool net_server_start(void);

/* True while a client is connected. */
bool net_server_has_client(void);

#endif /* NET_SERVER_H_ */
