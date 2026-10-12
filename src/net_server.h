#ifndef NET_SERVER_H_
#define NET_SERVER_H_

#include <stdbool.h>
#include <stddef.h>

/*
 * Raw TCP host link on port 23 (Rayforge: "GRBL (Telnet)"). One client at a
 * time; a second connection is answered with a [MSG:busy] line and closed.
 * Bytes go to host_serial.c as the second transport, so the GRBL protocol is
 * exactly the one spoken over USB.
 *
 * The client socket has TCP_NODELAY and keepalive (about 3 s idle, 1 s
 * interval, 3 probes), so a vanished client is noticed in about 6 s and the
 * running job is aborted (host_link_net_down).
 *
 * Replies never touch the network from the caller's task: they are copied into
 * an 8 KB ring and a separate task sends them in batches, so a weak WiFi link
 * cannot stall the GRBL task, and thousands of tiny "ok" lines do not each
 * become a TCP segment. The client is dropped only if the ring stays full or
 * nothing can be sent for a second.
 */

#define NET_SERVER_PORT 23

/* Starts the server task on core 1. Idempotent. */
bool net_server_start(void);

/* Asks the server task to drop the client (aborts like a lost connection). */
void net_server_drop_client(void);

/* One "[MSG:net ...]" line for $S: client state, drop counters and reasons, TX ring
 * high-water mark, internal heap free and minimum free. */
void net_server_stats(char *buf, size_t n);

/* True while a client is connected. */
bool net_server_has_client(void);

#endif /* NET_SERVER_H_ */
