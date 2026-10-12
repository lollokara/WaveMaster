#ifndef NET_WIFI_H_
#define NET_WIFI_H_

#include <stdbool.h>
#include <stddef.h>

/*
 * WiFi station for the network host link (docs/WIFI_PLAN.md).
 *
 * Off unless $240 = 1 and credentials are stored. Everything here runs in the
 * "net_mgr" task on core 1: the WiFi driver is initialised from that task so
 * its interrupt is allocated on core 1, away from dac_task. The station
 * requires WPA2 or better, keeps power save off and reconnects with backoff.
 *
 * Credentials live in their own NVS namespace, set over the host link with
 *   $WIFI=<ssid>,<password>   (first comma splits; password 8..63 characters)
 *   $WIFI=CLEAR               (forget them)
 *   $WIFI                     (status line)
 * The password is never printed or logged.
 */

/* Starts the manager task. Safe to call when WiFi is disabled. */
bool net_wifi_start(void);

/* "$WIFI..." handler. arg points just after "$WIFI" (case preserved). Output
 * lines go through out(). Returns 0 or a GRBL error code. */
int net_wifi_command(const char *arg, void (*out)(const char *line));

/* True while the station has an IP address. */
bool net_wifi_up(void);

#endif /* NET_WIFI_H_ */
