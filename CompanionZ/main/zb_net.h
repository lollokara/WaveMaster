#ifndef ZB_NET_H_
#define ZB_NET_H_

#include <stdbool.h>
#include <stddef.h>

#include "zb_motion.h"

#define ZB_FW_VERSION "1.0.0"

/* WiFi (STA with SoftAP provisioning fallback), mDNS, COM link TCP :7700, UDP beacon :7701. */
void zb_net_start(void);

/* Motion event hook: forward to the connected S3 (called from the motion task). */
void zb_net_on_event(enum zb_evt_type type, float pos_mm, int err);
/* Calibration state changed: forward "EVT CAL <json>" to the S3 (called from the cal task). */
void zb_net_on_cal(void);

bool zb_net_s3_connected(void);
bool zb_net_ap_active(void);
bool zb_net_sta_connected(void);
int  zb_net_rssi(void);                          /* dBm, 0 when not associated */
void zb_net_ip(char *buf, size_t n);             /* STA IP, or the AP IP when only the AP runs */

void zb_net_wifi_ssid(char *buf, size_t n);      /* stored SSID ("" when none) */
/* Store credentials (pass may be ""). 0 ok, 4 bad argument. Takes effect after reboot. */
int  zb_net_wifi_set(const char *ssid, const char *pass);
void zb_net_reboot_later(unsigned ms);
const char *zb_err_text(int code);              /* protocol error code -> short text */

#endif
