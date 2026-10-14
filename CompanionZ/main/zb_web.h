#ifndef ZB_WEB_H_
#define ZB_WEB_H_

#include "zb_motion.h"

/* HTTP :80 (LittleFS UI, REST, /ws). Call after zb_net_start(). */
void zb_web_start(void);
/* Motion event / calibration change hooks: push to WebSocket clients (any task). */
void zb_web_on_event(enum zb_evt_type type, float pos_mm, int err);
void zb_web_on_cal(void);

#endif
