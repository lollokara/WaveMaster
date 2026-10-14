#ifndef ZB_FLEET_H_
#define ZB_FLEET_H_

#include <stdbool.h>
#include <stddef.h>

#include "zb_motion.h"

/* ESPFleet device agent glue (docs/ZBRO_PLAN.md section 9). Everything here is non-blocking for the
 * callers: the motion task and the cal task only bump counters and post to a queue / task notification;
 * a priority-1 "zb_fleet" task does the metrics, the log lines and the agent start. */

/* Creates the glue task. Returns at once. The agent itself is started by that task once the STA has an IP
 * (never in SoftAP-only provisioning mode) and when the hub setting is not "off". Call after zb_net_start(). */
void zb_fleet_start(void);

/* Hooks, safe from any task (not from an ISR). */
void zb_fleet_on_event(enum zb_evt_type type, float pos_mm, int err);   /* motion event, from the motion task */
void zb_fleet_on_cal(void);                                             /* calibration state changed */
void zb_fleet_note_estop(void);                                         /* an E-stop was requested */

/* Status for /api/status. `hub` gets the effective hub URL ("" when not resolved, "off" when disabled). */
struct zb_fleet_status {
    bool enabled;          /* hub setting is not "off" */
    bool started;          /* agent running */
    bool registered;       /* hub accepted at least one heartbeat and is currently reachable */
    int  last_hb_age_s;    /* seconds since the last accepted heartbeat, -1 = never */
    unsigned heartbeats;
};
void zb_fleet_status(struct zb_fleet_status *st, char *hub, size_t hub_n);

#endif
