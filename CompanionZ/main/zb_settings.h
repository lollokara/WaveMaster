#ifndef ZB_SETTINGS_H_
#define ZB_SETTINGS_H_

#include <stdbool.h>
#include <stddef.h>

/* Z-Bro settings: NVS namespace "zbro", loaded at boot, validated on write (docs/ZBRO_PLAN.md §4).
 * Table-driven: one row per key (key, label, group, unit, default, min, max, step, type). Reads are
 * cheap array lookups. Thread-safe (a mutex guards writes). */

enum zb_set_id {
    ZS_STEPS_REV, ZS_LEAD_MM, ZS_MICROSTEP, ZS_DIR_INVERT, ZS_ES_INVERT, ZS_HOME_TO_MAX,
    ZS_MAX_RANGE, ZS_MAX_SPEED, ZS_ACCEL, ZS_DECEL, ZS_HOME_FAST, ZS_HOME_SLOW, ZS_HOME_BACKOFF,
    ZS_ES_DEBOUNCE_US, ZS_PULSE_US, ZS_IDLE_OFF_S, ZS_SLEEP_IDLE, ZS_RPT_HZ, ZS_ALLOW_UNHOMED,
    /* calibration results */
    ZS_LIM_SPEED, ZS_LIM_ACCEL, ZS_LIM_DECEL, ZS_LIM_HOME_FAST, ZS_ES_REPEAT_UM, ZS_CAL_TIME,
    ZS_COUNT
};

void  zb_settings_init(void);                     /* nvs_flash_init + load */
float zb_set_get(enum zb_set_id id);
/* 0 ok; 4 bad argument (unknown key / out of range / above calibrated limit). */
int   zb_set_set(enum zb_set_id id, float v, bool persist);
int   zb_set_set_by_key(const char *key, float v, bool persist);
int   zb_set_find(const char *key);               /* id or -1 */
const char *zb_set_key(enum zb_set_id id);
bool  zb_set_get_by_key(const char *key, float *v);
void  zb_set_save(void);                          /* commit pending writes */
/* Effective range now (max may be capped by a calibrated limit). */
void  zb_set_range(enum zb_set_id id, float *lo, float *hi);
/* JSON for GET /api/settings (grouped, see plan §6). Returns malloc'd string, caller frees. */
char *zb_set_to_json(void);
unsigned zb_set_generation(void);                 /* bumped on every change */

/* Hostname (string, own NVS key). */
void zb_hostname_get(char *buf, size_t n);
int  zb_hostname_set(const char *name);

/* Derived: steps per mm = steps_rev * microstep / lead_mm. */
float zb_steps_per_mm(void);

#endif
