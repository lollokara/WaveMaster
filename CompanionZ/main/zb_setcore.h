#ifndef ZB_SETCORE_H_
#define ZB_SETCORE_H_

#include <stddef.h>
#include "zb_settings.h"

/* Pure settings table, validation and JSON export (no ESP-IDF dependencies, host tested). */

enum zbc_type { ZBC_NUM, ZBC_BOOL, ZBC_ENUM };

struct zbc_def {
    const char *key, *label, *group, *unit;
    float def, min, max, step;
    enum zbc_type type;
    const float *opts;   /* ZBC_ENUM only */
    int n_opts;
};

extern const struct zbc_def zbc_defs[ZS_COUNT];

/* Effective range: max is lowered to the calibrated limit when one exists (lim_* > 0). */
void zbc_range(const float *vals, enum zb_set_id id, float *lo, float *hi);
/* 0 ok, 4 bad argument. */
int  zbc_validate(const float *vals, enum zb_set_id id, float v);
/* snprintf-style: returns the length the JSON needs (excluding NUL), writes at most n bytes. */
size_t zbc_json(const float *vals, char *buf, size_t n);
/* Hostname: 1..31 chars of [A-Za-z0-9-], no leading/trailing '-'. */
int  zbc_hostname_valid(const char *name);

#endif
