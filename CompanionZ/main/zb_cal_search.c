#include "zb_cal_search.h"

int zbs_est_trials(const struct zbs_cfg *cfg)
{
    int n = 1;
    float v = cfg->start;
    if (cfg->start <= 0.0f || cfg->factor <= 1.0f) return 1;
    while (v < cfg->max && n < ZBS_MAX_TRIALS) { v *= cfg->factor; n++; }
    return n + cfg->bisections;
}

struct zbs_result zbs_search(const struct zbs_cfg *cfg, zbs_trial_fn fn, void *ctx)
{
    struct zbs_result r = { ZBS_FAIL_START, 0.0f, 0 };
    float max = cfg->max, lo = 0.0f, hi, v;
    int t;

    if (cfg->start <= 0.0f || cfg->factor <= 1.0f) return r;
    if (max < cfg->start) max = cfg->start;
    v = cfg->start;
    for (;;) {
        t = fn(ctx, v); r.trials++;
        if (t < 0) { r.status = ZBS_ABORTED; r.limit = lo; return r; }
        if (!t) break;
        lo = v;
        if (v >= max) { r.status = ZBS_CAPPED; r.limit = max; return r; }
        v *= cfg->factor;
        if (v > max) v = max;
        if (r.trials >= ZBS_MAX_TRIALS) { r.status = ZBS_OK; r.limit = lo; return r; }
    }
    if (lo <= 0.0f) return r;                 /* failed at start */
    hi = v;
    for (int i = 0; i < cfg->bisections; i++) {
        float mid = 0.5f * (lo + hi);
        t = fn(ctx, mid); r.trials++;
        if (t < 0) { r.status = ZBS_ABORTED; r.limit = lo; return r; }
        if (t) lo = mid; else hi = mid;
    }
    r.status = ZBS_OK;
    r.limit = lo;
    return r;
}
