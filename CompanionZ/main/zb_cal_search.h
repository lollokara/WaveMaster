#ifndef ZB_CAL_SEARCH_H_
#define ZB_CAL_SEARCH_H_

/* Pure sweep + bisection used by calibration (docs/ZBRO_PLAN.md section 5). No ESP-IDF dependencies.
 * Geometric sweep upward from `start` (x factor) until a trial fails or `max` passes, then bisects
 * between the last pass and the first fail. */

#define ZBS_MAX_TRIALS 40

/* Trial callback: 1 = pass, 0 = fail, negative = abort the search. */
typedef int (*zbs_trial_fn)(void *ctx, float value);

enum zbs_status {
    ZBS_OK,          /* limit found: highest passing value (to bisection resolution) */
    ZBS_CAPPED,      /* `max` passed without failure; limit = max */
    ZBS_FAIL_START,  /* even `start` failed (or bad config); limit = 0 */
    ZBS_ABORTED
};

struct zbs_cfg { float start, factor, max; int bisections; };
struct zbs_result { enum zbs_status status; float limit; int trials; };

struct zbs_result zbs_search(const struct zbs_cfg *cfg, zbs_trial_fn fn, void *ctx);
/* Upper bound on the number of trials for progress estimation. */
int zbs_est_trials(const struct zbs_cfg *cfg);

#endif
