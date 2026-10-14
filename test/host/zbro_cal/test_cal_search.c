/* Host test for CompanionZ/main/zb_cal_search.c against a simulated axis that loses steps above a threshold. */
#include <math.h>
#include <stdio.h>
#include "zb_cal_search.h"

static int fails;
#define CHECK(c) do { if (!(c)) { fails++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

struct sim { float thr; int calls; int abort_at; float last; };

static int sim_trial(void *ctx, float v)
{
    struct sim *s = ctx;
    s->calls++; s->last = v;
    if (s->abort_at && s->calls >= s->abort_at) return -1;
    return v <= s->thr;   /* steps are lost above the threshold */
}

int main(void)
{
    struct zbs_cfg c = { 5.0f, 1.5f, 200.0f, 3 };
    float thrs[] = { 5.0f, 7.4f, 12.0f, 33.3f, 80.0f, 199.0f };

    for (unsigned i = 0; i < sizeof thrs / sizeof *thrs; i++) {
        struct sim s = { thrs[i], 0, 0, 0 };
        struct zbs_result r = zbs_search(&c, sim_trial, &s);
        CHECK(r.status == ZBS_OK);
        CHECK(r.limit <= thrs[i]);                       /* never reports an unsafe value */
        CHECK(r.limit >= thrs[i] / 1.5f - 1e-3f);        /* within one geometric step */
        CHECK(thrs[i] - r.limit <= thrs[i] * 0.5f / 8 + 1e-3f); /* bisection resolution (range/8) */
        CHECK(r.trials == s.calls);
        CHECK(r.trials <= zbs_est_trials(&c));
    }
    { /* passes everything: capped at max, never tested above max */
        struct sim s = { 1e9f, 0, 0, 0 };
        float mx = 0;
        struct zbs_result r = zbs_search(&c, sim_trial, &s);
        mx = s.last;
        CHECK(r.status == ZBS_CAPPED && r.limit == 200.0f && mx == 200.0f);
    }
    { /* fails at start */
        struct sim s = { 1.0f, 0, 0, 0 };
        struct zbs_result r = zbs_search(&c, sim_trial, &s);
        CHECK(r.status == ZBS_FAIL_START && r.limit == 0.0f && r.trials == 1);
    }
    { /* abort mid-sweep and mid-bisection */
        struct sim s = { 50.0f, 0, 3, 0 };
        struct zbs_result r = zbs_search(&c, sim_trial, &s);
        CHECK(r.status == ZBS_ABORTED && s.calls == 3);
        struct sim s2 = { 50.0f, 0, 8, 0 };
        r = zbs_search(&c, sim_trial, &s2);
        CHECK(r.status == ZBS_ABORTED && s2.calls == 8);
    }
    { /* start above max is clamped; bad config rejected */
        struct zbs_cfg b = { 50.0f, 1.5f, 10.0f, 3 };
        struct sim s = { 1e9f, 0, 0, 0 };
        struct zbs_result r = zbs_search(&b, sim_trial, &s);
        CHECK(r.status == ZBS_CAPPED && r.limit == 50.0f);
        struct zbs_cfg g = { 0.0f, 1.5f, 10.0f, 3 };
        r = zbs_search(&g, sim_trial, &s);
        CHECK(r.status == ZBS_FAIL_START && r.trials == 0);
    }
    if (fails) { printf("%d failures\n", fails); return 1; }
    printf("zbro cal search tests passed\n");
    return 0;
}
