/* Host-side unit tests for src/motion.c (trajectory generator). */
#include "motion.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct pt { float x, y; uint8_t p; };
static struct pt *cap;
static size_t ncap, capcap;

static void sink_tick(void *ctx, float x, float y, uint8_t p)
{
    (void)ctx;
    if (ncap == capcap) {
        capcap = capcap ? capcap * 2 : 4096;
        cap = realloc(cap, capcap * sizeof(*cap));
    }
    cap[ncap].x = x;
    cap[ncap].y = y;
    cap[ncap].p = p;
    ncap++;
}

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; \
    printf("  FAIL line %d: %s -- ", __LINE__, #c); printf(__VA_ARGS__); printf("\n"); } } while (0)

static struct motion_gen G;
static struct motion_params P;

static void setup(void)
{
    memset(&P, 0, sizeof(P));
    P.tick_us = 10.0f;
    P.jump_speed_mm_s = 3000.0f;
    P.jump_accel_mm_s2 = 2000000.0f;
    P.jump_delay_us = 300.0f;
    P.mark_delay_us = 100.0f;
    P.laser_off_delay_us = 120.0f;
    P.polygon_angle_deg = 30.0f;
    P.wobble_pitch_mm = 0.5f;
    struct motion_sink s = { sink_tick, NULL };
    ncap = 0;
    motion_gen_init(&G, &P, &s, 0, 0);
}

static struct motion_seg mark(float x, float y, float v, uint8_t pw)
{
    struct motion_seg s;
    memset(&s, 0, sizeof(s));
    s.type = MOTION_MARK; s.power = pw; s.x = x; s.y = y; s.speed_mm_s = v;
    return s;
}
static struct motion_seg jump(float x, float y)
{
    struct motion_seg s;
    memset(&s, 0, sizeof(s));
    s.type = MOTION_JUMP; s.x = x; s.y = y;
    return s;
}
static struct motion_seg dwell(float us)
{
    struct motion_seg s;
    memset(&s, 0, sizeof(s));
    s.type = MOTION_DWELL; s.dwell_us = us;
    return s;
}
static void push(struct motion_seg s) { motion_gen_push(&G, &s); }

static float dist(size_t i, size_t j)
{ return hypotf(cap[j].x - cap[i].x, cap[j].y - cap[i].y); }

/* length of trailing run of power==0 ticks */
static size_t trailing_zero(void)
{ size_t n = 0; while (n < ncap && cap[ncap - 1 - n].p == 0) n++; return n; }

/* trailing ticks at the same position as the last tick, minus the landing tick */
static size_t trailing_same(void)
{
    size_t n = 1;
    while (n < ncap && cap[ncap - 1 - n].x == cap[ncap - 1].x && cap[ncap - 1 - n].y == cap[ncap - 1].y) n++;
    return n - 1;
}

static size_t count_at(float x, float y)
{
    size_t n = 0;
    for (size_t i = 0; i < ncap; i++)
        if (cap[i].x == x && cap[i].y == y) n++;
    return n;
}

static void test_single_mark(void)
{
    printf("test 1: single mark\n");
    setup();
    float v = 500.0f, L = 10.0f;
    push(mark(L, 0, v, 200));
    CHECK(ncap == 0, "nothing emitted before successor (%zu)", ncap);
    motion_gen_finish(&G);
    size_t hold = (size_t)lroundf(fmaxf(P.mark_delay_us, P.laser_off_delay_us) / P.tick_us);
    CHECK(trailing_zero() == hold, "hold %zu expected %zu", trailing_zero(), hold);
    size_t nm = ncap - hold;
    CHECK(cap[nm - 1].x == L && cap[nm - 1].y == 0, "last mark tick exactly at end");
    for (size_t i = 0; i < hold; i++)
        CHECK(cap[nm + i].x == L && cap[nm + i].p == 0, "hold tick");
    float step = v * P.tick_us * 1e-6f;
    for (size_t i = 1; i + 1 < nm; i++) {
        float d = dist(i - 1, i);
        CHECK(fabsf(d - step) < 1e-4f, "step %zu = %g", i, d);
        CHECK(cap[i].p == 200, "power");
    }
    CHECK(dist(nm - 2, nm - 1) <= step + 1e-4f, "last step <= step");
    float T = (float)nm * P.tick_us * 1e-6f;
    CHECK(fabsf(T - L / v) <= P.tick_us * 1e-6f * 1.01f, "time %g vs %g", T, L / v);
    CHECK(G.x == L, "gen position");
    CHECK(!motion_gen_has_pending(&G), "no pending");
}

static void test_circle(void)
{
    printf("test 2: linearised circle (1000 x 0.003 mm)\n");
    setup();
    const int N = 1000;
    float chord = 0.003f, R = chord * N / (2.0f * (float)M_PI);
    float v = 500.0f, step = v * P.tick_us * 1e-6f;   /* 0.005 */
    double ang = 2.0 * M_PI / N;
    push(jump(R, 0));
    for (int i = 1; i <= N; i++)
        push(mark(R * cosf((float)(ang * i)), R * sinf((float)(ang * i)), v, 255));
    motion_gen_finish(&G);
    size_t start = 0;
    while (cap[start].p == 0) start++;
    size_t end = ncap - trailing_zero();
    float maxd = 0, mind = 1e9f;
    for (size_t i = start + 1; i + 1 < end; i++) {
        float d = dist(i - 1, i);
        if (d > maxd) maxd = d;
        if (d < mind) mind = d;
    }
    /* a step straddling a vertex is a slightly shortened chord: allow 2% */
    CHECK(maxd <= step * 1.0001f && mind >= step * 0.98f, "step range %g..%g (want %g)", mind, maxd, step);
    float total = (float)N * chord;
    long expect = (long)(total / step);
    CHECK(labs((long)(end - start) - expect) <= 3, "count %zu vs %ld", end - start, expect);
    CHECK(fabsf(cap[end - 1].x - R) < 1e-4f && fabsf(cap[end - 1].y) < 1e-4f, "closed at end");
    printf("  step range %g..%g, %zu mark ticks\n", mind, maxd, end - start);
}

static void test_speed_change(void)
{
    printf("test 3: speed change between marks\n");
    setup();
    push(mark(1.0f, 0, 100.0f, 255));      /* step 0.001 */
    push(mark(2.0f, 0, 400.0f, 255));      /* step 0.004 */
    push(mark(3.0f, 0, 200.0f, 255));      /* step 0.002 */
    motion_gen_finish(&G);
    size_t end = ncap - trailing_zero();
    for (size_t i = 1; i + 1 < end; i++) {
        float d = dist(i - 1, i), x = cap[i].x;
        float expect = x <= 1.0f ? 0.001f : (x <= 2.0f ? 0.004f : 0.002f);
        if (fabsf(x - 1.0f) < 0.0045f || fabsf(x - 2.0f) < 0.0045f)
            CHECK(d <= 0.004f + 1e-4f && d >= 0.001f - 1e-4f, "near vertex step %g x=%g", d, x);
        else
            CHECK(fabsf(d - expect) < 1e-4f, "step %g expected %g at x=%g", d, expect, x);
    }
    /* time-based carry: total time = sum of segment times (within a tick) */
    float T = (float)end * 10e-6f, Te = 1.0f / 100 + 1.0f / 400 + 1.0f / 200;
    CHECK(fabsf(T - Te) < 3e-5f, "time %g vs %g", T, Te);
    CHECK(cap[end - 1].x == 3.0f, "end exact");
    for (size_t i = 1; i < end; i++) CHECK(cap[i].x > cap[i - 1].x, "monotonic at %zu", i);
}

static void test_jump(void)
{
    printf("test 4: jump profiles / chaining\n");
    for (int mode = 0; mode < 3; mode++) {
        setup();
        P.jump_delay_per_mm_us = 10.0f;
        float L = mode == 0 ? 50.0f : (mode == 1 ? 0.5f : 20.0f);
        if (mode == 2) P.jump_accel_mm_s2 = 0;
        motion_gen_set_params(&G, &P);
        push(jump(L, 0));
        motion_gen_finish(&G);
        size_t hold = (size_t)lroundf((300.0f + 10.0f * L) / 10.0f);
        CHECK(trailing_same() == hold, "delay %zu vs %zu", trailing_same(), hold);
        size_t nj = ncap - hold;
        CHECK(cap[nj - 1].x == L && cap[nj - 1].y == 0, "lands exactly mode %d", mode);
        float dt = 10e-6f, vmax = 0, amax = 0, vprev = 0;
        for (size_t i = 0; i < nj; i++) {
            float x0 = i ? cap[i - 1].x : 0;
            float v = (cap[i].x - x0) / dt;
            if (v > vmax) vmax = v;
            float a = fabsf(v - vprev) / dt;
            if (i > 0 && a > amax) amax = a;
            vprev = v;
            CHECK(cap[i].p == 0, "power 0");
            CHECK(cap[i].x >= x0, "monotonic");
        }
        float vlast = (cap[nj - 1].x - cap[nj - 2].x) / dt;
        float v0 = cap[0].x / dt;
        CHECK(vmax <= P.jump_speed_mm_s * 1.001f, "vmax %g", vmax);
        if (mode != 2) {
            /* 10%: float quantisation of x at 50 mm adds ~4% noise to the 2nd difference */
            CHECK(amax <= P.jump_accel_mm_s2 * 1.10f, "amax %g", amax);
            CHECK(vlast < 2.0f * P.jump_accel_mm_s2 * dt && v0 < 2.0f * P.jump_accel_mm_s2 * dt,
                  "rest at start/end: %g %g", v0, vlast);
        }
        if (mode == 0) CHECK(vmax > 0.99f * P.jump_speed_mm_s, "cruise reached %g", vmax);
        if (mode == 1) CHECK(vmax < P.jump_speed_mm_s, "triangle peak %g", vmax);
        printf("  mode %d: L=%g ticks=%zu vmax=%g amax=%g\n", mode, L, nj, vmax, amax);
    }
    /* chaining: jump,jump,jump -> only the final one gets the delay */
    setup();
    push(jump(1, 0)); push(jump(1, 1)); push(jump(0, 1));
    motion_gen_finish(&G);
    CHECK(trailing_same() == 30, "final delay %zu", trailing_same());
    size_t run = 0, maxrun = 0;
    for (size_t i = 1; i + 30 < ncap; i++) {
        if (cap[i].x == cap[i - 1].x && cap[i].y == cap[i - 1].y) run++; else run = 0;
        if (run > maxrun) maxrun = run;
    }
    CHECK(maxrun < 3, "no delay between chained jumps (run %zu)", maxrun);
    CHECK(cap[ncap - 1].x == 0 && cap[ncap - 1].y == 1, "end");
    /* jump then mark: delay in between */
    setup();
    push(jump(1, 0)); push(mark(2, 0, 1000, 100));
    motion_gen_finish(&G);
    size_t i = 0;
    while (!(cap[i].x == 1.0f && cap[i].y == 0)) i++;
    size_t j = i;
    while (cap[j].x == 1.0f) j++;
    CHECK(j - i == 30 + 1, "jump delay before mark: %zu", j - i);   /* landing tick + 30 */
    CHECK(cap[j].p == 100, "mark follows delay");
    /* zero-length jump */
    setup();
    push(jump(0, 0)); motion_gen_finish(&G);
    CHECK(ncap == 0, "zero jump emits nothing");
}

static void test_zero_power(void)
{
    printf("test 5: power-0 mark inside a polyline\n");
    setup();
    push(mark(1, 0, 500, 255));
    push(mark(2, 0, 500, 0));
    push(mark(3, 0, 500, 255));
    motion_gen_finish(&G);
    size_t end = ncap - trailing_zero();
    size_t nz = 0;
    for (size_t i = 1; i + 1 < end; i++) {
        CHECK(fabsf(dist(i - 1, i) - 0.005f) < 1e-4f, "constant step %g at %zu", dist(i - 1, i), i);
        if (cap[i].x > 1.0001f && cap[i].x < 1.9999f) { CHECK(cap[i].p == 0, "p0 mid"); nz++; }
        if (cap[i].x < 0.9999f || cap[i].x > 2.0001f) CHECK(cap[i].p == 255, "p255");
    }
    CHECK(nz > 150, "zero-power ticks %zu", nz);
}

static void test_polygon(void)
{
    printf("test 6: polygon delay\n");
    setup();
    P.polygon_delay_us = 400; P.polygon_angle_deg = 30;
    motion_gen_set_params(&G, &P);
    push(mark(1, 0, 500, 180)); push(mark(1, 1, 500, 180));
    motion_gen_finish(&G);
    size_t at = count_at(1.0f, 0.0f);
    /* 90 deg -> 200 us = 20 hold ticks, plus the snapped vertex tick itself */
    CHECK(at == 21, "ticks at 90deg vertex: %zu (want 21)", at);
    for (size_t i = 0; i < ncap; i++)
        if (cap[i].x == 1.0f && cap[i].y == 0.0f) CHECK(cap[i].p == 180, "hold keeps power");

    setup();
    P.polygon_delay_us = 400; P.polygon_angle_deg = 30;
    motion_gen_set_params(&G, &P);
    float a = 10.0f * (float)M_PI / 180.0f;
    push(mark(1, 0, 500, 180)); push(mark(1 + cosf(a), sinf(a), 500, 180));
    motion_gen_finish(&G);
    CHECK(count_at(1.0f, 0.0f) <= 1, "no hold at 10deg (%zu)", count_at(1.0f, 0.0f));
    size_t end = ncap - trailing_zero();
    for (size_t i = 1; i + 1 < end; i++)
        CHECK(fabsf(dist(i - 1, i) - 0.005f) < 1e-4f, "10deg continuous step %g", dist(i - 1, i));

    setup();
    P.polygon_delay_us = 400; motion_gen_set_params(&G, &P);
    push(mark(1, 0, 500, 90)); push(mark(0, 0, 500, 90));
    motion_gen_finish(&G);
    CHECK(count_at(1.0f, 0.0f) == 41, "reversal hold %zu (want 41)", count_at(1.0f, 0.0f));
}

static void test_dwell(void)
{
    printf("test 7: dwell\n");
    setup();
    push(jump(1, 2)); push(dwell(250)); push(jump(3, 3));
    motion_gen_finish(&G);
    size_t i = 0;
    while (!(cap[i].x == 1.0f && cap[i].y == 2.0f)) i++;
    size_t n = 0;
    while (cap[i + n].x == 1.0f && cap[i + n].y == 2.0f) { CHECK(cap[i + n].p == 0, "p0"); n++; }
    CHECK(n == 1 + 30 + 25, "landing + jump delay + dwell ticks: %zu", n);
    setup();
    push(dwell(100)); motion_gen_finish(&G);
    CHECK(ncap == 10, "lone dwell %zu", ncap);
    setup();
    push(mark(0.5f, 0, 500, 255)); push(dwell(100)); motion_gen_finish(&G);
    CHECK(ncap == 100 + 12 + 10, "mark+hold+dwell %zu", ncap);
}

static void test_finish_reset(void)
{
    printf("test 8: finish / reset / misc\n");
    setup();
    push(mark(0.1f, 0, 500, 255));
    CHECK(motion_gen_has_pending(&G), "pending");
    motion_gen_finish(&G);
    CHECK(!motion_gen_has_pending(&G), "not pending after finish");
    size_t n1 = ncap;
    push(mark(0.2f, 0, 500, 255));          /* fresh polyline from (0.1,0) */
    motion_gen_finish(&G);
    CHECK(fabsf(cap[n1].x - 0.1f - 0.005f) < 1e-5f, "fresh start, first step %g", cap[n1].x - 0.1f);
    CHECK(cap[ncap - 1].x == 0.2f, "second end");

    setup();
    push(mark(5, 5, 500, 255));
    motion_gen_reset(&G, 1, 1);
    CHECK(!motion_gen_has_pending(&G), "reset clears pending");
    motion_gen_finish(&G);
    CHECK(ncap == 0, "nothing emitted after reset");
    push(jump(2, 1)); motion_gen_finish(&G);
    CHECK(cap[0].x > 1.0f && cap[0].x < 2.0f && cap[0].y == 1.0f, "jump starts at reset pos");

    setup();
    push(mark(1, 0, 500, 255)); push(mark(1, 0, 500, 7)); push(mark(1, 0, 500, 255));
    push(mark(2, 0, 500, 255));
    motion_gen_finish(&G);
    size_t end = ncap - trailing_zero();
    for (size_t i = 1; i + 1 < end; i++)
        CHECK(fabsf(dist(i - 1, i) - 0.005f) < 1e-4f, "step with zero-length segs");

    setup();
    P.wobble_diameter_mm = 0.1f; P.wobble_pitch_mm = 0.2f; motion_gen_set_params(&G, &P);
    push(mark(2, 0, 500, 255)); motion_gen_finish(&G);
    end = ncap - trailing_zero();
    float maxy = 0;
    for (size_t i = 0; i < end; i++) if (fabsf(cap[i].y) > maxy) maxy = fabsf(cap[i].y);
    CHECK(maxy > 0.045f && maxy <= 0.0501f, "wobble amplitude %g", maxy);
    CHECK(cap[end - 1].x == 2.0f && cap[end - 1].y == 0, "wobble ends on path");
    CHECK(fabsf(cap[0].y) < 0.01f, "wobble starts near path");

    setup();
    P.tick_us = 7.3f; motion_gen_set_params(&G, &P);
    push(mark(30, 40, 321, 255)); push(mark(-10, 5, 321, 255)); motion_gen_finish(&G);
    end = ncap - trailing_zero();
    float step = 321 * 7.3e-6f;
    for (size_t i = 1; i + 1 < end; i++) {
        float d = dist(i - 1, i);
        /* the corner at (30,40) is cut by the polyline: chord < step across it */
        int corner = fabsf(cap[i].x - 30) < 0.01f && fabsf(cap[i].y - 40) < 0.01f;
        CHECK(d <= step * 1.0001f + 1e-4f && (d >= step * 0.99f || corner), "diag step %g", d);
    }
}

int main(void)
{
    test_single_mark();
    test_circle();
    test_speed_change();
    test_jump();
    test_zero_power();
    test_polygon();
    test_dwell();
    test_finish_reset();
    printf("\n%d checks, %d failures\n", checks, fails);
    free(cap);
    return fails ? 1 : 0;
}
