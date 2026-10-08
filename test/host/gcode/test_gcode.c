#include <math.h>
#include <stdio.h>
#include <string.h>

#include "gcode_parse.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)
#define NEAR(a, b, t) CHECK(fabs((double)(a) - (double)(b)) <= (t))

static void test_parse(void)
{
    struct gcode_block b;
    const struct gcode_word *w;

    CHECK(gcode_parse_line("g1 x.5 Y-0 f3000 ; comment", &b) == 0);
    CHECK(b.n == 4);
    w = gcode_find(&b, 'X'); CHECK(w && w->value == 0.5f);
    w = gcode_find(&b, 'F'); CHECK(w && w->value == 3000.0f);
    CHECK(gcode_parse_line("G0 X+3 (inline) Y 4", &b) == 0);
    w = gcode_find(&b, 'X'); CHECK(w && w->value == 3.0f);
    w = gcode_find(&b, 'Y'); CHECK(w && w->value == 4.0f);
    CHECK(gcode_parse_line("N10 G4 P0.01", &b) == 0 && b.n == 2);
    CHECK(gcode_parse_line("", &b) == 0 && b.n == 0);
    CHECK(gcode_parse_line("; only comment", &b) == 0 && b.n == 0);
    CHECK(gcode_parse_line("G1 X", &b) == GCODE_ERR_BAD_NUMBER);
    CHECK(gcode_parse_line("G1 X.", &b) == GCODE_ERR_BAD_NUMBER);
    CHECK(gcode_parse_line("=5", &b) == GCODE_ERR_NO_LETTER);
    CHECK(gcode_parse_line("G1X1Y2", &b) == 0 && b.n == 3);
}

static void test_power(void)
{
    CHECK(gcode_power_byte(0, 1000, 0, 100) == 0);
    CHECK(gcode_power_byte(1000, 1000, 0, 100) == 255);
    CHECK(gcode_power_byte(5000, 1000, 0, 100) == 255);
    CHECK(gcode_power_byte(500, 1000, 0, 100) == 128);
    CHECK(gcode_power_byte(1, 1000, 0, 100) == 1);
    CHECK(gcode_power_byte(1000, 1000, 10, 50) == 128);
    NEAR(gcode_mark_speed(3000, 300000, 500), 50.0, 1e-4);
    NEAR(gcode_mark_speed(0, 300000, 500), 500.0, 1e-4);
    NEAR(gcode_mark_speed(1, 300000, 500), 0.1, 1e-4);
    NEAR(gcode_mark_speed(1e9f, 6000, 500), 100.0, 1e-4);
}

static int run_arc(struct gcode_arc *a, double r, double tol_check, float cx, float cy, int *count)
{
    float x, y;
    double maxdev = 0;

    *count = 0;
    while (gcode_arc_next(a, &x, &y)) {
        double d = hypot(x - cx, y - cy);

        (*count)++;
        if (fabs(d - r) > maxdev)
            maxdev = fabs(d - r);
    }
    return maxdev <= tol_check;
}

static void test_arcs(void)
{
    struct gcode_arc a;
    int n;
    float x, y;

    /* Full circle CCW, radius 10 centred at (10,0) starting at (0,0). */
    CHECK(gcode_arc_init(&a, 0, 0, 0, 0, 10, 0, false, 0.01f) == 0);
    CHECK(a.n > 20);
    NEAR(a.da, 2 * M_PI, 1e-9);
    CHECK(run_arc(&a, 10, 1e-4, 10, 0, &n));
    CHECK(n == a.n);

    /* Last point is exactly the target. */
    CHECK(gcode_arc_init(&a, 10, 0, 0, 10, -10, 0, false, 0.01f) == 0);
    NEAR(a.da, M_PI / 2, 1e-9);
    while (gcode_arc_next(&a, &x, &y)) {}
    CHECK(x == 0.0f && y == 10.0f);

    /* CW quarter: from (10,0) to (0,-10) around origin. */
    CHECK(gcode_arc_init(&a, 10, 0, 0, -10, -10, 0, true, 0.01f) == 0);
    NEAR(a.da, -M_PI / 2, 1e-9);

    /* Chord error stays within tolerance (sagitta of the segment). */
    CHECK(gcode_arc_init(&a, 50, 0, -50, 0, -50, 0, false, 0.005f) == 0);
    {
        double dth = fabs(a.da) / a.n;

        CHECK(50.0 * (1.0 - cos(dth / 2)) <= 0.005 + 1e-9);
    }

    /* Radius mismatch lands exactly on target. */
    CHECK(gcode_arc_init(&a, 10, 0, 0, 10.3f, -10, 0, false, 0.01f) == 0);
    while (gcode_arc_next(&a, &x, &y)) {}
    CHECK(x == 0.0f && y == 10.3f);

    /* R form: semicircle from (0,0) to (10,0), r=5, CW => bulges up (+y). */
    CHECK(gcode_arc_init_r(&a, 0, 0, 10, 0, 5, true, 0.01f) == 0);
    NEAR(a.cx, 5, 1e-6);
    NEAR(a.cy, 0, 1e-6);
    CHECK(gcode_arc_init_r(&a, 0, 0, 10, 0, 5, false, 0.01f) == 0);
    CHECK(gcode_arc_init_r(&a, 0, 0, 10, 0, 4, true, 0.01f) == GCODE_ERR_BAD_TARGET);
    CHECK(gcode_arc_init_r(&a, 0, 0, 0, 0, 4, true, 0.01f) == GCODE_ERR_BAD_TARGET);
    /* 90 degree CCW arc via R: (0,0)->(5,5), r=5 => centre (0,5) (positive R = short way). */
    CHECK(gcode_arc_init_r(&a, 0, 0, 5, 5, 5, false, 0.01f) == 0);
    NEAR(a.cx, 0, 1e-6); NEAR(a.cy, 5, 1e-6);
    NEAR(a.da, M_PI / 2, 1e-6);
    CHECK(gcode_arc_init_r(&a, 0, 0, 5, 5, -5, false, 0.01f) == 0);
    NEAR(fabs(a.da), 3 * M_PI / 2, 1e-6);

    /* Zero radius (I=J=0). */
    CHECK(gcode_arc_init(&a, 1, 1, 2, 2, 0, 0, true, 0.01f) == GCODE_ERR_BAD_TARGET);

    /* Tiny radius vs tolerance still terminates and lands on target. */
    CHECK(gcode_arc_init(&a, 0, 0, 0, 0, 0.001f, 0, false, 0.01f) == 0);
    CHECK(a.n >= 1 && a.n <= 8);
}

int main(void)
{
    test_parse();
    test_power();
    test_arcs();
    if (fails) {
        printf("%d failure(s)\n", fails);
        return 1;
    }
    printf("gcode tests passed\n");
    return 0;
}
