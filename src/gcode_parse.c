#include "gcode_parse.h"
#include <math.h>
#include <stddef.h>

#define ARC_MAX_SEGS 65535

int gcode_parse_number(const char **pp, float *out)
{
    const char *p = *pp;
    bool neg = false;
    bool digits = false;
    double v = 0.0;

    if (*p == '+' || *p == '-') {
        neg = (*p == '-');
        p++;
    }
    while (*p >= '0' && *p <= '9') {
        v = v * 10.0 + (double)(*p - '0');
        digits = true;
        p++;
    }
    if (*p == '.') {
        double scale = 0.1;

        p++;
        while (*p >= '0' && *p <= '9') {
            v += (double)(*p - '0') * scale;
            scale *= 0.1;
            digits = true;
            p++;
        }
    }
    if (!digits)
        return GCODE_ERR_BAD_NUMBER;
    *pp = p;
    *out = (float)(neg ? -v : v);
    return GCODE_OK;
}

int gcode_parse_line(const char *line, struct gcode_block *b)
{
    const char *p = line;

    b->n = 0;
    while (*p == ' ' || *p == '\t')
        p++;
    if (*p == '%')
        return GCODE_OK;
    if (*p == '/')
        p++;

    for (;;) {
        char c = *p;
        float v;
        int e;

        if (c == '\0' || c == ';')
            break;
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            p++;
            continue;
        }
        if (c == '(') {
            while (*p && *p != ')')
                p++;
            if (*p == ')')
                p++;
            continue;
        }
        if (c >= 'a' && c <= 'z')
            c = (char)(c - 'a' + 'A');
        if (c < 'A' || c > 'Z')
            return GCODE_ERR_NO_LETTER;
        p++;
        while (*p == ' ' || *p == '\t')
            p++;
        e = gcode_parse_number(&p, &v);
        if (e)
            return e;
        if (c == 'N')
            continue;
        if (b->n >= GCODE_MAX_WORDS)
            return GCODE_ERR_UNSUPPORTED;
        b->w[b->n].letter = c;
        b->w[b->n].value = v;
        b->n++;
    }
    return GCODE_OK;
}

const struct gcode_word *gcode_find(const struct gcode_block *b, char letter)
{
    for (int i = 0; i < b->n; i++)
        if (b->w[i].letter == letter)
            return &b->w[i];
    return NULL;
}

uint8_t gcode_power_byte(float s, float smax, float pmin_pct, float pmax_pct)
{
    float f, pct, v;

    if (!(s > 0.0f))
        return 0;
    f = (smax > 0.0f) ? s / smax : 1.0f;
    if (f > 1.0f)
        f = 1.0f;
    pct = pmin_pct + (pmax_pct - pmin_pct) * f;
    v = roundf(255.0f * pct / 100.0f);
    if (v < 1.0f)
        v = 1.0f;
    if (v > 255.0f)
        v = 255.0f;
    return (uint8_t)v;
}

float gcode_mark_speed(float feed_mm_min, float max_rate_mm_min, float default_mm_s)
{
    float v = (feed_mm_min > 0.0f) ? feed_mm_min / 60.0f : default_mm_s;
    float vmax = max_rate_mm_min / 60.0f;

    if (v > vmax)
        v = vmax;
    if (v < 0.1f)
        v = 0.1f;
    return v;
}

static int arc_setup(struct gcode_arc *a, float x0, float y0, float x1, float y1,
                     double cx, double cy, bool cw, float tol)
{
    const double pi = 3.14159265358979323846;
    double a1, rmax, dth, ratio, nn;
    double sx = (double)x0 - cx, sy = (double)y0 - cy;
    double ex = (double)x1 - cx, ey = (double)y1 - cy;

    a->cx = cx;
    a->cy = cy;
    a->r0 = sqrt(sx * sx + sy * sy);
    a->r1 = sqrt(ex * ex + ey * ey);
    a->x1 = x1;
    a->y1 = y1;
    if (a->r0 < 1e-6)
        return GCODE_ERR_BAD_TARGET;

    a->a0 = atan2(sy, sx);
    a1 = atan2(ey, ex);
    a->da = a1 - a->a0;
    if (cw) {
        if (a->da >= -1e-9)
            a->da -= 2.0 * pi;
    } else {
        if (a->da <= 1e-9)
            a->da += 2.0 * pi;
    }
    if (a->r1 < 1e-6)            /* target on the centre: degenerate */
        a->r1 = a->r0;

    rmax = a->r0 > a->r1 ? a->r0 : a->r1;
    if (!(tol > 1e-6f))
        tol = 1e-6f;
    ratio = 1.0 - (double)tol / rmax;
    if (ratio < 0.0)
        ratio = 0.0;
    dth = 2.0 * acos(ratio);     /* max sweep per chord for this tolerance */
    if (dth > pi / 2.0)
        dth = pi / 2.0;
    if (dth < 1e-6)
        dth = 1e-6;
    nn = ceil(fabs(a->da) / dth);
    if (nn < 1.0)
        nn = 1.0;
    if (nn > ARC_MAX_SEGS)
        nn = ARC_MAX_SEGS;
    a->n = (int)nn;
    a->i = 0;
    return GCODE_OK;
}

int gcode_arc_init(struct gcode_arc *a, float x0, float y0, float x1, float y1,
                   float io, float jo, bool cw, float tol)
{
    return arc_setup(a, x0, y0, x1, y1, (double)x0 + io, (double)y0 + jo, cw, tol);
}

int gcode_arc_init_r(struct gcode_arc *a, float x0, float y0, float x1, float y1,
                     float r, bool cw, float tol)
{
    double dx = (double)x1 - x0, dy = (double)y1 - y0;
    double d2 = dx * dx + dy * dy;
    double h2, h, rr = r;

    if (d2 < 1e-12)
        return GCODE_ERR_BAD_TARGET;      /* R form cannot do a full circle */
    h2 = 4.0 * rr * rr - d2;
    if (h2 < 0.0) {
        if (h2 > -1e-4 * (1.0 + d2))      /* rounding: exactly a half circle */
            h2 = 0.0;
        else
            return GCODE_ERR_BAD_TARGET;
    }
    h = -sqrt(h2) / sqrt(d2);
    if (!cw)
        h = -h;
    if (rr < 0.0)
        h = -h;
    return arc_setup(a, x0, y0, x1, y1,
                     (double)x0 + 0.5 * (dx - dy * h),
                     (double)y0 + 0.5 * (dy + dx * h), cw, tol);
}

bool gcode_arc_next(struct gcode_arc *a, float *x, float *y)
{
    double t, ang, r;

    if (a->i >= a->n)
        return false;
    a->i++;
    if (a->i == a->n) {
        *x = a->x1;
        *y = a->y1;
        return true;
    }
    t = (double)a->i / (double)a->n;
    ang = a->a0 + a->da * t;
    r = a->r0 + (a->r1 - a->r0) * t;
    *x = (float)(a->cx + r * cos(ang));
    *y = (float)(a->cy + r * sin(ang));
    return true;
}
