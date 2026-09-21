#include "laser_fill.h"
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Bounds the number of scanline/polygon-edge intersections handled per
 * scanline - generous for any reasonably-simple engraving outline (a
 * polygon would need over 32 edges crossing the *same* scanline to
 * overflow this, which text/vector-art shapes essentially never do). */
#define LASER_FILL_MAX_XINTS 64

void laser_fill_generate(const float *px, const float *py, size_t n,
                          float angle_deg, float spacing_mm,
                          void (*emit_stroke)(float, float, float, float))
{
    float rad, rot_c, rot_s, back_c, back_s;
    float *rx, *ry;
    float min_v = 1e30f, max_v = -1e30f;
    size_t i;
    float y;

    if (n < 3 || spacing_mm <= 0.0001f || !emit_stroke)
        return;

    rx = malloc(n * sizeof(float));
    ry = malloc(n * sizeof(float));
    if (!rx || !ry) {
        free(rx);
        free(ry);
        return;
    }

    /* Rotate the polygon by -angle so hatch lines become plain horizontal
     * scanlines in this rotated frame; each stroke's endpoints are
     * rotated back by +angle before being emitted. */
    rad = angle_deg * (float)M_PI / 180.0f;
    rot_c = cosf(-rad);
    rot_s = sinf(-rad);
    back_c = cosf(rad);
    back_s = sinf(rad);

    for (i = 0; i < n; i++) {
        float u = px[i] * rot_c - py[i] * rot_s;
        float v = px[i] * rot_s + py[i] * rot_c;

        rx[i] = u;
        ry[i] = v;
        if (v < min_v)
            min_v = v;
        if (v > max_v)
            max_v = v;
    }

    for (y = min_v + spacing_mm * 0.5f; y < max_v; y += spacing_mm) {
        float xints[LASER_FILL_MAX_XINTS];
        int nx = 0;
        size_t j;

        for (i = 0; i < n; i++) {
            size_t k = (i + 1) % n;
            float y0 = ry[i], y1 = ry[k];
            float x0 = rx[i], x1 = rx[k];

            if ((y0 <= y && y1 > y) || (y1 <= y && y0 > y)) {
                float t = (y - y0) / (y1 - y0);

                if (nx < LASER_FILL_MAX_XINTS)
                    xints[nx++] = x0 + t * (x1 - x0);
            }
        }

        /* Ascending insertion sort - nx is small (bounded above). */
        for (i = 1; i < (size_t)nx; i++) {
            float key = xints[i];

            j = i;
            while (j > 0 && xints[j - 1] > key) {
                xints[j] = xints[j - 1];
                j--;
            }
            xints[j] = key;
        }

        /* Even-odd rule: consecutive intersection pairs are inside spans. */
        for (i = 0; i + 1 < (size_t)nx; i += 2) {
            float ux0 = xints[i], ux1 = xints[i + 1];
            float ox0 = ux0 * back_c - y * back_s;
            float oy0 = ux0 * back_s + y * back_c;
            float ox1 = ux1 * back_c - y * back_s;
            float oy1 = ux1 * back_s + y * back_c;

            emit_stroke(ox0, oy0, ox1, oy1);
        }
    }

    free(rx);
    free(ry);
}
