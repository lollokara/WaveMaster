#ifndef LASER_FILL_H_
#define LASER_FILL_H_

#include <stddef.h>

/*
 * Pure 2D geometry: generates a hatch fill for a closed polygon in
 * machine-space mm (the same units/frame grbl_task.c tracks position in -
 * no volts conversion happens here). Parallel scan lines at `angle_deg`
 * (standard math convention: 0 = horizontal, measured in the polygon's own
 * coordinate frame) spaced `spacing_mm` apart are intersected against every
 * polygon edge using the even-odd fill rule - the standard scanline
 * polygon-fill algorithm. This correctly handles convex and concave
 * (non-self-intersecting) single-contour polygons; it does not handle
 * multiple disjoint contours (e.g. a letter "O"'s inner hole) or
 * self-intersecting outlines - both are known gaps, see STATUS.md.
 *
 * Each inside span becomes one call to emit_stroke(x0,y0,x1,y1). This
 * module has no notion of "laser on/off" or DAC volts - the caller
 * (grbl_task.c's mark_stroke(), used as the emit_stroke callback) is
 * responsible for turning each span into an actual marking move.
 */
void laser_fill_generate(const float *px, const float *py, size_t n,
                          float angle_deg, float spacing_mm,
                          void (*emit_stroke)(float x0, float y0, float x1, float y1));

#endif /* LASER_FILL_H_ */
