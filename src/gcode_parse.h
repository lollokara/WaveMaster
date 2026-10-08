#ifndef GCODE_PARSE_H_
#define GCODE_PARSE_H_

/*
 * Pure G-code helpers - no ESP-IDF dependencies, unit-tested on the host
 * (test/host/gcode/). The protocol/state machine lives in grbl.c.
 */

#include <stdbool.h>
#include <stdint.h>

#define GCODE_MAX_WORDS 32

struct gcode_word {
    char letter;     /* 'A'..'Z' */
    float value;
};

struct gcode_block {
    int n;
    struct gcode_word w[GCODE_MAX_WORDS];
};

/* GRBL status codes used here. */
#define GCODE_OK              0
#define GCODE_ERR_NO_LETTER   1   /* expected command letter */
#define GCODE_ERR_BAD_NUMBER  2   /* bad number format */
#define GCODE_ERR_OVERFLOW    14  /* line overflow */
#define GCODE_ERR_UNSUPPORTED 20
#define GCODE_ERR_BAD_TARGET  33  /* invalid target (arc geometry) */

/* Parse a decimal number: optional sign, digits, optional '.', digits. At
 * least one digit is required; no exponent. Advances *pp. 0 or error 2. */
int gcode_parse_number(const char **pp, float *out);

/* Tokenise one line into words. Whitespace, ';' comments, '(...)' comments,
 * a leading '/' (block delete) and '%' lines are ignored; letters are case
 * insensitive; N<line> words are dropped. Returns 0 or a GRBL error. */
int gcode_parse_line(const char *line, struct gcode_block *b);

/* First word with this letter, or NULL. */
const struct gcode_word *gcode_find(const struct gcode_block *b, char letter);

/* Laser power byte for an S word: 0 if s <= 0, else
 * clamp(round(255 * (pmin + (pmax - pmin) * min(s/smax, 1)) / 100), 1, 255).
 * pmin / pmax are percentages. */
uint8_t gcode_power_byte(float s, float smax, float pmin_pct, float pmax_pct);

/* Marking speed (mm/s) for a feed in mm/min: F/60 clamped to
 * [0.1, max_rate_mm_min/60]; feed <= 0 gives default_mm_s (also clamped). */
float gcode_mark_speed(float feed_mm_min, float max_rate_mm_min, float default_mm_s);

/* Arc linearisation. Iterate with gcode_arc_next(); the last point returned
 * is exactly the target. Nothing is allocated. */
struct gcode_arc {
    double cx, cy;       /* centre */
    double r0, r1;       /* radius at start / at the target */
    double a0, da;       /* start angle, signed sweep (rad) */
    float x1, y1;        /* target */
    int n, i;            /* segment count, segments emitted */
};

/* Centre-offset form: centre = start + (io, jo). cw = G2. tol = chord
 * tolerance in mm. Returns 0 or GCODE_ERR_BAD_TARGET (zero radius). Equal
 * start and end give a full circle. The radius runs linearly from the start
 * radius to the target's radius, so a mismatch becomes a gentle spiral
 * instead of a kink at the end. */
int gcode_arc_init(struct gcode_arc *a, float x0, float y0, float x1, float y1,
                   float io, float jo, bool cw, float tol);

/* Radius form (GRBL semantics: R < 0 selects the long way round). Returns 0
 * or GCODE_ERR_BAD_TARGET if the radius is too small / endpoints coincide. */
int gcode_arc_init_r(struct gcode_arc *a, float x0, float y0, float x1, float y1,
                     float r, bool cw, float tol);

/* Next point on the arc; false when finished. */
bool gcode_arc_next(struct gcode_arc *a, float *x, float *y);

#endif /* GCODE_PARSE_H_ */
