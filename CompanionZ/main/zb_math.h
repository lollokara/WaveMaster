#ifndef ZB_MATH_H_
#define ZB_MATH_H_

#include <stdbool.h>
#include <stdint.h>

/* Pure motion math (no ESP-IDF dependencies, host tested in test/host/zbro_motion).
 *
 * Step generator: trapezoid with independent accel/decel, evaluated one step at a time without sqrt
 * (v' = v + a/v while accelerating, v' = v - d/v while braking). Rates are in steps/s. The generator
 * starts at vs, ramps to vc and starts braking exactly when the remaining steps equal the braking
 * distance, so short moves become a triangle automatically. */

struct zb_gen {
    float v, vs, vc, a, d;
    int32_t n;   /* total steps */
    int32_t i;   /* steps already produced */
};

void    zb_gen_init(struct zb_gen *g, int32_t total, float vs, float vc, float a, float d);
/* Step rate of the next step (steps/s); advances the generator. Must not be called when i >= n. */
float   zb_gen_next(struct zb_gen *g);
/* Shorten the move so it brakes to a stop from the current speed using d (n = i + braking steps). */
void    zb_gen_truncate_to_stop(struct zb_gen *g);

/* Clamp a step rate to what the hardware can output (max 1e6/(2*pulse_us), min ~35 Hz). */
float    zb_rate_clamp(float rate, int pulse_us);
/* Full STEP period in 1 MHz ticks for a rate; never shorter than 2*pulse_us, never above 32000. */
uint32_t zb_period_ticks(float rate, int pulse_us);

int32_t zb_mm_to_steps(float mm, float steps_per_mm);
float   zb_steps_to_mm(int32_t steps, float steps_per_mm);

/* Usable step range [lo, hi] of a homed axis. The soft limit stays at least `back` steps inside the
 * endstop trigger point (trigger = max_steps when homing to max, 0 otherwise), so a move to the end of
 * the travel never trips the endstop guard. */
void    zb_soft_limits(int32_t max_steps, int32_t back, bool home_to_max, int32_t *lo, int32_t *hi);
/* Validate a move in the float domain (no int32 overflow possible) and return its step delta.
 * 0 ok, 3 out of range, 4 bad argument (not finite / bad scale). */
int     zb_plan_move(float target_mm, bool relative, bool homed, int32_t pos, float spm,
                     float max_range_mm, float backoff_mm, bool home_to_max, int32_t *delta);
/* Highest homing speed (mm/s) whose braking distance v^2/(2*decel) fits in `backoff_mm`. */
float   zb_home_speed_limit(float decel_mm_s2, float backoff_mm);

#endif
