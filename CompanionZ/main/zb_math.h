#ifndef ZB_MATH_H_
#define ZB_MATH_H_

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

#endif
