#ifndef LASER_IO_H_
#define LASER_IO_H_

/*
 * The two fast laser lines the ESP32 drives directly (IPG YLP DB25):
 *
 *   GPIO 4  LASER_IO_PIN_GATE  -> DB25 pin 19, EMISSION MODULATION.
 *           The laser emits while this is active. Switched with
 *           sample-accurate timing against the DAC stream.
 *   GPIO 3  LASER_IO_PIN_SYNC  -> DB25 pin 20, SYNC (pulse repetition rate).
 *           Free-running square wave from the LEDC peripheral.
 *
 * Both are driven low (inactive) as early as possible in app_main and by
 * every abort path. Fit a 10k pull-down on the gate line so it is also
 * inactive while the ESP32 is in reset.
 */

#include <stdbool.h>
#include <stdint.h>

#define LASER_IO_PIN_GATE 4
#define LASER_IO_PIN_SYNC 3

/* Configure both pins as outputs, gate inactive, SYNC stopped. Safe to call
 * first thing in app_main (does not read calib). */
void laser_io_early_init(void);

/* Start SYNC at hz with duty_pct (LEDC); hz <= 0 stops it (held low). */
void laser_io_set_prr(float hz, float duty_pct);

/* Gate polarity: active_low=true inverts the electrical level. */
void laser_io_set_gate_active_low(bool active_low);

/* Set the gate. IRAM-safe, callable from ISRs; honours polarity and the
 * global kill switch below. */
void laser_io_gate(bool on);

/* Kill switch: while set, laser_io_gate(true) is ignored and the gate is
 * held inactive. Used by abort / feed hold / preview. */
void laser_io_gate_kill(bool kill);

/* Diagnostics: logical gate state as last driven (true = laser enabled),
 * kill switch, and the SYNC frequency actually running (0 = stopped). */
struct laser_io_state {
    bool gate_on;
    bool killed;
    bool active_low;
    uint32_t prr_hz;
};
void laser_io_get_state(struct laser_io_state *out);

#endif /* LASER_IO_H_ */
