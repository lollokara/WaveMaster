#ifndef CALIB_H_
#define CALIB_H_

/*
 * Persistent settings ($-parameters), stored in NVS, loaded at boot,
 * live-settable via "$<n>=<value>" and listed by "$$".
 *
 * Numbering keeps the GRBL meanings Rayforge reads ($30/$32 power and
 * laser mode, $110/$111 max rate, $120/$121 accel, $130/$131 work area,
 * $12 arc tolerance, $13 units) and puts everything galvo-specific in its
 * own range. Values are floats; flags are 0/1.
 *
 *  $3    axis invert mask: bit0 = invert X, bit1 = invert Y        [0]
 *  $12   arc tolerance, mm (G2/G3 chord error)                    [0.01]
 *  $13   report inches - fixed 0 (read-only)
 *  $30   S value for 100% power                                   [1000]
 *  $31   S value for 0% power - fixed 0 (read-only)
 *  $32   laser mode - fixed 1 (read-only)
 *  $100  X scale, volts per mm                                    [0.1]
 *  $101  Y scale, volts per mm                                    [0.1]
 *  $110  max marking speed, mm/min ($111 is an alias)             [300000]
 *  $120  jump acceleration, mm/s^2, 0 = no ramp ($121 alias)      [2000000]
 *  $130  work area width, mm                                      [100]
 *  $131  work area height, mm                                     [100]
 *  $140  X offset, volts                                          [0]
 *  $141  Y offset, volts                                          [0]
 *  $142  radial (F-theta) correction k1: r' = r(1 + k1 r^2)       [0]
 *  $143  swap X/Y axes (0/1)                                      [0]
 *  $144  origin: 0 = work area corner (0,0)..(W,H) centred on the
 *        lens; 1 = (0,0) is the lens centre                       [0]
 *  $150  wobble diameter, mm, 0 = off                             [0]
 *  $151  wobble pitch, mm per revolution                          [0.5]
 *  $200  tick period, us (one DAC point per tick)                 [10]
 *  $201  jump speed, mm/s                                         [3000]
 *  $203  default marking speed when no F was given, mm/s         [500]
 *  $210  laser-on delay, us                                       [100]
 *  $211  laser-off delay, us                                      [120]
 *  $212  jump delay, us                                           [300]
 *  $213  extra jump delay per mm of jump, us/mm                   [0]
 *  $214  mark delay (hold at end of a polyline), us               [100]
 *  $215  polygon delay at a 180 deg corner, us                    [0]
 *  $216  polygon delay angle threshold, degrees                   [30]
 *  $220  pulse repetition rate (SYNC), Hz                         [30000]
 *  $221  SYNC duty cycle, %                                       [50]
 *  $223  power mode: 0 analog (ATmega word), 1 pulse density      [0]
 *  $224  power at S = 0+ (min), %                                 [0]
 *  $225  power at S = $30 (max), %                                [100]
 *  $226  auto-arm when the first mark needs the laser (0/1)        [1]
 *  $227  max wait for arm/ready, ms                               [4000]
 *  $229  pulse-density period, ticks                              [10]
 *  $230  gate (EMISSION MODULATION) active low (0/1)              [0]
 */

#include <stdbool.h>

enum calib_id {
    CAL_AXIS_INVERT,
    CAL_ARC_TOL,
    CAL_S_MAX,
    CAL_X_SCALE,
    CAL_Y_SCALE,
    CAL_MAX_MARK_RATE,     /* mm/min */
    CAL_JUMP_ACCEL,
    CAL_FIELD_W,
    CAL_FIELD_H,
    CAL_X_OFFSET,
    CAL_Y_OFFSET,
    CAL_RADIAL_K1,
    CAL_SWAP_XY,
    CAL_ORIGIN_CENTER,
    CAL_WOBBLE_D,
    CAL_WOBBLE_PITCH,
    CAL_TICK_US,
    CAL_JUMP_SPEED,
    CAL_DEFAULT_MARK_SPEED,
    CAL_LASER_ON_DELAY,
    CAL_LASER_OFF_DELAY,
    CAL_JUMP_DELAY,
    CAL_JUMP_DELAY_PER_MM,
    CAL_MARK_DELAY,
    CAL_POLYGON_DELAY,
    CAL_POLYGON_ANGLE,
    CAL_PRR_HZ,
    CAL_PRR_DUTY,
    CAL_POWER_MODE,
    CAL_POWER_MIN,
    CAL_POWER_MAX,
    CAL_AUTO_ARM,
    CAL_ARM_TIMEOUT_MS,
    CAL_PD_PERIOD,
    CAL_GATE_ACTIVE_LOW,
    CAL_COUNT
};

/* Loads NVS (initialises the NVS partition). Call first thing in app_main. */
void calib_init(void);

/* Current value of a setting. Cheap (array read) - fine in hot paths, but
 * hot loops should still cache values they use per tick. */
float calib_get(enum calib_id id);

/* Sets a setting by its "$" number, validating range and persisting it.
 * Returns 0 on success, or a GRBL error code: 3 (unsupported $ number),
 * 13/33-ish style codes are not needed - use 3 for unknown/read-only and
 * 9 ("G-code locked out") is not used. Invalid value -> 3 as well. */
int calib_set_param(int number, float value);

/* Emits every setting as "$<n>=<value>\r\n" (GRBL-compatible plus the
 * galvo range), one write_line() call per line. */
void calib_dump(void (*write_line)(const char *line));

/* Monotonic counter bumped on every successful calib_set_param(), so
 * consumers can cheaply notice changes. */
unsigned calib_generation(void);

#endif /* CALIB_H_ */
