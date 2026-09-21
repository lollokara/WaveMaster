#ifndef CALIB_H_
#define CALIB_H_

#include <stdbool.h>

/*
 * Persistent per-axis calibration (volts-per-mm gain + a volts offset),
 * stored in NVS flash so it survives reboots. Loaded once at boot via
 * calib_init(); updated live via GRBL-style "$<number>=<value>" settings
 * (see grbl_task.c), each of which persists immediately.
 *
 * Setting numbers (deliberately chosen to echo real GRBL's $100/$101
 * axis-scale convention, though the units here are V/mm, not steps/mm):
 *   $100 = X volts-per-mm
 *   $101 = Y volts-per-mm
 *   $110 = X offset (volts)
 *   $111 = Y offset (volts)
 *
 * Marking-quality parameters (galvo/fiber-laser specific, matching the
 * concepts real BJJCZ/EZCAD-class marking cards expose - see STATUS.md's
 * feature-gap writeup):
 *   $130 = jump delay (us) - settle time after a non-marking (G0/laser-off)
 *          move before the next command executes.
 *   $131 = mark delay (us) - settle time after a marking move.
 *   $132 = polygon delay (us) - additional settle time added on top of the
 *          mark delay at every vertex between marking segments (not
 *          angle-aware - applied uniformly, unlike real corner-angle-based
 *          polygon delay).
 *   $133 = laser-on delay (us) - time given to the laser to physically
 *          respond after the engrave GPIO is asserted, before the next
 *          command (typically the first point of the mark) executes.
 *   $134 = laser-off delay (us) - same, after the engrave GPIO is cleared.
 *   $140 = radial (F-theta field) correction coefficient k1, applied as
 *          r' = r * (1 + k1*r^2) in mm-space before the per-axis
 *          volts-per-mm/offset conversion - a low-order barrel/pincushion
 *          correction, not a full NxN calibration-grid table. 0 = disabled
 *          (identity, matches prior linear-only behavior).
 *   $150 = wobble diameter (mm) - 0 disables. Circular kerf-widening
 *          offset applied perpendicular to travel direction on marking runs.
 *   $151 = wobble pitch (mm) - path distance per wobble revolution.
 *   $160 = skywriting margin (mm) - 0 disables. Extends each marking
 *          stroke by this distance at both ends with the laser off, so the
 *          galvo is already at speed when the laser turns on/off instead
 *          of decelerating into/out of the mark.
 *   $170 = hatch line spacing (mm), used by M64/M65 fill regions.
 *   $171 = hatch angle (degrees), used by M64/M65 fill regions.
 *
 *   $180 = max path acceleration (mm/s^2), used by the look-ahead motion
 *          planner (src/motion_planner.{h,c}) for both the accel/decel
 *          ramps within a segment and the junction-deviation cornering
 *          speed limit between segments.
 *   $181 = junction deviation (mm) - GRBL-style cornering tolerance: how
 *          far the planner will let the actual path deviate from a sharp
 *          corner in exchange for not slowing to a stop there. Smaller =
 *          more conservative (slower) cornering, larger = faster but
 *          rounder corners.
 *   $182 = max velocity (mm/s) - hard cap on cruise speed regardless of
 *          what F requests, so a mistaken/huge feedrate can't demand a
 *          physically-unreasonable speed.
 *
 *   $190 = M68 preview-loop edge subdivisions (points per recorded edge).
 *          Found necessary on real hardware: jumping straight corner-to-
 *          corner exceeds the galvo's slew rate (visible ringing/
 *          "hotspots"); too few subdivisions instead makes each
 *          intermediate point its own visible dwell spot. Raise this
 *          (shrinks each point's dwell time for a given $68 Hz, since
 *          more points share the same per-edge time budget) until the
 *          motion looks continuous rather than a string of dots - live-
 *          tunable, no reflash needed to find the right value for a
 *          given galvo/mirror combination.
 */

void calib_init(void);

float calib_x_volts_per_mm(void);
float calib_x_offset_volts(void);
float calib_y_volts_per_mm(void);
float calib_y_offset_volts(void);

float calib_jump_delay_us(void);
float calib_mark_delay_us(void);
float calib_polygon_delay_us(void);
float calib_laser_on_delay_us(void);
float calib_laser_off_delay_us(void);
float calib_radial_k1(void);
float calib_wobble_diameter_mm(void);
float calib_wobble_pitch_mm(void);
float calib_skywrite_margin_mm(void);
float calib_preview_edge_subdivisions(void);
float calib_hatch_spacing_mm(void);
float calib_hatch_angle_deg(void);

float calib_max_accel_mm_s2(void);
float calib_junction_deviation_mm(void);
float calib_max_velocity_mm_s(void);

/* Sets a calibration parameter by its "$" setting number, persisting it
 * to NVS immediately. Returns false if the number isn't recognized. */
bool calib_set_param(int number, float value);

/* Emits the current calibration as "$<n>=<value>\r\n" lines, one
 * write_line() call per line (used by the "$$" settings dump). */
void calib_dump(void (*write_line)(const char *line));

#endif /* CALIB_H_ */
