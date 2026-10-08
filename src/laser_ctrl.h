#ifndef LASER_CTRL_H_
#define LASER_CTRL_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/*
 * Shared command queue between the GCODE/UART task (core 1) and the
 * DAC/SPI task (core 0). The GCODE task never touches the SPI bus or the
 * ad3552r driver directly - it only enqueues target values here, so a slow
 * or blocked SPI transaction on core 0 can never stall UART line parsing
 * on core 1.
 */

enum laser_cmd_type {
    LASER_CMD_MOVE,     /* target position, in volts, both axes */
    LASER_CMD_ENGRAVE,  /* engrave GPIO on/off */
    LASER_CMD_PRR,      /* pulse repetition rate, in Hz */
    LASER_CMD_STREAM,   /* a whole precomputed path, one SPI burst */
};

struct laser_cmd {
    enum laser_cmd_type type;
    /* Only meaningful for LASER_CMD_MOVE. Set when this move's G-code line
     * also carried an S-word: the corresponding atmega_link_set_power()
     * call was fired async (see atmega_link.h) so grbl_task never blocks,
     * but dac_task must still wait for it to settle (bounded, via
     * atmega_link_wait_power_settled()) before writing this move to the
     * DAC - otherwise the galvo could reach the new position before the
     * new pixel's power has actually taken effect on the ATMEGA, which
     * breaks dithering/raster correctness. False for every other command
     * and for moves with no S-word on the same line (nothing to wait for). */
    bool gate_on_power;
    union {
        struct {
            float x_volts;
            float y_volts;
        } move;
        bool engrave_on;
        float prr_hz;
        struct {
            /* Heap-allocated by the producer (e.g. grbl_task's arc
             * interpolation, or mark_stroke()'s wobble/skywrite/feed-paced
             * runs); ownership transfers to whoever calls laser_ctrl_next()
             * and gets this command - they must free() both arrays once
             * done (see dac_task.c). Already converted to volts by the
             * producer (via laser_ctrl_mm_to_volts()). */
            float *x_volts;
            float *y_volts;
            size_t n_points;
            /* If true, dac_task paces writes one point at a time (real
             * per-point delay, honoring point_delay_us) instead of using
             * the fast quad-SPI burst path - for feed-rate-paced marking
             * runs, where hitting the requested speed matters more than
             * raw throughput. False (the burst path) for arcs/hatch fill,
             * which have no feed-rate target and want max speed instead. */
            bool paced;
            uint32_t point_delay_us;
            /* How many times to replay this point buffer back-to-back
             * (paced streams only; 1 for every other producer). 0 means
             * "keep replaying until another command is queued" - what M68's
             * preview loop uses.
             *
             * This exists so a repeating shape costs one repetition's worth
             * of heap instead of the whole repeated path: M68 used to
             * memcpy its outline ~41 times into a single ~160KB buffer, and
             * two of those coexisting (one still draining, one just built)
             * overflowed this board's ~300KB heap. Looping on this side
             * also removes the dead time the host spent re-issuing M68
             * between bursts, during which the galvo simply parked. */
            uint32_t repeats;
        } stream;
    };
};

/* Called once from app_main before either task starts. */
void laser_ctrl_init(void);

/* Called by the GCODE task (core 1) to request a new state. Non-blocking:
 * drops the command and returns false if the queue is momentarily full
 * (the DAC task only takes microseconds per command, so this should not
 * happen in practice). */
bool laser_ctrl_submit(const struct laser_cmd *cmd);

/* Called by the DAC task (core 0) to receive the next queued command,
 * blocking up to timeout_ms. Returns false on timeout. */
bool laser_ctrl_next(struct laser_cmd *cmd, uint32_t timeout_ms);

/* True if at least one command is waiting to be picked up. Lets a
 * long-running command on the DAC task (an indefinitely repeating paced
 * stream - see stream.repeats) notice that something newer has arrived and
 * stop, without having to dequeue it first. */
bool laser_ctrl_pending(void);

/* Convert a machine-space coordinate (mm) to DAC output (volts) for both
 * axes together, applying (in order): the radial (F-theta field)
 * correction (calib_radial_k1() - needs both axes at once, since it's a
 * function of distance from field center), then each axis's independent
 * volts-per-mm gain and volts offset (calib.h - live settable/persisted),
 * then clamping to the galvo's hardware range. Shared by every producer
 * of DAC points (plain moves, arcs, mark_stroke()'s wobble/skywrite/paced
 * runs, hatch fill) so they all agree on the same conversion. */
void laser_ctrl_mm_to_volts(float x_mm, float y_mm, float *out_x_volts, float *out_y_volts);

#endif /* LASER_CTRL_H_ */
