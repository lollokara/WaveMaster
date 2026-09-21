#include "laser_ctrl.h"
#include "calib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* Galvo full-scale range: AD3552R channels are configured for
 * AD3552R_CH_OUTPUT_RANGE_NEG_10__10V in ad3552r_board.c, so +-10V is the
 * hard electrical limit - a fixed hardware constant, not user-calibrated.
 * Scaling the actual commanded voltage range down (e.g. for careful
 * initial bring-up on real hardware) belongs in the per-axis volts-per-mm
 * gain instead ($100/$101 in calib.h - live-settable, no reflash needed),
 * not here: this clamp exists only to prevent exceeding the DAC's real
 * electrical range, never as a scaling/calibration mechanism. */
#define GALVO_MAX_VOLTS   10.0f

/* Sized for arc interpolation bursts (grbl_task.c's apply_arc() can enqueue
 * well over a hundred segments for a single G2/G3 line): at ~75k
 * updates/sec measured on real hardware (see dac_task.c's boot-time speed
 * test), core 0 drains this fast enough that depth mostly just needs to
 * absorb the producer/consumer scheduling jitter between the two cores,
 * not sustain genuinely simultaneous throughput. */
#define LASER_CMD_QUEUE_LEN 256

static QueueHandle_t s_cmd_queue;

void laser_ctrl_init(void)
{
    s_cmd_queue = xQueueCreate(LASER_CMD_QUEUE_LEN, sizeof(struct laser_cmd));
}

bool laser_ctrl_submit(const struct laser_cmd *cmd)
{
    /* A short block (rather than failing instantly) absorbs scheduling
     * jitter between the two cores during arc-segment bursts without
     * meaningfully affecting G-code line throughput. */
    return xQueueSend(s_cmd_queue, cmd, pdMS_TO_TICKS(50)) == pdTRUE;
}

bool laser_ctrl_next(struct laser_cmd *cmd, uint32_t timeout_ms)
{
    return xQueueReceive(s_cmd_queue, cmd, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

static float clamp_volts(float v)
{
    if (v > GALVO_MAX_VOLTS)
        return GALVO_MAX_VOLTS;
    if (v < -GALVO_MAX_VOLTS)
        return -GALVO_MAX_VOLTS;
    return v;
}

void laser_ctrl_mm_to_volts(float x_mm, float y_mm, float *out_x_volts, float *out_y_volts)
{
    float k1 = calib_radial_k1();
    float cx = x_mm, cy = y_mm;

    if (k1 != 0.0f) {
        /* Low-order barrel/pincushion correction: r' = r*(1+k1*r^2),
         * applied around the field center (0,0 in machine-space mm). This
         * approximates the F-theta lens distortion real galvo marking
         * heads exhibit; it is not a full per-point calibration-grid
         * table (see calib.h). */
        float r2 = x_mm * x_mm + y_mm * y_mm;
        float factor = 1.0f + k1 * r2;
        cx = x_mm * factor;
        cy = y_mm * factor;
    }

    *out_x_volts = clamp_volts(cx * calib_x_volts_per_mm() + calib_x_offset_volts());
    *out_y_volts = clamp_volts(cy * calib_y_volts_per_mm() + calib_y_offset_volts());
}
