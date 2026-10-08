#include <stdio.h>

#include "atmega_link.h"
#include "calib.h"
#include "dac_task.h"
#include "esp_log.h"
#include "galvo_out.h"
#include "grbl.h"
#include "host_serial.h"
#include "laser_io.h"
#include "motion_task.h"

static const char *TAG = "wavemaster";

void app_main(void)
{
    bool hw_ok;

    laser_io_early_init();               /* gate inactive before anything else */
    calib_init();
    atmega_link_init();
    if (!host_serial_init())             /* from here on logs are [MSG:...] lines */
        ESP_LOGE(TAG, "host serial init failed");

    hw_ok = dac_task_start();
    if (hw_ok) {
        galvo_out_init();
        hw_ok = motion_task_start();
    }
    if (!hw_ok)
        grbl_set_fault("DAC/motion start failed");

    if (!grbl_task_start())
        ESP_LOGE(TAG, "GRBL task failed to start");

    if (hw_ok)
        ESP_LOGI(TAG, "ready: tick %.2f us, field %.0fx%.0f mm, scale %.4f/%.4f V/mm",
                 (double)galvo_out_tick_us(), (double)calib_get(CAL_FIELD_W),
                 (double)calib_get(CAL_FIELD_H), (double)calib_get(CAL_X_SCALE),
                 (double)calib_get(CAL_Y_SCALE));
}
