#include <stdio.h>

#include "atmega_link.h"
#include "calib.h"
#include "dac_task.h"
#include "esp_log.h"
#include "espfleet_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "galvo_out.h"
#include "grbl.h"
#include "host_serial.h"
#include "laser_io.h"
#include "motion_task.h"
#include "net_wifi.h"
#include "zbro_link.h"

static const char *TAG = "wavemaster";

#define FLEET_TASK_STACK 6144
#define FLEET_TASK_PRIO  1
#define FLEET_TASK_CORE  1               /* core 0 belongs to dac_task */
#define FLEET_HUB_URL    "http://192.168.3.87:8080"

static void fleet_task(void *arg)
{
    (void)arg;
    for (;;) {
        espfleet_loop();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* Reports reset reason, version and health to the ESPFleet hub. Waits for the
 * WiFi station's IP by itself. Needs NVS (calib_init) and the host log wrapper
 * (host_serial_init) to be up so it can chain onto it. */
static void fleet_start(void)
{
    espfleet_config_t cfg;

    espfleet_config_default(&cfg);
    cfg.hub = FLEET_HUB_URL;
    cfg.name = "wavemaster";
    cfg.enable_ota = false;
    cfg.enable_mdns = false;
    cfg.spawn_task = false;
    if (espfleet_start(&cfg) != 0) {
        ESP_LOGE(TAG, "ESPFleet agent failed to start");
        return;
    }
    if (xTaskCreatePinnedToCore(fleet_task, "espfleet", FLEET_TASK_STACK, NULL,
                                FLEET_TASK_PRIO, NULL, FLEET_TASK_CORE) != pdPASS)
        ESP_LOGE(TAG, "ESPFleet task failed to start");
}

void app_main(void)
{
    bool hw_ok;

    laser_io_early_init();               /* gate inactive before anything else */
    calib_init();
    /* Apply the stored gate polarity at once: with an active-low gate the
     * early-init level (low) would otherwise mean "laser on" until the
     * output stage starts. */
    laser_io_set_gate_active_low(calib_get(CAL_GATE_ACTIVE_LOW) != 0.0f);
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
    if (!net_wifi_start())               /* idle unless $240 = 1 and credentials are stored */
        ESP_LOGE(TAG, "network manager failed to start");
    if (!zbro_link_start())              /* idle until the WiFi station is up */
        ESP_LOGE(TAG, "Z-Bro link failed to start");
    fleet_start();

    if (hw_ok)
        ESP_LOGI(TAG, "ready: tick %.2f us, field %.0fx%.0f mm, scale %.4f/%.4f V/mm",
                 (double)galvo_out_tick_us(), (double)calib_get(CAL_FIELD_W),
                 (double)calib_get(CAL_FIELD_H), (double)calib_get(CAL_X_SCALE),
                 (double)calib_get(CAL_Y_SCALE));
}
