#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "atmega_link.h"
#include "calib.h"
#include "dac_task.h"
#include "esp_attr.h"
#include "esp_core_dump.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
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

#define CRASH_REPORT_TICKS 150           /* 30 s after boot: the hub link is up */

#define TWDT_MAGIC 0x74776474u

/* The task watchdog only names the starved task on the console, which the hub never sees. The interrupt hook
 * below keeps that text, plus what each core was running, in RTC memory (it survives the reset) for the next
 * boot to log. */
static RTC_NOINIT_ATTR uint32_t s_twdt_magic;
static RTC_NOINIT_ATTR char s_twdt_info[224];
static RTC_NOINIT_ATTR uint32_t s_crash_seen;       /* fingerprint of the core dump already sent to the hub */

static void twdt_collect(void *opaque, const char *msg)
{
    (void)opaque;
    strlcat(s_twdt_info, msg, sizeof(s_twdt_info));
}

void IRAM_ATTR esp_task_wdt_isr_user_handler(void)
{
    char part[40];
    int cpus = 0;

    s_twdt_info[0] = '\0';
    esp_task_wdt_print_triggered_tasks(twdt_collect, NULL, &cpus);
    for (int core = 0; core < 2; core++) {
        TaskHandle_t t = xTaskGetCurrentTaskHandleForCore(core);

        snprintf(part, sizeof(part), " | core%d running %s", core, t ? pcTaskGetName(t) : "?");
        strlcat(s_twdt_info, part, sizeof(s_twdt_info));
    }
    s_twdt_magic = TWDT_MAGIC;
}

static void twdt_report(void)
{
    size_t n = strnlen(s_twdt_info, sizeof(s_twdt_info) - 1);
    char chunk[100];

    if (s_twdt_magic != TWDT_MAGIC)
        return;
    s_twdt_magic = 0;
    s_twdt_info[sizeof(s_twdt_info) - 1] = '\0';
    for (size_t i = 0; i < n; i++)
        if (s_twdt_info[i] == '\n' || s_twdt_info[i] == '\r')
            s_twdt_info[i] = ' ';
    for (size_t off = 0; off < n; off += sizeof(chunk) - 1) {
        strlcpy(chunk, s_twdt_info + off, sizeof(chunk));
        ESP_LOGE(TAG, "task watchdog:%s", chunk);
    }
}

/* A panic or watchdog reset leaves its summary in the coredump partition. Log it so the hub gets the task, the
 * PC and the backtrace (resolve them with the firmware.elf whose sha is printed). The dump stays in flash: it
 * holds every task's stack, and esptool can read it until the next crash replaces it. Each dump is logged once
 * (fingerprint kept in RTC memory). */
static void crash_report(void)
{
    esp_core_dump_summary_t *s;
    char line[96];
    size_t len = 0;
    uint32_t fp;
    esp_err_t err = esp_core_dump_image_check();

    twdt_report();
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "no core dump from the last reset (%s)", esp_err_to_name(err));
        return;
    }
    s = malloc(sizeof(*s));
    if (!s)
        return;
    if (esp_core_dump_get_summary(s) == ESP_OK) {
        uint32_t n = s->exc_bt_info.depth < 16 ? s->exc_bt_info.depth : 16;

        fp = s->exc_pc;
        for (uint32_t i = 0; i < n; i++)
            fp = fp * 31u + s->exc_bt_info.bt[i];
        if (fp == s_crash_seen) {
            free(s);
            return;
        }
        s_crash_seen = fp;
        ESP_LOGE(TAG, "last crash: task %s pc 0x%08lx elf %.9s%s", s->exc_task, (unsigned long)s->exc_pc,
                 (const char *)s->app_elf_sha256, s->exc_bt_info.corrupted ? " (backtrace corrupted)" : "");
        line[0] = '\0';
        for (uint32_t i = 0; i < n; i++) {
            len += snprintf(line + len, sizeof(line) - len, " 0x%08lx", (unsigned long)s->exc_bt_info.bt[i]);
            if (len > sizeof(line) - 14 || i + 1 == n) {
                ESP_LOGE(TAG, "backtrace:%s", line);
                len = 0;
                line[0] = '\0';
            }
        }
    }
    free(s);
}

static void fleet_task(void *arg)
{
    (void)arg;
    for (unsigned tick = 0;; tick++) {
        espfleet_loop();
        vTaskDelay(pdMS_TO_TICKS(200));
        if (tick == CRASH_REPORT_TICKS)
            crash_report();
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
