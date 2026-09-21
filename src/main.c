#include <stdio.h>
#include <stdarg.h>
#include "esp_log.h"
#include "laser_ctrl.h"
#include "calib.h"
#include "dac_task.h"
#include "grbl_task.h"
#include "atmega_link.h"

static const char *TAG = "wavemaster";

/*
 * Debug logs share the same physical link (USB-Serial-JTAG) as the
 * GRBL/PC protocol - this board only exposes one usable serial connection
 * to a PC. To keep a strict GRBL sender from choking on interleaved log
 * output, every log line is prefixed with "; " - the GRBL comment marker,
 * which compliant senders/parsers ignore. This assumes esp_log makes one
 * vprintf call per complete line (the common case); it is not a
 * byte-exact guarantee for multi-line messages.
 */
static int grbl_comment_vprintf(const char *fmt, va_list args)
{
    char buf[256];
    int len;

    len = vsnprintf(buf + 2, sizeof(buf) - 2, fmt, args);
    if (len <= 0)
        return len;

    buf[0] = ';';
    buf[1] = ' ';
    /* Single write call to minimize (not eliminate) interleaving with the
     * other core's concurrent log/GRBL-response writes to the same UART. */
    fwrite(buf, 1, (size_t)len + 2 > sizeof(buf) ? sizeof(buf) : (size_t)len + 2, stdout);

    return len;
}

void app_main(void)
{
    esp_log_set_vprintf(grbl_comment_vprintf);

    ESP_LOGI(TAG, "WaveMaster ESP32-S3 boot");

    calib_init();
    laser_ctrl_init();
    atmega_link_init();

    if (!dac_task_start()) {
        ESP_LOGE(TAG, "DAC task failed to start, halting bring-up");
        return;
    }

    if (!grbl_task_start()) {
        ESP_LOGE(TAG, "GRBL task failed to start, halting bring-up");
        return;
    }

    ESP_LOGI(TAG, "WaveMaster ready: core0=DAC/SPI, core1=GRBL/UART");
}
