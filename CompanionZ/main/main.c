#include "esp_log.h"
#include "zb_cal.h"
#include "zb_motion.h"
#include "zb_net.h"
#include "zb_settings.h"
#include "zb_web.h"

static void on_motion_event(enum zb_evt_type type, float pos_mm, int err)
{
    zb_net_on_event(type, pos_mm, err);
    zb_web_on_event(type, pos_mm, err);
}

static void on_cal_change(void)
{
    zb_net_on_cal();
    zb_web_on_cal();
}

void app_main(void)
{
    /* Settings first (NVS), then motion: zb_motion_start() configures the A4988 pins and leaves the driver
     * disabled before any radio or network code runs. */
    zb_settings_init();
    zb_motion_start(on_motion_event);
    zb_cal_set_cb(on_cal_change);
    zb_net_start();
    zb_web_start();
}
