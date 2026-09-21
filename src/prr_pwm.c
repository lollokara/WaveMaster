#include "prr_pwm.h"
#include <stdint.h>
#include "driver/ledc.h"

/* Placeholder pin - adjust to match the actual laser driver wiring. */
#define BOARD_PIN_PRR 3

#define PRR_LEDC_TIMER   LEDC_TIMER_0
#define PRR_LEDC_CHANNEL LEDC_CHANNEL_0
#define PRR_LEDC_MODE    LEDC_LOW_SPEED_MODE
#define PRR_DUTY_RES     LEDC_TIMER_10_BIT

void prr_pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = PRR_LEDC_MODE,
        .timer_num = PRR_LEDC_TIMER,
        .duty_resolution = PRR_DUTY_RES,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_cfg = {
        .gpio_num = BOARD_PIN_PRR,
        .speed_mode = PRR_LEDC_MODE,
        .channel = PRR_LEDC_CHANNEL,
        .timer_sel = PRR_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ch_cfg);
}

void prr_pwm_set_hz(float hz)
{
    if (hz <= 0.0f) {
        ledc_set_duty(PRR_LEDC_MODE, PRR_LEDC_CHANNEL, 0);
        ledc_update_duty(PRR_LEDC_MODE, PRR_LEDC_CHANNEL);
        return;
    }

    ledc_set_freq(PRR_LEDC_MODE, PRR_LEDC_TIMER, (uint32_t)hz);
    ledc_set_duty(PRR_LEDC_MODE, PRR_LEDC_CHANNEL, (1 << PRR_DUTY_RES) / 2);
    ledc_update_duty(PRR_LEDC_MODE, PRR_LEDC_CHANNEL);
}
