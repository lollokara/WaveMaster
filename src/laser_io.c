#include "laser_io.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <math.h>
#include <stdatomic.h>

static const char *TAG = "laser_io";

#define PRR_TIMER   LEDC_TIMER_0
#define PRR_CHANNEL LEDC_CHANNEL_0
#define PRR_MODE    LEDC_LOW_SPEED_MODE
#define LEDC_SRC_HZ 80000000u

/* All state read from ISRs lives in DRAM (plain statics). */
static atomic_bool s_active_low;
static atomic_bool s_kill;
static bool s_prr_running;

static inline void IRAM_ATTR gate_write_level(bool active)
{
    bool low = atomic_load(&s_active_low);
    bool lvl = active ? !low : low;

    gpio_ll_set_level(&GPIO, LASER_IO_PIN_GATE, lvl ? 1 : 0);
}

void laser_io_early_init(void)
{
    /* Latch the inactive level before the pad becomes an output so it never
     * glitches active. Polarity is active-high until calib is loaded. */
    atomic_store(&s_active_low, false);
    atomic_store(&s_kill, false);
    gpio_set_level(LASER_IO_PIN_GATE, 0);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << LASER_IO_PIN_GATE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gate_write_level(false);

    gpio_set_level(LASER_IO_PIN_SYNC, 0);
    cfg.pin_bit_mask = 1ULL << LASER_IO_PIN_SYNC;
    gpio_config(&cfg);
    gpio_set_level(LASER_IO_PIN_SYNC, 0);
    s_prr_running = false;
}

void laser_io_set_gate_active_low(bool active_low)
{
    atomic_store(&s_active_low, active_low);
    gate_write_level(false); /* re-drive the (new) inactive level */
}

void IRAM_ATTR laser_io_gate(bool on)
{
    if (on && atomic_load(&s_kill))
        on = false;
    gate_write_level(on);
    /* A kill raised between the check and the write must still win. */
    if (on && atomic_load(&s_kill))
        gate_write_level(false);
}

void IRAM_ATTR laser_io_gate_kill(bool kill)
{
    atomic_store(&s_kill, kill);
    if (kill)
        gate_write_level(false);
}

void laser_io_set_prr(float hz, float duty_pct)
{
    if (!(hz > 0.0f)) {
        if (s_prr_running)
            ledc_stop(PRR_MODE, PRR_CHANNEL, 0);
        s_prr_running = false;
        gpio_set_level(LASER_IO_PIN_SYNC, 0);
        return;
    }

    uint32_t freq = (uint32_t)(hz + 0.5f);
    /* Lowest frequency the 14-bit timer can divide down to from 80 MHz. */
    uint32_t fmin = LEDC_SRC_HZ / (1024u * (1u << 14)) + 1u;
    if (freq < fmin)
        freq = fmin;

    uint32_t res = ledc_find_suitable_duty_resolution(LEDC_SRC_HZ, freq);
    if (res == 0) {
        ESP_LOGE(TAG, "PRR %lu Hz not reachable", (unsigned long)freq);
        return;
    }
    if (res > LEDC_TIMER_14_BIT)
        res = LEDC_TIMER_14_BIT;

    ledc_timer_config_t tc = {
        .speed_mode = PRR_MODE,
        .duty_resolution = (ledc_timer_bit_t)res,
        .timer_num = PRR_TIMER,
        .freq_hz = freq,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    if (ledc_timer_config(&tc) != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config(%lu Hz, %lu bit) failed",
                 (unsigned long)freq, (unsigned long)res);
        return;
    }

    uint32_t full = 1u << res;
    float d = duty_pct;
    if (d < 0.0f) d = 0.0f;
    if (d > 100.0f) d = 100.0f;
    uint32_t duty = (uint32_t)lroundf(d * 0.01f * (float)full);
    if (duty < 1) duty = 1;
    if (duty > full - 1) duty = full - 1;

    ledc_channel_config_t cc = {
        .gpio_num = LASER_IO_PIN_SYNC,
        .speed_mode = PRR_MODE,
        .channel = PRR_CHANNEL,
        .timer_sel = PRR_TIMER,
        .duty = duty,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    if (ledc_channel_config(&cc) != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed");
        return;
    }
    s_prr_running = true;
}
