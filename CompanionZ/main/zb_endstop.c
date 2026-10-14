#include "zb_endstop.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_ll.h"

#include "zb_pins.h"
#include "zb_settings.h"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t *s_cap_src;
static volatile bool     s_latched;
static volatile int32_t  s_cap;
static volatile uint32_t s_edge_us;     /* time of the most recent edge (wraps, differences only) */
static volatile bool     s_invert;
static uint32_t          s_debounce_us = 2000;

static void refresh_settings(void)
{
    s_invert = zb_set_get(ZS_ES_INVERT) != 0.0f;
    s_debounce_us = (uint32_t)zb_set_get(ZS_ES_DEBOUNCE_US);
}

/* NC wiring (es_invert = 0): switch closed to GND normally, open (pull-up high) = triggered. */
static inline bool IRAM_ATTR level_active(void)
{
    return (gpio_ll_get_level(&GPIO, ZB_PIN_ENDSTOP) != 0) != s_invert;
}

static void IRAM_ATTR es_isr(void *arg)
{
    (void)arg;
    s_edge_us = (uint32_t)esp_timer_get_time();
    if (level_active() && !s_latched) {
        s_cap = s_cap_src ? *s_cap_src : 0;
        s_latched = true;
    }
}

void zb_endstop_init(volatile int32_t *capture_src)
{
    s_cap_src = capture_src;
    refresh_settings();
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << ZB_PIN_ENDSTOP,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);   /* already installed is fine */
    gpio_isr_handler_add(ZB_PIN_ENDSTOP, es_isr, NULL);
}

bool zb_endstop_active(void)
{
    refresh_settings();
    return level_active();
}

void zb_endstop_rearm(void)
{
    refresh_settings();
    portENTER_CRITICAL(&s_mux);
    s_latched = false;
    s_edge_us = (uint32_t)esp_timer_get_time();
    if (level_active()) {
        s_cap = s_cap_src ? *s_cap_src : 0;
        s_latched = true;
    }
    portEXIT_CRITICAL(&s_mux);
}

bool zb_endstop_poll(int32_t *captured)
{
    refresh_settings();
    portENTER_CRITICAL(&s_mux);
    bool latched = s_latched;
    int32_t cap = s_cap;
    uint32_t edge = s_edge_us;
    portEXIT_CRITICAL(&s_mux);
    if (!latched) return false;
    if ((uint32_t)((uint32_t)esp_timer_get_time() - edge) < s_debounce_us) return false;
    if (level_active()) {
        if (captured) *captured = cap;
        return true;
    }
    portENTER_CRITICAL(&s_mux);
    if (s_edge_us == edge) s_latched = false;   /* stable inactive: it was a bounce */
    portEXIT_CRITICAL(&s_mux);
    return false;
}
