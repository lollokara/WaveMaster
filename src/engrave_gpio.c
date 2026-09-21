#include "engrave_gpio.h"
#include "driver/gpio.h"

/* Placeholder pin - adjust to match the actual laser driver wiring. */
#define BOARD_PIN_ENGRAVE 4

void engrave_gpio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BOARD_PIN_ENGRAVE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&cfg);
    gpio_set_level(BOARD_PIN_ENGRAVE, 0);
}

void engrave_gpio_set(bool on)
{
    gpio_set_level(BOARD_PIN_ENGRAVE, on ? 1 : 0);
}
