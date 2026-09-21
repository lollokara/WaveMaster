#include "no_os_gpio.h"
#include <stdlib.h>
#include <errno.h>
#include "driver/gpio.h"

int32_t no_os_gpio_get_optional(struct no_os_gpio_desc **desc,
                                 struct no_os_gpio_init_param *param)
{
    struct no_os_gpio_desc *d;

    if (!param) {
        *desc = NULL;
        return 0;
    }

    d = calloc(1, sizeof(*d));
    if (!d)
        return -ENOMEM;

    d->number = param->number;
    *desc = d;

    return 0;
}

int32_t no_os_gpio_direction_output(struct no_os_gpio_desc *desc, uint8_t value)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << desc->number,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&cfg) != ESP_OK)
        return -EIO;

    return no_os_gpio_set_value(desc, value);
}

int32_t no_os_gpio_set_value(struct no_os_gpio_desc *desc, uint8_t value)
{
    if (gpio_set_level((gpio_num_t)desc->number, value) != ESP_OK)
        return -EIO;

    return 0;
}

int32_t no_os_gpio_remove(struct no_os_gpio_desc *desc)
{
    free(desc);
    return 0;
}
