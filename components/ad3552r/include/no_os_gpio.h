#ifndef NO_OS_GPIO_H_
#define NO_OS_GPIO_H_

#include <stdint.h>

#define NO_OS_GPIO_LOW  0
#define NO_OS_GPIO_HIGH 1

struct no_os_gpio_init_param {
    int32_t number;
};

struct no_os_gpio_desc {
    int32_t number;
};

int32_t no_os_gpio_get_optional(struct no_os_gpio_desc **desc,
                                 struct no_os_gpio_init_param *param);
int32_t no_os_gpio_direction_output(struct no_os_gpio_desc *desc, uint8_t value);
int32_t no_os_gpio_set_value(struct no_os_gpio_desc *desc, uint8_t value);
int32_t no_os_gpio_remove(struct no_os_gpio_desc *desc);

#endif /* NO_OS_GPIO_H_ */
