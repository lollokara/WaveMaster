#ifndef ENGRAVE_GPIO_H_
#define ENGRAVE_GPIO_H_

#include <stdbool.h>

/* Digital output signalling the laser driver to fire ("engrave") while
 * high. Pin choice is a placeholder - adjust BOARD_PIN_ENGRAVE in
 * engrave_gpio.c to match the actual wiring to the laser driver. */
void engrave_gpio_init(void);
void engrave_gpio_set(bool on);

#endif /* ENGRAVE_GPIO_H_ */
