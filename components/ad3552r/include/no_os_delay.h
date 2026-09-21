#ifndef NO_OS_DELAY_H_
#define NO_OS_DELAY_H_

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

static inline void no_os_mdelay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static inline void no_os_udelay(uint32_t us)
{
    esp_rom_delay_us(us);
}

#endif /* NO_OS_DELAY_H_ */
