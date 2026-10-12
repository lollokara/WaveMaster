#ifndef SHIM_FREERTOS_H_
#define SHIM_FREERTOS_H_
#include <stdint.h>
typedef uint32_t TickType_t;
typedef int BaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY 0xffffffffu
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
static inline int xPortInIsrContext(void) { return 0; }
#endif
