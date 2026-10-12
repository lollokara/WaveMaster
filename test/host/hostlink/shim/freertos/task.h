#ifndef SHIM_TASK_H_
#define SHIM_TASK_H_
#include "FreeRTOS.h"
extern TickType_t g_tick;
extern const char *g_task_name;
static inline TickType_t xTaskGetTickCount(void) { return g_tick; }
static inline const char *pcTaskGetName(void *h) { (void)h; return g_task_name; }
#define xTaskCreatePinnedToCore(f, n, s, a, p, h, c) pdPASS
#endif
