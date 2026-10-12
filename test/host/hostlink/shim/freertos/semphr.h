#ifndef SHIM_SEMPHR_H_
#define SHIM_SEMPHR_H_
#include "FreeRTOS.h"
/* Single-threaded test: a mutex is a depth counter, so a missing give or a
 * give without take shows up as a non-zero or negative depth. */
typedef int *SemaphoreHandle_t;
static inline SemaphoreHandle_t shim_mutex(void) { static int pool[8]; static int n; return &pool[n++]; }
#define xSemaphoreCreateRecursiveMutex() shim_mutex()
#define xSemaphoreCreateMutex() shim_mutex()
#define xSemaphoreTakeRecursive(m, t) ((void)(t), ++*(m), pdTRUE)
#define xSemaphoreGiveRecursive(m) (--*(m), pdTRUE)
#define xSemaphoreTake(m, t) ((void)(t), ++*(m), pdTRUE)
#define xSemaphoreGive(m) (--*(m), pdTRUE)
#endif
