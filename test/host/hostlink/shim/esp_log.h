#ifndef SHIM_ESP_LOG_H_
#define SHIM_ESP_LOG_H_
#include <stdarg.h>
typedef int (*vprintf_like_t)(const char *, va_list);
static inline vprintf_like_t esp_log_set_vprintf(vprintf_like_t f) { (void)f; return 0; }
#endif
