#ifndef NO_OS_PRINT_LOG_H_
#define NO_OS_PRINT_LOG_H_

#include "esp_log.h"

#define pr_err(fmt, ...)   ESP_LOGE("ad3552r", fmt, ##__VA_ARGS__)
#define pr_debug(fmt, ...) ESP_LOGD("ad3552r", fmt, ##__VA_ARGS__)

#endif /* NO_OS_PRINT_LOG_H_ */
