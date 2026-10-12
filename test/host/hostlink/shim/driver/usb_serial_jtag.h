#ifndef SHIM_USBJ_H_
#define SHIM_USBJ_H_
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
typedef struct { uint32_t tx_buffer_size, rx_buffer_size; } usb_serial_jtag_driver_config_t;
int usb_serial_jtag_write_bytes(const void *s, size_t n, TickType_t wait);
static inline int usb_serial_jtag_read_bytes(void *b, uint32_t n, TickType_t w) { (void)b; (void)n; (void)w; return 0; }
static inline int usb_serial_jtag_is_driver_installed(void) { return 1; }
static inline esp_err_t usb_serial_jtag_driver_install(usb_serial_jtag_driver_config_t *c) { (void)c; return ESP_OK; }
#endif
