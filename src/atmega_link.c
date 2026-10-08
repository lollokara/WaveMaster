/* STUB - replaced by the ATmega agent. */
#include "atmega_link.h"
#include <string.h>
void atmega_link_init(void) {}
void atmega_link_set_armed(bool armed) { (void)armed; }
void atmega_link_set_guide(bool on) { (void)on; }
void atmega_link_set_power(uint8_t power) { (void)power; }
bool atmega_link_wait_power(uint8_t power, uint32_t timeout_ms) { (void)power; (void)timeout_ms; return true; }
bool atmega_link_wait_ready(uint32_t timeout_ms) { (void)timeout_ms; return true; }
void atmega_link_get_status(struct atmega_status *out) { memset(out, 0, sizeof(*out)); }
