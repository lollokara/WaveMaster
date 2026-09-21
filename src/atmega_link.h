#ifndef ATMEGA_LINK_H_
#define ATMEGA_LINK_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Serial link to the ATmega328P board in ../arduino-laser-control (see
 * that project's src/main.cpp for the authoritative protocol). It owns:
 * guide-laser on/off, laser power setpoint, and system arming. It does
 * NOT own firing (the Engrave GPIO, driven directly by this ESP32 - see
 * engrave_gpio.h) or PRR (driven directly by this ESP32 - see prr_pwm.h) -
 * the ATMEGA's own SET_PWM_EMIT/SET_PWM_SYNC commands exist for its
 * internal modulation/sync signals and are intentionally not driven from
 * the GRBL S/Q words.
 *
 * Wiring: UART0's default pins (GPIO43 TX / GPIO44 RX) - free on this
 * board since the PC/GRBL link uses the native USB-Serial-JTAG peripheral
 * instead (see grbl_task.c / sdkconfig.defaults). Baud 250000, 8N1,
 * matching arduino-laser-control's Serial.begin(250000).
 */

void atmega_link_init(void);

/* Async (fire-and-forget, queued to a background task) - never blocks the
 * caller, so grbl_task stays responsive ('?' queries, new lines, Ctrl-X)
 * no matter what the ATMEGA is doing. See atmega_link.c's header comment
 * for the full design rationale, especially around atmega_link_set_power()
 * and atmega_link_wait_power_settled() below. */
void atmega_link_set_guide(bool on);
void atmega_link_set_armed(bool armed);
void atmega_link_set_power(float percent); /* 0-100, converted to the ATMEGA's 0-255 byte */

/* Blocks the CALLER (not grbl_task - meant for dac_task) until the most
 * recently queued atmega_link_set_power() has been confirmed applied by
 * the ATMEGA (or has timed out / been dropped - either way, it has
 * settled). Bounded at timeout_ms. For dithered/raster engraving: a move
 * queued alongside an S-word should gate on this before dac_task actually
 * writes the DAC, so the galvo never reaches the next position before the
 * current pixel's power has taken effect. Returns true if confirmed
 * within the window, false otherwise (caller should treat false as
 * "power may not match the commanded value yet"). */
bool atmega_link_wait_power_settled(uint32_t timeout_ms);

#endif /* ATMEGA_LINK_H_ */
