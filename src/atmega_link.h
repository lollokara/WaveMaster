#ifndef ATMEGA_LINK_H_
#define ATMEGA_LINK_H_

/*
 * Link to the ATmega328P companion (ArduinoCompanion/) over UART0
 * (GPIO44 TX / GPIO43 RX after the pin swap, 250000 baud 8N1).
 *
 * The ATmega owns the slow laser-interface lines of the IPG DB25:
 *   - power word D0..D7 + LATCH strobe
 *   - EMISSION ENABLE / MO ("arm"), with its non-blocking 2 s arm sequence
 *   - guide (red) laser
 *   - status inputs (STAT 11/12/16, 5 V detect)
 * The fast lines (EMISSION MODULATION = gate, SYNC = PRR) are driven
 * directly by the ESP32 (laser_io.h).
 *
 * Wire protocol v2 (framed, CRC-8, see ArduinoCompanion/README.md):
 *   frame := 0xA5 | cmd | len | payload[len] | crc8(cmd,len,payload)
 *   every command is answered by a frame with cmd | 0x80 (ACK) whose
 *   payload is the 8-byte status block, or cmd 0xFF (NAK, payload = error).
 * The ATmega disarms and zeroes power if it hears nothing for 1 s
 * (heartbeat), so an ESP32 crash/reset can never leave the laser armed.
 *
 * All setters are asynchronous for the caller: they update the desired
 * state and wake the link task (core 1), which sends it, retries, and
 * keeps polling status every 200 ms (the heartbeat).
 */

#include <stdbool.h>
#include <stdint.h>

struct atmega_status {
    bool link_ok;        /* got a valid reply within the last 500 ms */
    bool armed;          /* EMISSION ENABLE is asserted */
    bool ready;          /* armed and the arm settle time has elapsed */
    bool guide_on;
    uint8_t power;       /* power word currently latched */
    uint8_t stat_bits;   /* bit0 STAT11, bit1 STAT12, bit2 STAT16 */
    uint16_t vdetect_mv; /* 5 V detect input, millivolts */
    uint32_t errors;     /* CRC/timeout/NAK count since boot */
};

void atmega_link_init(void);

void atmega_link_set_armed(bool armed);
void atmega_link_set_guide(bool on);

/* Request a new power word. Returns immediately. */
void atmega_link_set_power(uint8_t power);

/* Block the caller (dac_task) until the ATmega has confirmed the given power
 * word is latched, or timeout_ms elapses. Returns true when confirmed. */
bool atmega_link_wait_power(uint8_t power, uint32_t timeout_ms);

/* Block the caller until the laser reports ready (armed + settled), or
 * timeout_ms elapses. Returns true when ready. */
bool atmega_link_wait_ready(uint32_t timeout_ms);

void atmega_link_get_status(struct atmega_status *out);

#endif /* ATMEGA_LINK_H_ */
