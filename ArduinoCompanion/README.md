# WaveMaster ATmega328P Companion

Arduino Mini (ATmega328P @ 16 MHz) firmware that owns the **slow** lines of the
laser's DB25 interface. The ESP32-S3 talks to it over UART0 using the framed,
CRC-protected protocol v2 below (250000 baud, 8N1, through a level shifter).

Division of labour:

| Owner   | Lines |
|---------|-------|
| ESP32   | EMISSION MODULATION (gate, DB25 pin 19) <- GPIO4; SYNC / PRR (DB25 pin 20) <- GPIO3 |
| ATmega  | power word D0..D7 + LATCH, EMISSION ENABLE (arm / MO), GUIDE laser, AUX OFF, STAT 11/12/16, 5 V detect, LEDs |

The ATmega no longer drives EMISSION MOD or SYNC (software PWM and the Timer 1
ISR were removed).

## Wiring

### Pinout (laser signal -> Arduino pin)

**Power word (D0..D7):**

| Bit | Pin | Note |
|-----|-----|------|
| D0 | D2  | moved (was A7) |
| D1 | D4  | moved (was A6) |
| D2 | A4  | |
| D3 | A2  | |
| D4 | A1  | |
| D5 | D13 | |
| D6 | D3  | |
| D7 | D5  | |

**Control:** EMISSION ENABLE -> A0, LATCH -> D6, GUIDE LASER -> D7, AUX OFF -> D8 (held HIGH).
**Status inputs:** 5V DETECT -> A5, STAT 11 -> D9, STAT 12 -> D10, STAT 16 -> A3.
**LEDs:** RED (5 V present) -> D11, GREEN (0 V present) -> D12.

All power-bit pins are `#define PIN_PWR_BITn` at the top of `src/main.cpp`.

### Required rewiring (from the previous version)

1. **Power bits D0 / D1:** move the laser D0 wire from **A7 to D2** and the D1
   wire from **A6 to D4**. A6/A7 are analog-input-only on the ATmega328P (no
   PORT/DDR bit), so bits 0 and 1 of the power word never reached the laser.
2. **EMISSION MODULATION:** the laser wire (DB25 pin 19) that was on Arduino
   D2 now goes to **ESP32 GPIO4**.
3. **SYNC:** the laser wire (DB25 pin 20) that was on Arduino D4 now goes to
   **ESP32 GPIO3**.
4. Do the moves in this order (or disconnect the laser) so a wire never lands
   on a pin that is still driven by the other function.
5. Grounds of ESP32, level shifter, Arduino and the laser interface must be common.

### Serial link

ESP32 UART0: TX = GPIO44, RX = GPIO43 (the pins are swapped relative to the
IOMUX defaults, configured via `uart_set_pin`). 250000 baud 8N1 through the
level shifter to the ATmega328P hardware UART (D1/D0). The Arduino board's CH340
sits in parallel on the same lines (USB serial monitor works, but do not run a
monitor while the ESP32 is polling).

## Building

```bash
pio run                 # env arduino_mini
pio run -t upload       # upload at 57600
```

Note: the firmware enables the AVR watchdog (250 ms). The classic Arduino Nano
"old bootloader" does not disable the watchdog after a WDT reset and can boot
loop; use the Optiboot bootloader (or flash with an ISP) if that happens.

## Protocol v2

### Frame

```
0xA5 | cmd | len | payload[len] | crc8
```

* `len <= 8`.
* `crc8` covers `cmd, len, payload` -- CRC-8, polynomial 0x07, init 0x00, no
  reflection, no final XOR.
* The receiver hunts for `0xA5`; a partially received frame is dropped after
  5 ms of silence. Multi-byte values are big endian.

### Commands (ESP32 -> ATmega)

| Cmd  | Name       | Payload | Effect |
|------|------------|---------|--------|
| 0x01 | SET_ARM    | 1 byte (0/1) | 1: EMISSION ENABLE high, `ready` after `ARM_SETTLE_MS` (2000 ms, non-blocking). Repeating while armed does not restart the timer. 0: EMISSION ENABLE low immediately, power word set to 0 and latched. Guide is unaffected. |
| 0x02 | SET_GUIDE  | 1 byte (0/1) | Guide (red) laser on/off. |
| 0x03 | SET_POWER  | 1 byte | Write the 8 power bits, then pulse LATCH. The reply is sent after the latch, so it confirms the value. |
| 0x04 | GET_STATUS | none | No action; doubles as the heartbeat. |

### Replies (ATmega -> ESP32)

Every valid command is answered by a frame with `cmd | 0x80` and an 8-byte
status payload:

| Byte | Content |
|------|---------|
| 0 | flags: bit0 armed, bit1 ready (armed and settle elapsed), bit2 guide on, bit3 heartbeat-timeout tripped since last reply (latched, cleared when reported), bit4 5 V present |
| 1 | power word currently latched |
| 2 | status inputs: bit0 STAT11, bit1 STAT12, bit2 STAT16 |
| 3..4 | 5 V detect, millivolts (big endian) |
| 5 | protocol version (2) |
| 6 | last error code |
| 7 | reserved (0) |

Bad CRC, bad length/argument or unknown command is answered by
`0xA5 0xFF 0x01 <err> <crc>`.

Error codes: 0 none, 1 CRC, 2 unknown command, 3 bad length, 4 bad argument.

### Safety

* **Heartbeat:** if armed and no valid frame arrives for 1000 ms, the ATmega
  disarms, zeroes and latches the power word, and sets flag bit3. The ESP32
  polls every 200 ms.
* **Watchdog:** AVR watchdog (250 ms) is reset in `loop()`; a hang resets into
  the safe state (disarmed, power 0, guide off, AUX OFF high).
* `setup()` puts every output in the safe state before anything else.

### LATCH timing

Power bits are written with direct port writes (~2 us for all eight), followed
by `LATCH_SETUP_US` (2 us) settling and a `LATCH_PULSE_US` (3 us) strobe. This
assumes the laser latches the word on the strobe; tune the defines if needed.

### Timing

At 250 kbaud one byte is 40 us. A SET_POWER request is 5 bytes (200 us) and the
reply 12 bytes (480 us): about 0.7 ms on the wire plus ~30 us ATmega processing
and the ESP32 task wake-up, i.e. roughly 0.75-0.9 ms round trip.

## LEDs

Red (D11): 5 V present (analog reading > ~3.4 V). Green (D12): 0 V present
(< ~1.5 V). Both off: ambiguous. Refreshed every 100 ms.
