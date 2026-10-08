# Arduino Laser Control

This is the Arduino Mini firmware for controlling the IPG laser via serial commands at 250000 baud. Replaces the BugBuster DB25 parallel interface.

## Hardware Setup

### Pinout (Laser Pin → Arduino Pin)

**Power Bits (8-bit parallel word D0-D7):**
- D0 → A7
- D1 → A6
- D2 → A4
- D3 → A2
- D4 → A1
- D5 → D13
- D6 → D3
- D7 → D5

**Control Lines:**
- EMISSION ENABLE (Arm) → A0
- EMISSION MOD (PWM) → D2
- GUIDE LASER → D7
- LATCH → D6
- SYNC (PWM) → D4
- AUX OFF → D8

**Status Inputs:**
- 5V DETECT → A5
- STAT 11 → D9
- STAT 12 → D10
- STAT 16 → A3

**LEDs:**
- RED LED (5V present) → D11
- GREEN LED (0V present) → D12

> [!WARNING]
> **ATmega328P Pin Limitation (A6 / A7):**
> On standard ATmega328P boards (Arduino Mini / Pro Mini / Nano), pins **A6** and **A7** are dedicated analog-only input channels (ADC6 / ADC7) and do not have digital I/O register hardware (`PORT`/`DDR`).
> Although `pinMode(OUTPUT)` and `digitalWrite()` compile without error in the Arduino framework, they have no physical effect. Power bits `D0` and `D1` cannot be driven digitally on standard hardware unless reassigned to digital GPIOs.

## Building & Uploading

### Prerequisites
- PlatformIO CLI or IDE
- Arduino Mini (ATmega328P) hardware

### Build
```bash
pio run -e arduino_mini
```

### Upload
```bash
pio run -e arduino_mini -t upload
```

### Monitor
```bash
pio device monitor -b 250000
```

## Serial Protocol

Commands are sent over UART at 250000 baud as binary packets. Each command consists of an opcode byte followed by optional parameter bytes.

### Commands

#### 0x01: SET_ARM
Arm/disarm the laser (raise EMISSION_ENABLE pin).
- Bytes: `0x01 <0|1>`
- Response: ACK (0x01)

#### 0x02: SET_FIRING
Fire the laser (control EMISSION_MOD pin).
- Bytes: `0x02 <0|1>`
- Response: ACK (0x02)

#### 0x03: SET_GUIDE
Enable/disable guide laser.
- Bytes: `0x03 <0|1>`
- Response: ACK (0x03)

#### 0x04: SET_POWER
Set laser power (0-255, 8-bit parallel word on D0-D7).
- Bytes: `0x04 <0-255>`
- Response: ACK (0x04)

#### 0x05: SET_PWM_EMIT
Set emission modulation PWM frequency and duty cycle.
- Bytes: `0x05 <freq_h> <freq_l> <duty>`
- Frequency: 1-8000 Hz (16-bit big-endian)
- Duty: 0-255 (0-100%)
- Response: ACK (0x05)

#### 0x06: SET_PWM_SYNC
Set sync signal PWM frequency and duty cycle.
- Bytes: `0x06 <freq_h> <freq_l> <duty>`
- Frequency: 1-8000 Hz (16-bit big-endian)
- Duty: 0-255 (0-100%)
- Response: ACK (0x06)

#### 0x07: GET_STATUS
Read current status from Arduino.
- Bytes: `0x07`
- Response: 8 bytes
  - Byte 0: Armed (0/1)
  - Byte 1: Firing (0/1)
  - Byte 2: Guide laser on (0/1)
  - Byte 3: Power value (0-255)
  - Byte 4: STAT_11 pin state
  - Byte 5: STAT_12 pin state
  - Byte 6: STAT_16 pin state
  - Byte 7: VCC voltage (high byte of mV)

#### 0x08: PULSE_LATCH
Pulse the latch pin to commit power bits.
- Bytes: `0x08`
- Response: ACK (0x08)

## LED Indicators

**Red LED (D11):** 5V present (high analog reading on 5V_DETECT)
**Green LED (D12):** 0V present (low analog reading on 5V_DETECT)
Both off: Ambiguous/transition state

## PWM Implementation

PWM on D2 (EMISSION_MOD) and D4 (SYNC) is implemented via software using Timer 1 interrupt (~8kHz). The frequency range is 1-8000 Hz with 8-bit duty cycle control (0-255).

Timer 0 could be used for additional PWM on D5/D6 if needed in future versions.

## Status Polling

The Python app polls status at ~1 Hz to monitor:
- Arm/firing/guide state
- Power setting
- Status pin states (STAT_11, STAT_12, STAT_16)
- VCC voltage

Status is displayed in the "Arduino status" panel in the control app.
