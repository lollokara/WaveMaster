# WaveMaster 🌊

**High-Performance ESP32-S3 Firmware for Fiber-Laser Galvano Controllers (AD3552R Dual DAC)**

WaveMaster turns an **ESP32-S3** into a real-time, dual-core fiber laser galvo controller. It drives an ultra-low latency **Analog Devices AD3552R dual 16-bit DAC** over high-speed SPI for galvanometer mirror positioning, exposes a **GRBL 1.1-compatible serial interface** for host senders (e.g., LightBurn, Rayforge, LaserWeb), and manages laser safety, setpoints, and arming alongside a companion **ATmega328P** controller.

---

## ⚡ Key Architecture & Highlights

* **Dual-Core Architecture (FreeRTOS)**:
  * **Core 0 (`dac_task`)**: Dedicated exclusively to DAC hardware control, SPI transfers, high-speed buffered streaming, look-ahead path pacing, laser firing (`Engrave GPIO`), and timing delays. Unsubscribes idle task and feeds task watchdog directly (`esp_task_wdt_reset`) to ensure jitter-free marking and previews.
  * **Core 1 (`grbl_task` & `atmega_task`)**: Handles G-code parsing, kinematics (trapezoidal planner, junction deviation, arc interpolation), calibration, and asynchronous background communication with the ATmega companion.
* **Analog Devices AD3552R 16-bit Dual DAC**:
  * Dual-channel ±10V / ±5V differential output for X and Y galvanometer servo amplifiers.
  * Supports single-point writes and high-throughput hardware-paced SPI streaming.
* **GRBL 1.1 Protocol Compatibility**:
  * Speaks standard G-code (`G0`, `G1`, `G2`, `G3`, `G4`, `G90`, `G91`, `G92`).
  * Realtime commands (`?`, `!`, `~`, `Ctrl-X`) and setting management (`$$`).
  * Seamless laser-mode semantics (`$32=1` convention): `G0` automatically inhibits laser firing during rapid traversing; `G1/G2/G3` mark.
* **Industrial Galvo Marking Features**:
  * Configurable settle delays: **Jump Delay**, **Mark Delay**, **Polygon Delay**, **Laser-On Delay**, and **Laser-Off Delay** (`$130`-`$134`).
  * F-theta lens radial correction ($k_1$ parameter via `$140`).
  * Wobble mark / kerf-widening generator (`$150` diameter, `$151` pitch).
  * Skywriting acceleration margin (`$160`).
  * Hardware boundary preview loop (`M68`).
  * Closed-polygon hatch-fill generator (`M64` / `M65`).

---

## 📌 Hardware Pinouts & Wiring

### 1. ESP32-S3 Pinout

| ESP32-S3 GPIO | Function / Signal | Connected To / Description |
|:---:|:---|:---|
| **GPIO 5** | `BOARD_PIN_SCK` | AD3552R SPI Serial Clock (SCK) |
| **GPIO 6** | `BOARD_PIN_CS` | AD3552R SPI Chip Select (CS) |
| **GPIO 7** | `BOARD_PIN_SPI_OP1` | AD3552R QSPI Mode Pin (Low = Classic/Dual, High = Quad) |
| **GPIO 8** | `BOARD_PIN_DIR` | AD3552R SPI Direction Control Pin |
| **GPIO 9** | `BOARD_PIN_RST` | AD3552R Hardware Reset (RST) |
| **GPIO 10** | `BOARD_PIN_D3` | AD3552R Data Lane 3 / QSPI |
| **GPIO 11** | `BOARD_PIN_D2` | AD3552R Data Lane 2 / QSPI |
| **GPIO 12** | `BOARD_PIN_D1` | AD3552R Data Lane 1 / SDO |
| **GPIO 13** | `BOARD_PIN_D0` | AD3552R Data Lane 0 / SDI |
| **GPIO 3** | `BOARD_PIN_PRR` | Laser Pulse Repetition Rate (PWM out, via LEDC) |
| **GPIO 4** | `BOARD_PIN_ENGRAVE` | Laser Firing Trigger Output (High = Firing) |
| **GPIO 44** | `UART0 TX` | Link to ATmega328P RX (via Level Shifter) |
| **GPIO 43** | `UART0 RX` | Link to ATmega328P TX (via Level Shifter) |
| **Native USB** | `USB-Serial-JTAG` | PC / Host G-Code & Monitor Connection (115200 baud) |

> [!NOTE]
> *The ATmega328P link runs at **250,000 baud 8N1** across a logic level shifter. In firmware, UART0 pins are explicitly assigned as **GPIO44 (TX)** and **GPIO43 (RX)**.*

---

### 2. Arduino Companion (ATmega328P) Pinout

Located in [`ArduinoCompanion/`](ArduinoCompanion/), this microcontroller offloads 8-bit parallel digital power level setting, safety arming sequences, and auxiliary laser control.

| ATmega Pin | Signal Name | Type | Description |
|:---:|:---|:---:|:---|
| **A7** | `PIN_D0` | Output | Power Bit 0 (LSB) to Fiber Laser Source |
| **A6** | `PIN_D1` | Output | Power Bit 1 |
| **A4** | `PIN_D2` | Output | Power Bit 2 |
| **A2** | `PIN_D3` | Output | Power Bit 3 |
| **A1** | `PIN_D4` | Output | Power Bit 4 |
| **13** | `PIN_D5` | Output | Power Bit 5 |
| **3** | `PIN_D6` | Output | Power Bit 6 |
| **5** | `PIN_D7` | Output | Power Bit 7 (MSB) |
| **6** | `PIN_LATCH` | Output | Latch pulse for power data |
| **A0** | `PIN_EMISSION_ENABLE` | Output | System Arm / Emission Enable relay/line |
| **2** | `PIN_EMISSION_MOD` | Output | Emission modulation signal (PWM) |
| **4** | `PIN_SYNC` | Output | Laser sync signal (PWM) |
| **7** | `PIN_GUIDE_LASER` | Output | Visible Red Guide Laser On/Off |
| **8** | `PIN_AUX_OFF` | Output | Auxiliary safety power cut (Default HIGH) |
| **11** | `PIN_RED_LED` | Output | Armed indicator LED |
| **12** | `PIN_GREEN_LED` | Output | Standby / Ready indicator LED |
| **A5** | `PIN_5V_DETECT` | Input | 5V rail voltage sense (analog) |
| **9** | `PIN_STAT_11` | Input | Status line from laser module |
| **10** | `PIN_STAT_12` | Input | Status line from laser module |
| **A3** | `PIN_STAT_16` | Input | Status line from laser module |
| **D0 (RX)** | `UART RX` | Input | Serial from ESP32 GPIO 44 (250 kBaud) |
| **D1 (TX)** | `UART TX` | Output | Serial to ESP32 GPIO 43 (250 kBaud) |

---

## 🛠️ Calibration & Parameters (`$$`)

All calibration settings are stored in ESP32 NVS flash and persist across power cycles. Values can be queried with `$$` and changed via `$<id>=<val>`.

| Setting | Name | Unit | Default | Description |
|:---|:---|:---:|:---:|:---|
| **`$100`** | X Volts/mm | V/mm | `1.0` | Output scale for X galvanometer axis |
| **`$101`** | Y Volts/mm | V/mm | `1.0` | Output scale for Y galvanometer axis |
| **`$110`** | X Offset | V | `0.0` | Voltage center offset for X galvo |
| **`$111`** | Y Offset | V | `0.0` | Voltage center offset for Y galvo |
| **`$130`** | Jump Delay | $\mu\text{s}$ | `250` | Galvo settle time after rapid traversal (`G0`) |
| **`$131`** | Mark Delay | $\mu\text{s}$ | `150` | Settle time at the end of a marking stroke |
| **`$132`** | Polygon Delay | $\mu\text{s}$ | `50` | Extra settle delay on stroke corner vertices |
| **`$133`** | Laser-On Delay | $\mu\text{s}$ | `100` | Delay after asserting laser fire before moving |
| **`$134`** | Laser-Off Delay| $\mu\text{s}$ | `100` | Delay after deasserting laser fire |
| **`$140`** | Radial Corr $k_1$ | — | `0.0` | $r' = r(1 + k_1 r^2)$ barrel/pincushion F-theta correction |
| **`$150`** | Wobble Diameter | mm | `0.0` | Beam wobble diameter (`0` = disabled) |
| **`$151`** | Wobble Pitch | mm | `0.1` | Forward distance per wobble cycle |
| **`$160`** | Skywriting Margin| mm | `0.0` | Traversal run-in/run-out margin with laser off |
| **`$170`** | Hatch Spacing | mm | `0.1` | Scanline spacing for `M64`/`M65` polygon fills |
| **`$171`** | Hatch Angle | deg | `0.0` | Scanline angle for hatch fills |
| **`$180`** | Acceleration | $\text{mm/s}^2$ | `10000` | Look-ahead motion planner maximum acceleration |
| **`$181`** | Junction Deviation| mm | `0.02` | Cornering tolerance factor |
| **`$182`** | Max Velocity | mm/s | `2000` | Hard cap on trajectory velocity |

---

## 📡 Custom M-Codes & Laser Protocol

| Code | Parameters | Description |
|:---|:---|:---|
| **`M3` / `M4`** | `S<0-1000>` | Laser marking mode enabled; sets power (0-100% mapped to ATmega) |
| **`M5`** | — | Laser marking mode off (Engrave disabled) |
| **`M10`** | — | **Arm Laser**: Initiates 2-second safety arm sequence on ATmega companion |
| **`M11`** | — | **Disarm Laser**: De-asserts emission enable line |
| **`M62`** | — | Turn **Red Guide Laser ON** |
| **`M63`** | — | Turn **Red Guide Laser OFF** |
| **`M64`** | — | Start recording closed polygon for hardware hatch-filling |
| **`M65`** | — | Finish polygon and execute hatch fill pattern |
| **`M66`** | — | Enter **Preview Mode** (guide laser ON, main laser forced OFF) |
| **`M67`** | — | Exit **Preview Mode** |
| **`M68`** | `P<Hz>` | Replay recorded outline continuously on galvo (boundary preview) |
| **`Q<Hz>`** | `<Hz>` | Configure laser Pulse Repetition Rate (PWM frequency on GPIO 3) |
| **`$RB`** | — | Debug command: Read back raw AD3552R DAC output registers over SPI |

---

## 🚀 Building & Flashing

### WaveMaster (ESP32-S3)
Using [PlatformIO](https://platformio.org/):

```bash
# Build firmware
pio run

# Flash to ESP32-S3 and launch serial monitor
pio run -t upload -t monitor
```

### Arduino Companion (ATmega328P)
```bash
cd ArduinoCompanion
pio run -t upload
```

---

## 📁 Repository Structure

```
├── CMakeLists.txt                 # ESP-IDF CMake definition
├── platformio.ini                 # PlatformIO configuration (esp32-s3-devkitc-1)
├── sdkconfig.defaults             # ESP-IDF configurations (USB-Serial-JTAG, TWDT)
├── components/
│   └── ad3552r/                   # Ported Analog Devices no-OS AD3552R driver + ESP32 SPI/GPIO HAL
├── src/
│   ├── main.c                     # System entrypoint, boot sequence, log multiplexer
│   ├── ad3552r_board.{c,h}        # AD3552R hardware init, QSPI setup, volts/code conversion
│   ├── dac_task.{c,h}             # Core 0: Real-time DAC SPI writes, streaming, delays
│   ├── grbl_task.{c,h}            # Core 1: G-code interpreter, status responses, laser logic
│   ├── motion_planner.{c,h}       # Kinematics lookahead planner, junction deviation
│   ├── atmega_link.{c,h}          # Asynchronous serial client to ATmega companion
│   ├── calib.{c,h}                # NVS-backed optical & timing calibration
│   ├── laser_ctrl.{c,h}           # Inter-core FreeRTOS command queues & unit conversions
│   ├── laser_fill.{c,h}           # Scanline polygon hatch-fill geometry generator
│   ├── engrave_gpio.{c,h}         # Laser trigger output GPIO control
│   └── prr_pwm.{c,h}              # LEDC PWM pulse-repetition-rate generator
├── ArduinoCompanion/              # Standalone ATmega328P companion firmware
│   ├── platformio.ini
│   ├── README.md
│   └── src/main.cpp
└── tools/
    ├── jog.py                     # Interactive CLI jogging utility
    └── square_loop.py             # Boundary test utility for preview/tuning
```

---

## 📜 License

Distributed under the MIT License. Feel free to use, adapt, and build upon this project.
