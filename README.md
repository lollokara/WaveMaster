# WaveMaster 🌊

**ESP32-S3 firmware for fiber-laser galvo controllers, driving an AD3552R dual 16-bit DAC.**

WaveMaster turns an ESP32-S3 into a galvo marking controller. It speaks GRBL 1.1
over USB for Rayforge, turns G-code into a constant-speed trajectory at a fixed
10 us tick, and streams it to the AD3552R as one continuous hardware-paced
quad-SPI transfer. The laser gate is switched in lock-step with the DAC samples.
A companion ATmega328P owns the slow laser lines (power word, arm, guide, status).

---

## ✨ Features

- **Constant-speed trajectory:** arc-length interpolation at a fixed tick (`$200`, default 10 us). The beam keeps its speed through corners. `$215` adds a hold at sharp turns.
- **Sample-locked laser gate:** the gate (DB25 pin 19) is set from the SPI transaction callback and toggled mid-chunk by a GPTimer alarm, both timed from the same instant as the DAC samples.
- **Jump trapezoids:** `G0` uses a trapezoidal profile (`$201` speed, `$120` acceleration) and a settle delay.
- **EZCAD-style delays:** laser-on / laser-off, jump, mark and polygon-corner delays (`$210`-`$216`).
- **Power modes:** analog power word over the ATmega link (`$223=0`), or pulse-density modulation at fixed power (`$223=1`).
- **GRBL 1.1 over USB** with character counting, 1024-byte RX window, realtime `?` `!` `~` Ctrl-X and jog cancel.
- **ATmega protocol v2:** CRC-8 framed, with a heartbeat. The ATmega disarms within 1 s if the ESP32 goes silent.
- **Stream statistics:** `$S` reports underruns, barriers and ATmega link state. `$RB` reads back the DAC registers when idle.

---

## 🧭 Pipeline

```
Rayforge --USB--> host_rx (core 1, prio 10) --> grbl (prio 9) --> motion_task (prio 8)
                                                                    | motion generator: one (x, y, power) per tick
                                                                    v
                                              galvo_out: mm -> DAC codes, gate edges, power barriers
                                                                    | chunk pool: 8 x 2048 ticks (12 KB DMA each)
                                                                    v
                    dac_task (core 0, prio 10): quad-SPI streaming, gate ISR + GPTimer
                                                                    |
                                                                    v
                                              AD3552R --> galvo X/Y drivers

atmega_link (core 1, prio 5) <--UART0 250k--> ATmega328P --> power word, arm, guide, status
```

Flow control is end to end. `motion_submit()` blocks while the segment queue is full. `grbl` then withholds `ok`, and the host's character counting pauses.

---

## 📦 Modules

| Path | Role |
|:---|:---|
| `src/main.c` | Boot sequence: early gate-off, NVS, UART, DAC, motion, GRBL. |
| `src/host_serial.{c,h}` | USB-Serial-JTAG driver, host RX task, realtime bytes, `[MSG:...]` logging. |
| `src/grbl.{c,h}` | GRBL 1.1 front end: line parser, modal state, `$` commands, status reports, reset handling. |
| `src/gcode_parse.{c,h}` | Word parser, arc linearisation, feed and power mapping. |
| `src/calib.{c,h}` | Settings table (`s_defs`), NVS persistence, `$$` dump. |
| `src/motion.{c,h}` | Pure-C trajectory generator (ticks). No ESP-IDF dependencies, host-testable. |
| `src/motion_task.{c,h}` | Core 1 task that owns the segment queue and the generator. Underrun policy. |
| `src/galvo_out.{c,h}` | Tick to DAC codes, gate timing, power modes, chunking, abort and feed hold. |
| `src/galvo_chunk.h` | Private chunk structure shared by `galvo_out` and `dac_task`. |
| `src/dac_task.{c,h}` | Core 0 owner of SPI and the AD3552R: streaming, gate ISR and timer, barriers, readback. |
| `src/laser_io.{c,h}` | GPIO4 gate (IRAM-safe, kill switch) and GPIO3 SYNC (LEDC). |
| `src/ad3552r_board.{c,h}` | AD3552R bring-up, pin map, quad-mode switching, volts-to-code conversion. |
| `src/atmega_link.{c,h}` | Protocol v2 client for the ATmega: power, arm, guide, status, heartbeat. |
| `components/ad3552r/` | Ported no-OS AD3552R driver with ESP-IDF SPI, GPIO and CRC shims. |
| `ArduinoCompanion/` | ATmega328P firmware: power word, arm, guide, status inputs, watchdog. See its [README](ArduinoCompanion/README.md). |
| `test/host/` | Host unit tests for the trajectory generator. |
| `test/host/gcode/` | Host unit tests for the G-code parser. |
| `tools/` | Host helpers: `stream_test.py` (character-counting bring-up and tuning patterns, preview unless `--fire`), `jog.py`. |

---

## 📌 Hardware

Pin-level wiring is in **[docs/WIRING.md](docs/WIRING.md)**, with a diagram in
[docs/wiring.svg](docs/wiring.svg). The short version:

- **ESP32-S3 GPIO4** is the laser gate, on DB25 pin 19. **GPIO3** is SYNC / PRR, on DB25 pin 20.
- **ESP32-S3 GPIO5-13** drive the AD3552R (SCK, CS, QSPI select, DIR, RST, D0-D3).
- **ESP32-S3 GPIO44 / GPIO43** are UART0 TX / RX to the ATmega, through a level shifter.
- **ATmega328P** drives the power word (DB25 pins 1-9), arm (pin 18), guide (pin 22), and E-stop (pin 23, held high). It reads status on pins 11, 12, 16 and 17.

Setting up Rayforge is in **[docs/RAYFORGE.md](docs/RAYFORGE.md)**.

Calibrating the galvo (orientation, scale, distortion, offset, delays, power, speed) is guided by `python3 tools/calibrate.py PORT`; the procedure is in **[tools/CALIBRATION.md](tools/CALIBRATION.md)**.

---

## 🛠️ Settings (`$$`)

Settings persist in NVS and load at boot. Read them with `$$`, and change one with
`$<n>=<value>`. The table below is copied verbatim from `src/calib.h`. Valid ranges
are in the `s_defs` table in `src/calib.c`.

```
$3    axis invert mask: bit0 = invert X, bit1 = invert Y        [0]
$12   arc tolerance, mm (G2/G3 chord error)                    [0.01]
$13   report inches - fixed 0 (read-only)
$30   S value for 100% power                                   [1000]
$31   S value for 0% power - fixed 0 (read-only)
$32   laser mode - fixed 1 (read-only)
$100  X scale, volts per mm                                    [0.1]
$101  Y scale, volts per mm                                    [0.1]
$110  max marking speed, mm/min ($111 is an alias)             [300000]
$120  jump acceleration, mm/s^2, 0 = no ramp ($121 alias)      [2000000]
$130  work area width, mm                                      [100]
$131  work area height, mm                                     [100]
$140  X offset, volts                                          [0]
$141  Y offset, volts                                          [0]
$142  radial (F-theta) correction k1: r' = r(1 + k1 r^2)       [0]
$143  swap X/Y axes (0/1)                                      [0]
$144  origin: 0 = work area corner (0,0)..(W,H) centred on the
      lens; 1 = (0,0) is the lens centre                       [0]
$150  wobble diameter, mm, 0 = off                             [0]
$151  wobble pitch, mm per revolution                          [0.5]
$200  tick period, us (one DAC point per tick)                 [10]
$201  jump speed, mm/s                                         [3000]
$203  default marking speed when no F was given, mm/s         [500]
$210  laser-on delay, us                                       [100]
$211  laser-off delay, us                                      [120]
$212  jump delay, us                                           [300]
$213  extra jump delay per mm of jump, us/mm                   [0]
$214  mark delay (hold at end of a polyline), us               [100]
$215  polygon delay at a 180 deg corner, us                    [0]
$216  polygon delay angle threshold, degrees                   [30]
$220  pulse repetition rate (SYNC), Hz                         [30000]
$221  SYNC duty cycle, %                                       [50]
$223  power mode: 0 analog (ATmega word), 1 pulse density      [0]
$224  power at S = 0+ (min), %                                 [0]
$225  power at S = $30 (max), %                                [100]
$226  auto-arm when the first mark needs the laser (0/1)        [1]
$227  max wait for arm/ready, ms                               [4000]
$229  pulse-density period, ticks                              [10]
$230  gate (EMISSION MODULATION) active low (0/1)              [0]
```

Read-only GRBL values (`$0`, `$1`, `$2`, `$4`-`$6`, `$10`, `$11`, `$20`-`$27`, `$31`, `$32`, `$102`, `$112`, `$122`, `$132`) are reported for sender compatibility and ignored when set.

---

## 📡 G-code and custom M-codes

| Code | Description |
|:---|:---|
| `G0` / `G1` | Travel (jump, laser off) / mark at `F` mm/min. Marking speed is capped at `$110`. |
| `G2` / `G3` | Arcs, linearised to the `$12` tolerance. |
| `G4 P<s>` | Dwell. |
| `G10 L2` / `L20` | Set work coordinate offsets. |
| `G20` / `G21` | Inches / millimetres. |
| `G53`, `G54`-`G59`, `G90`, `G91`, `G92` (`G92.1`-`G92.3` clear it) | Machine coordinates, work offsets, absolute / relative, coordinate offset. |
| `M3` / `M4` / `M5` | Laser mode on (`M4` and `M3` set power from `S`) / off. `S` maps to power through `$30`, `$224` and `$225`. |
| **`M10`** / **`M11`** | **Arm** / disarm the laser (ATmega emission enable, with a 2 s settle). |
| **`M62`** / **`M63`** | Guide (red) laser on / off. |
| **`M66`** / **`M67`** | **Preview** on (guide on, marking laser forced off, real trajectory) / off. |
| `M0`, `M1`, `M2`, `M6`-`M9`, `M30`, `M64`, `M65`, `M68`, `M69` | Accepted. Most are no-ops. `M2` and `M30` end the program. |
| `$J=` | Jog. A jog cancel (0x85) stops motion. |
| `$H` | Accepted, does nothing (no homing). |
| `$RB` | Read back the AD3552R output registers (idle only). |
| `$S` | Stream and ATmega link statistics. |
| `$I` | Version and buffer sizes. |
| `?`, `!`, `~`, Ctrl-X | Status, feed hold, resume, soft reset (laser off immediately). |

---

## Flashing (macOS)

Guided setup and flashing for both boards (ATmega companion first, then ESP32-S3):

```bash
bash tools/setup_mac.sh
.venv/bin/python tools/flash_boards.py
```

See [tools/README_FLASHING.md](tools/README_FLASHING.md) for safety steps, subcommands and troubleshooting.

---

## 🚀 Build and flash

Requires [PlatformIO](https://platformio.org/). The platform is pinned to
`espressif32@6.13.0` (ESP-IDF 5.5), in `platformio.ini`.

```bash
# ESP32-S3 firmware
pio run
pio run -t upload -t monitor

# ATmega328P companion (flash before reconnecting the laser; see docs/WIRING.md)
cd ArduinoCompanion && pio run -t upload

# Host unit tests
make -C test/host              # trajectory generator
make -C test/host/gcode        # G-code parser
```

`sdkconfig.defaults` selects the USB-Serial-JTAG console, sets `CONFIG_FREERTOS_HZ=1000`, and keeps the ISR code that the stream path uses in IRAM.

---

## 📜 License

Distributed under the MIT License, as stated in the previous README. There is no `LICENSE` file in this repository yet.
