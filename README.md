# WaveMaster 🌊

**ESP32-S3 firmware for fiber-laser galvo controllers, driving an AD3552R dual 16-bit DAC.**

WaveMaster turns an ESP32-S3 into a galvo marking controller. It speaks GRBL 1.1
over USB for Rayforge, turns G-code into a constant-speed trajectory at a fixed
10 us tick, and streams it to the AD3552R as one continuous hardware-paced
quad-SPI transfer. The laser gate is switched in lock-step with the DAC samples.
A companion ATmega328P owns the slow laser lines (power word, arm, guide, status).

---

## 🚦 Onboarding: from a fresh clone to the first mark

Follow the phases in order. Each one ends with a check; don't move on until it passes.
Commands assume macOS and the repo at `~/…/WaveMaster`; on Linux use `/dev/ttyACM0`-style
ports and set up the venv by hand (see [tools/README_FLASHING.md](tools/README_FLASHING.md)).
Below, `ESP` is the ESP32-S3 port, e.g. `/dev/cu.usbmodem1101`.

**0. Safety first.** Laser in its enclosure, interlock working, eye protection at hand. Until
phase 7 the marking laser never fires: every tool below runs in guide-laser preview
(`M66`) unless you explicitly pass `--fire` and type `FIRE`.

**1. Install the tools** (once)
```bash
git clone https://github.com/lollokara/WaveMaster.git && cd WaveMaster
bash tools/setup_mac.sh        # Homebrew/Python check, Rosetta 2 on Apple Silicon, .venv with PlatformIO
```
✔ Ends with "Setup complete" and lists your serial ports.

**2. Wire it** (laser powered off, DB25 unplugged). Read [docs/WIRING.md](docs/WIRING.md):
DB25 pins 19 (gate) and 20 (SYNC/PRR) go to ESP32 GPIO4/GPIO3 with 10 kΩ pull-downs; power
bits D0/D1 move from Arduino A7/A6 to D2/D4; pin 23 (E-stop) through a real interlock.
✔ Every row of the DB25 table checked with a multimeter (continuity, no shorts to 5 V).

**3. Flash both boards**
```bash
.venv/bin/python tools/flash_boards.py      # detect → build → ATmega first → ESP32 → verify
```
✔ `verify` passes: `[VER:1.1h.WaveMaster:]`, `[OPT:V,512,1024]`, `atmega link=1`,
`$RB` readback, and the boot log shows `self-test OK`. It also saves a `$$` backup in
`tools/backups/`. If anything fails, the wizard prints the fix; see the troubleshooting
table in [tools/README_FLASHING.md](tools/README_FLASHING.md).

**4. First contact, no laser**
```bash
.venv/bin/python tools/rftest/run_tests.py --mock smoke   # rehearsal against a simulated board
.venv/bin/python tools/rftest/run_tests.py ESP smoke       # handshake, $$, $S, $RB, a guide square, cancel test
```
✔ `RESULT: PASS`, cancel returns to Idle within 2 s. Reports land in `tools/rftest/runs/`.

**5. Watch the galvo move with the guide laser**
```bash
.venv/bin/python tools/rftest/run_tests.py ESP all-guide   # shapes, frame, raster, 2000-segment stress
```
✔ Smooth, evenly bright red traces (no bright corners, no dots along lines), `underruns 0`
in the report. If the shapes are mirrored or rotated, the next phase fixes it. Optional but
recommended once: put the scope on DB25 pin 19 vs. the DAC output and on SCLK/CS (see the
bring-up checklist in [STATUS.md](STATUS.md)).

**6. Calibrate geometry** (guide laser on paper is enough to start)
```bash
.venv/bin/python tools/calibrate.py ESP                   # galvo range, field size, orientation, scale, distortion, offset
```
(`range` finds how many volts the galvos follow and suggests the work area; `focus`, the first step, needs
`--fire` and is skipped here.)
✔ A drawn 20 mm square measures 20.0 mm on both axes and the crosshair lands on your
reference. Procedure and measuring tips: [tools/CALIBRATION.md](tools/CALIBRATION.md).

**7. First real marks** (test card in the field, enclosure closed, low power)
```bash
.venv/bin/python tools/calibrate.py ESP --fire --max-power 300 --step focus     # set the focal height first
.venv/bin/python tools/calibrate.py ESP --fire --max-power 300 --step delays --step power --step speed
.venv/bin/python tools/rftest/run_tests.py ESP fire-power-ladder --fire --max-power 300
```
Each fire run frames the job with the guide laser and asks you to confirm before it arms.
✔ Line starts and ends are clean (laser on/off delays tuned), the power ladder shows where
marking starts and saturates. Then re-run `calibrate.py ESP --fire --step scale --step distortion`
on a marked card for the best accuracy (the scale is only valid at the focus height found by `focus`; the wizard
also offers the work area that fits the new scale).

**8. Connect Rayforge.** Follow [docs/RAYFORGE.md](docs/RAYFORGE.md): driver "GRBL (Serial)",
dialect `grbl` for vectors or `grbl_raster` for engraving, work area = `$130`×`$131`,
max power = `$30`, homing off. Close Rayforge before using the scripts again (one program per
serial port).
✔ A small vector job from Rayforge marks at the expected size and speed, and `$S` (Rayforge
console) shows `underruns=0`.

**When something goes wrong:** `$S` (stream and ATmega status), `$RB` (DAC readback when
idle), and the logs in `tools/rftest/runs/<run>/` (`session.log` is the full byte-level
transcript). Settings backups can be restored with
`tools/flash_boards.py restore <file>` or `tools/calibrate.py ESP --restore <file>`.

---

## ✨ Features

- **Constant-speed trajectory:** arc-length interpolation at a fixed tick (`$200`, default 10 us). The beam keeps its speed through corners. `$215` adds a hold at sharp turns.
- **Sample-locked laser gate:** the gate (DB25 pin 19) is set from the SPI transaction callback and toggled mid-chunk by a GPTimer alarm, both timed from the same instant as the DAC samples.
- **Jump trapezoids:** `G0` uses a trapezoidal profile (`$201` speed, `$120` acceleration) and a settle delay.
- **EZCAD-style delays:** laser-on / laser-off, jump, mark and polygon-corner delays (`$210`-`$216`).
- **Power modes:** analog power word over the ATmega link (`$223=0`), or pulse-density modulation at fixed power (`$223=1`).
- **GRBL 1.1 over USB or WiFi** (raw TCP, port 23) with character counting, 1024-byte RX window, realtime `?` `!` `~` Ctrl-X and jog cancel.
- **ATmega protocol v2:** CRC-8 framed, with a heartbeat. The ATmega disarms within 1 s if the ESP32 goes silent.
- **Stream statistics:** `$S` reports underruns, barriers and ATmega link state. `$RB` reads back the DAC registers when idle. `$GT` and `$LT` test the gate line and the laser without motion.

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
| `src/net_wifi.{c,h}` | WiFi station manager (core 1), credentials in NVS, the `$WIFI` command. Off unless `$240=1`. |
| `src/net_server.{c,h}` | Raw TCP host link on port 23: one client, keepalive, second transport for `host_serial`. |
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
| `tools/` | Host helpers: `stream_test.py` (character-counting bring-up and tuning patterns, preview unless `--fire`; the port can be `host:port` for WiFi), `jog.py`. |

---

## 📌 Hardware

Pin-level wiring is in **[docs/WIRING.md](docs/WIRING.md)**, with a diagram in
[docs/wiring.svg](docs/wiring.svg). The short version:

- **ESP32-S3 GPIO4** is the laser gate, on DB25 pin 19. **GPIO3** is SYNC / PRR, on DB25 pin 20.
- **ESP32-S3 GPIO5-13** drive the AD3552R (SCK, CS, QSPI select, DIR, RST, D0-D3).
- **ESP32-S3 GPIO44 / GPIO43** are UART0 TX / RX to the ATmega, through a level shifter.
- **ATmega328P** drives the power word (DB25 pins 1-9), arm (pin 18), guide (pin 22), and E-stop (pin 23, held high). It reads status on pins 11, 12, 16 and 17.

Setting up Rayforge is in **[docs/RAYFORGE.md](docs/RAYFORGE.md)**.

Calibrating the galvo (focus, range, orientation, scale, distortion, offset, delays, power, prr, speed) is guided by `python3 tools/calibrate.py PORT`; the procedure is in **[tools/CALIBRATION.md](tools/CALIBRATION.md)**.

---

## 🛠️ Settings (`$$`)

Settings persist in NVS and load at boot. Read them with `$$`, and change one with
`$<n>=<value>`. Values are floats, flags are 0/1. A value outside its range is rejected.
The same table is in `src/calib.h` (comments) and `src/calib.c` (`s_defs`, the source of truth).
A number in brackets is an alias that writes the same value.

| `$` | Meaning | Default | Range | NVS key |
|---|---|---|---|---|
| `$3` | Axis invert mask: bit 0 inverts X, bit 1 inverts Y. | 0 | 0 … 3 | `axinv` |
| `$12` | Arc tolerance, mm: the chord error allowed when G2/G3 arcs are split into lines. | 0.01 | 0.0005 … 1 | `arctol` |
| `$30` | S value that means 100 % power (Rayforge's max power). | 1000 | 1 … 100000 | `smax` |
| `$100` | X scale, volts per mm of work area. | 0.1 | -100 … 100 | `xvpm` |
| `$101` | Y scale, volts per mm of work area. | 0.1 | -100 … 100 | `yvpm` |
| `$110` (`$111`) | Maximum marking speed, mm/min (`$111` writes the same value). | 300000 | 1 … 3000000 | `maxrate` |
| `$120` (`$121`) | Jump acceleration, mm/s²; 0 means no ramp (`$121` writes the same value). | 2000000 | 0 … 1e9 | `jaccel` |
| `$130` | Work area width, mm. | 100 | 1 … 1000 | `fieldw` |
| `$131` | Work area height, mm. | 100 | 1 … 1000 | `fieldh` |
| `$140` | X offset, volts. | 0 | -10 … 10 | `xoff` |
| `$141` | Y offset, volts. | 0 | -10 … 10 | `yoff` |
| `$142` | Radial (F-theta) lens correction k1: r' = r(1 + k1·r²). | 0 | -1 … 1 | `radk1` |
| `$143` | Swap the X and Y axes (0/1). | 0 | 0 … 1 | `swapxy` |
| `$144` | Origin: 0 = work area corner, (0,0)..(W,H) centred on the lens; 1 = (0,0) is the lens centre. | 0 | 0 … 1 | `origc` |
| `$150` | Wobble diameter, mm; 0 = off. | 0 | 0 … 10 | `wobd` |
| `$151` | Wobble pitch, mm per revolution. | 0.5 | 0.001 … 100 | `wobp` |
| `$200` | Tick period, µs: one DAC point per tick (sets the point rate). | 10 | 2 … 200 | `tickus` |
| `$201` | Jump (G0) speed, mm/s. | 3000 | 1 … 100000 | `jspeed` |
| `$203` | Marking speed used when no F was given, mm/s. | 500 | 0.1 … 100000 | `dmspeed` |
| `$210` | Laser-on delay, µs. | 100 | 0 … 100000 | `laserond` |
| `$211` | Laser-off delay, µs. | 120 | 0 … 100000 | `laseroffd` |
| `$212` | Jump delay (settle after a jump), µs. | 300 | 0 … 1000000 | `jumpdly` |
| `$213` | Extra jump delay per mm of jump, µs/mm. | 0 | 0 … 100000 | `jdlypmm` |
| `$214` | Mark delay: hold at the end of a polyline, µs. | 100 | 0 … 1000000 | `markdly` |
| `$215` | Polygon delay at a 180° corner, µs. | 0 | 0 … 100000 | `polydly` |
| `$216` | Polygon delay angle threshold, degrees. | 30 | 0 … 180 | `polyang` |
| `$220` | Pulse repetition rate (SYNC, DB25 pin 20), Hz. | 30000 | 0 … 2000000 | `prrhz` |
| `$221` | SYNC duty cycle, %. | 50 | 1 … 99 | `prrduty` |
| `$223` | Power mode: 0 = analog power word over the ATmega link, 1 = pulse density at fixed power. | 0 | 0 … 1 | `pwrmode` |
| `$224` | Power at S = 0+ (minimum), %. | 0 | 0 … 100 | `pwrmin` |
| `$225` | Power at S = `$30` (maximum), %. | 100 | 0 … 100 | `pwrmax` |
| `$226` | Auto-arm when the first mark needs the laser (0/1). | 1 | 0 … 1 | `autoarm` |
| `$227` | Maximum wait for arm / ready, ms. | 4000 | 0 … 60000 | `armtmo` |
| `$229` | Pulse-density period, ticks (used when `$223=1`). | 10 | 2 … 255 | `pdper` |
| `$230` | Gate (EMISSION MODULATION) active low (0/1). Check `docs/WIRING.md` before changing. | 0 | 0 … 1 | `gatelow` |
| `$231` | Auto guide laser for framing: guide on while moving with `M3 S0` (Rayforge's Frame), off at `M5` / `M4` / S>0 (0/1). | 1 | 0 … 1 | `autoguide` |
| `$240` | WiFi host link on (0/1). Needs credentials from `$WIFI=ssid,password`. See the WiFi section. | 0 | 0 … 1 | `wifien` |

Read-only GRBL values (`$0`, `$1`, `$2`, `$4`-`$6`, `$10`, `$11`, `$20`-`$27`, `$31`, `$32`, `$102`, `$112`, `$122`, `$132`) are reported for sender compatibility and ignored when set.

---

## 📶 WiFi host link

The controller can also talk GRBL over WiFi, so Rayforge connects without a cable. It is the same protocol as
USB, carried on a raw TCP socket (port 23), and Rayforge uses its stock **GRBL (Telnet)** driver. Plan and
reasons: [docs/WIFI_PLAN.md](docs/WIFI_PLAN.md). Rayforge setup: [docs/RAYFORGE.md](docs/RAYFORGE.md).

Set it up once over USB:

```
$WIFI=MyNetwork,my-password     store the network (WPA2 or better, password 8..63 characters)
$240=1                          turn WiFi on (off by default)
$WIFI                           status: [WIFI:enabled=1 creds=1 state=connected ssid=MyNetwork ip=192.168.1.50 ...]
$WIFI=CLEAR                     forget the credentials
```

`!`, `?` and `~` cannot be typed in a `$` line: GRBL acts on them as realtime bytes (feed hold, status, resume) wherever
they appear, so `$WIFI=Casa!,secret` arrives as `Casa` plus a feed hold. If the SSID or the password contains one of
them, send both parts hex-encoded: `$WIFIH=<hex ssid>,<hex password>`, for example
`python3 -c "print('Casa!'.encode().hex(), 'my?pass~word'.encode().hex())"`.

The SSID is split at the first comma, so it cannot contain one. The password is never printed or logged.
Quick check from a shell: `nc <ip> 23`, then type `$I`. Streaming test: `python3 tools/stream_test.py <ip>:23 grid`.

Rules to know:

- **One host at a time.** USB and WiFi feed the same stream. Replies go to the host that spoke last while the machine
  was idle. While a job runs, or within 1 s of the active host's last G-code line, input from the other host is
  ignored, except **Ctrl-X**, which always works and never waits for a lock.
- **Network settings only over USB.** `$WIFI=ssid,password` and `$WIFI=CLEAR` are refused from a WiFi client (the
  password would travel in clear). `$WIFI` alone (status) works from either host.
- **Losing the WiFi client stops the job.** If the active TCP client disappears, the firmware does what Ctrl-X does:
  stop, gate off, disarm. USB becomes the active host again. This happens at once when WiFi is switched off or the
  access point says goodbye. If the access point or the client just disappears, it takes as long as the WiFi driver or
  TCP keepalive needs to notice (about 6 s or more), and the job keeps marking until then. A client that stops
  reading replies for 200 ms is treated as lost.
- **No authentication.** Anyone on the same network who can reach port 23 can send G-code. Keep the controller on a
  trusted or separate network. The laser still has to be armed, and the ATmega interlocks are unchanged.
- Settings changes (`$240`, credentials) apply within about a second, no reboot needed.
- Not verified on hardware yet: see [STATUS.md](STATUS.md).

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
