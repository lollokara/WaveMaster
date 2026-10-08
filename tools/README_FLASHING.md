# Flashing WaveMaster from a Mac

Two boards, flashed in this order: the **ATmega328P companion** (Arduino Mini/Nano, CH340 USB-serial)
first, then the **ESP32-S3** (native USB-Serial-JTAG).

## 1. Prerequisites

- macOS with Terminal. Data-capable USB cables for both boards.
- Homebrew and Python 3.10+ (the setup script offers to install them).

```bash
bash tools/setup_mac.sh
```

This is idempotent. It creates `./.venv` with `platformio>=6.1.16` and `pyserial`, pre-downloads
both toolchains, adds `.venv/` and `tools/backups/` to `.gitignore`, and lists detected serial ports.

## 2. Safety before flashing

- **Laser OFF.** Power the laser supply down and **unplug the DB25 cable** (or at least have the laser powered down).
- Flash the ATmega **first**. The old companion firmware drives D2/D4 as PWM outputs; the new wiring moves
  DB25 pins 19/20 to ESP32 GPIO4/GPIO3. Never connect the laser while the old firmware is still on the ATmega.
- The ESP32 may stay connected during the ATmega flash.
- The ATmega shares its UART with the ESP32 link through a level shifter, and the CH340 sits in parallel on
  those lines. After flashing, **close any serial monitor** on the Arduino's USB port, or unplug its USB
  (keep it plugged only if you need it to power the board). Otherwise the ATmega cannot reach the ESP32 link reliably.
- Close Rayforge/LightBurn and any other app using the serial ports.

## 3. Run the wizard

```bash
.venv/bin/python tools/flash_boards.py          # same as: ... all
```

It runs: detect, choose ports, build both, ATmega safety checklist and flash, prompt to unplug/close the
Arduino USB, flash ESP32, wait up to 15 s for the port to come back, verify, and print next steps.

Global flag: `--no-color` (colors are also off when stdout is not a TTY). Exit code 0 = success,
1 = failure, 2 = usage/port selection problem, 130 = aborted.

## 4. Subcommands

| Command | What it does |
|:---|:---|
| `detect` | Lists ports with VID:PID and description; classifies as ESP32-S3 (303A:1001), CH340/FTDI Arduino, or unknown. |
| `build` | `pio run` for both projects, PASS/FAIL each. |
| `flash-atmega [--port P]` | Safety checklist, build, `pio run -e arduino_mini -t upload` in `ArduinoCompanion/`. |
| `flash-esp32 [--port P] [--erase]` | Build and upload `esp32-s3-devkitc-1`. `--erase` runs `pio run -t erase` first (asks to confirm). |
| `verify [--port P]` | Opens the ESP32 port at 115200, waits for a `<...>` status, runs `$I`, `$S`, `$RB`, `$$`; prints a PASS/FAIL checklist with fixes and saves a settings backup. |
| `backup [--port P]` | Saves `$$` to `tools/backups/settings-YYYYmmdd-HHMMSS.json`. |
| `restore FILE [--port P]` | Sends every `$N=v` from the backup and reports any non-`ok` reply. |
| `all` | The full wizard (default). Extra flags: `--atmega-port`, `--esp-port`, `--erase`. |
| `selftest` | Runs the response-parser self-test with canned firmware replies (no hardware). |

`verify` checks: status report, `[VER:1.1h.WaveMaster:]`, `[OPT:V,<queue>,1024]`, `[MSG:atmega link=1 ...]`
(armed/ready/power/vdet/err), galvo stats (underruns), `$RB` DAC readback, and the boot self-test line if
one arrives (only visible if the board resets while connected).

## 5. Troubleshooting

| Symptom | Cause / fix |
|:---|:---|
| `Bad CPU type in executable` / `Unknown system error -86` (Apple Silicon) | Intel-only PlatformIO tools (avr-gcc, ninja) need Rosetta 2: `softwareupdate --install-rosetta --agree-to-license`, then rerun. `setup_mac.sh` now checks and offers this. |
| Port not found | Try another cable (charge-only cables have no data), replug, run `detect`. ESP32: `/dev/cu.usbmodem*`; Arduino: `/dev/cu.usbserial-*` or `/dev/cu.wchusbserial*`. |
| CH340 not showing up | macOS 10.15+/Big Sur and later include a CH34x driver. If missing, install WCH's driver, allow the extension in System Settings > Privacy & Security, replug. |
| `avrdude: stk500_getsync(): not in sync` | Wrong port or board, or bootloader speed. New-bootloader Nano clones need 115200: in `ArduinoCompanion/platformio.ini` set `upload_speed = 115200` (default 57600). Also check the monitor is closed and the cable carries data. |
| ESP32 will not enter boot mode / port vanishes | Hold BOOT, tap RESET, release BOOT, rerun. The port name may change; run `detect`. |
| `Resource busy` | Another program holds the port: Rayforge, LightBurn, `screen`, Arduino IDE, `pio device monitor`. Close it. |
| `verify`: `link=0` | ATmega not flashed (run `flash-atmega`); level shifter unpowered; TX/RX swapped (ESP32 GPIO44 TX to ATmega RX, GPIO43 RX from ATmega TX); Arduino USB serial monitor open (close it or unplug the USB); heartbeat not up yet (wait, rerun). |
| `verify`: `RB unavailable` | DAC init failed or SPI error/busy. Check AD3552R SPI wiring, supplies and reference, GPIO7 (OP1) mode strap per `docs/WIRING.md`; reset and retry. |
| `verify`: no status report | Wrong port, or firmware crashed. Reflash with BOOT/RESET; close other apps. |
| `--erase` consequences | Wipes NVS: **all `$` settings** (scale `$100/$101`, offsets `$110/$111`, delays `$130-$134`, ...) return to defaults. Run `backup` first, then `restore FILE` afterwards. |

## 6. Next steps

1. Verify wiring against `docs/WIRING.md` with the laser still OFF.
2. Follow the test plans in `tools/rftest/`.
3. Calibrate (`$100/$101`, `$110/$111`, `$130-$134`), then run `backup`.
