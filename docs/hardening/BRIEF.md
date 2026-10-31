# Hardening brief (read first)

Project: WaveMaster, an ESP32-S3 (IDF 5.5.3, PlatformIO) controller for a galvo laser. It speaks a
GRBL 1.1 subset to Rayforge (host app, repo at /Volumes/Projects/rayforge, read-only for you) over USB
serial and WiFi (telnet port 23). Hardware: AD3552R dual DAC for the galvo (SPI), an ATmega for laser
arm/power/guide (atmega_link.c), and a wireless Z axis "Z-Bro" (ESP32-C3, zbro_link.c, TCP :7700).

Branch: `hardening/audit-and-features` (from main + the paced-drop fix in dac_task.c).
Build (works, ~30 s incremental), from the repo root:
`PLATFORMIO_CORE_DIR=/Volumes/Projects/TBBuild/.platformio /Volumes/Projects/pio311-env/bin/pio run -e esp32-s3-devkitc-1-debug`
(use `-e esp32-s3-devkitc-1` for the release env). Do not use the plain `pio`/`platformio` on PATH: it
runs on Python 3.14 and fails, and without the real core dir path SCons reports duplicate targets.
Output: `.pio/build/<env>/firmware.bin`. Run builds one at a time; they share `.pio`.

## Hard rules
- NO hardware access: do not flash, open the serial port, ping or connect to 192.168.3.x devices, or
  run anything that can emit laser light. Build and host-side checks only.
- Do not touch /Volumes/Projects/rayforge except to read. Do not push, merge or force anything.
- Do not run `tell-owner`; the coordinator does that. Ask the coordinator with the `ask` command from
  your Orca preamble.
- Keep code style of the surrounding files (4-space C, comments only for the why). Do not remove logging.
- Every behaviour change needs the smallest possible diff and a reason tied to a finding.

## What is known (evidence from the S3 hub logs and core dumps, 2026-10-31)
1. Crash A, interrupt watchdog on core 0: dump shows IDLE0 asleep in esp_cpu_wait_for_intr, IDLE1 idle,
   all tasks blocked normally, no lock held. The core 0 tick (systimer, level 1) stopped arriving, so the
   300 ms INT_WDT fired. Seen as reset reasons `int_wdt` and `task_wdt`/`unknown`. Cause NOT found.
   dac_task runs on core 0; its gptimer ISR uses intr_priority 3 (above the level-1 tick).
2. Crash B, task watchdog on core 1 (IDLE1 starved by `motion`): when the DAC stream cannot open
   (`dac_task stream open failed: -5`, DAC registers read 0xff), try_start() dropped chunks instantly,
   so motion_task never blocked. Mitigated by drop_paced() in dac_task.c (this branch).
3. The DAC lost its configuration at runtime several times (register 0x19 0x44 -> 0x00, then all 0xff),
   only around runs where the laser fired, and an `unknown` reset followed within ~40 s. A zero-power
   (S0) serial job of 606 lines produced none of it. Suspected supply/EMI; unproven.
4. Z-Bro: the C3 closes the TCP link ~10 s into a homing that has not finished, and logs
   `C3 not answering (waiting for reply type 2/3: 5 s)` every 1-4 min under use.
5. Auto guide ($231, default 1): grbl.c turns the guide laser on for any G1/G2/G3 while `M3` with S=0,
   including inside real jobs that use constant power (M3) with zero-power segments.
6. `dac_task stream underrun with the laser on (host too slow?)` is frequent during Rayforge jobs.
7. The S3 reports crash backtraces, resets and health to the ESPFleet hub; the debug build adds
   freeze_watch/debug_tools. The watchdog `last crash` line is re-reported on every boot until
   overwritten, so repeated lines are not repeated crashes.
8. Opening the USB serial port can reset the chip; esptool always does.

## Goal
Harden the firmware (crashes, races, watchdog safety, error handling, laser-safety invariants) and add
the controller features Rayforge can use. Audit the whole firmware; refactor where it pays.
