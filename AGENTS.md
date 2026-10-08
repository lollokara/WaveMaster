# WaveMaster — Agent & Contributor Guidelines

This document provides direct, focused context for AI agents and developers working on the WaveMaster firmware. Keep changes precise, verify hardware assumptions, and avoid architectural regressions.

---

## 1. System Model & Boundaries

WaveMaster coordinates two distinct microcontrollers and an analog front-end:

1. **ESP32-S3 (Primary Controller — `src/`)**:
   - **Core 0 (`dac_task`)**: SPI bus master, DAC hardware writes, hardware-paced stream loop, laser firing trigger (`Engrave GPIO 4`), PRR PWM (`GPIO 3`), settle delays.
   - **Core 1 (`grbl_task` + `atmega_task`)**: G-code parser, kinematics (look-ahead planner, junction deviation), NVS calibration, non-blocking serial communication with the ATmega.
   - **Communication between cores**: FreeRTOS queues (`s_cmd_queue` in `laser_ctrl.c`). Never perform direct cross-core SPI transactions or block Core 0.

2. **Analog Devices AD3552R (Dual 16-bit DAC)**:
   - Driven via single-lane or quad SPI (hardware mode selected by `BOARD_PIN_SPI_OP1` / GPIO 7).
   - Writes register `AD3552R_REG_ADDR_CH_DAC_24B(ch)` directly.
   - Differential analog voltages control X and Y galvo drivers.

3. **ATmega328P (`ArduinoCompanion/`)**:
   - Manages laser power level (8-bit parallel digital word on `D0-D7`), emission enable relay/arm sequence, and red guide laser (`M62`/`M63`).
   - Connected via UART0 (`GPIO 44 TX` / `GPIO 43 RX`) at 250,000 baud 8N1 across a level shifter.
   - Commands are asynchronous fire-and-forget from `grbl_task`'s perspective (`atmega_task` handles UART round-trips).

---

## 2. Hard Real-Time Constraints (Do Not Break)

* **Watchdog Feeding (`dac_task`)**:
  - `CONFIG_FREERTOS_HZ` is 100 (1 tick = 10ms). **Never insert `vTaskDelay(...)` inside the fast pacing or preview loop** — even a 1-tick delay creates visible 10ms flat spots in galvo position and causes laser stutter.
  - Watchdog handling during continuous bursts is done via `paced_wdt_begin()` and `esp_task_wdt_reset()`.
* **USB-Serial-JTAG vs UART0**:
  - Host G-code commands and logs use the native **USB-Serial-JTAG** port (`stdio`).
  - All firmware log lines are wrapped with `; ` (`grbl_comment_vprintf`) so host G-code parsers (LightBurn, Rayforge) ignore them. Never output unformatted debug prints to `stdout` without checking this mechanism.
  - Physical UART0 pins are dedicated exclusively to the ATmega328P link.
* **Heap Budget**:
  - Available internal RAM is ~300KB.
  - Large streaming bursts must be batched (`PACED_BATCH_MAX_BYTES` is 16KB). Never allocate massive multi-megabyte coordinate buffers in RAM.

---

## 3. Workflow & Verification Strategy

When modifying firmware code:

1. **Static Build Check**:
   ```bash
   pio run
   ```
   Ensure compilation passes cleanly with zero warnings in `src/`.

2. **ATmega Companion**:
   If changing the companion firmware or protocol:
   ```bash
   cd ArduinoCompanion && pio run
   ```

3. **Verify Against Register Readback**:
   - When altering voltage conversions or DAC code paths, do not assume SPI success equals correct voltage.
   - Use the built-in `$RB` command to verify that `ad3552r_write_reg` actually sets the expected DAC register code.

4. **Preserve Calibration Semantics**:
   - Optical scale `$100`/`$101` (V/mm), offset `$110`/`$111` (V), and timing delays `$130`-`$134` persist in NVS via `calib.c`.
   - Any new tuning parameter should follow this pattern and update `calib.c` / `calib_dump()`.

---

## 4. Git & Push Discipline

* The repo uses dual-push configuration for `origin` (`git@192.168.3.87:...` and `git@github.com:...`).
* On macOS / exFAT volumes, ensure `COPYFILE_DISABLE=1` is set and strip any `._*` AppleDouble sidecars before staging commits:
  ```bash
  export COPYFILE_DISABLE=1
  find . -name "._*" -delete
  git add <files>
  git commit -m "..."
  git push origin main
  ```
