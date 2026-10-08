# WaveMaster — Agent & Contributor Guidelines

This document provides direct, focused context for AI agents and developers working on the WaveMaster firmware. Keep changes precise, verify hardware assumptions, and avoid architectural regressions.

---

## 1. System Model & Boundaries

WaveMaster coordinates an ESP32-S3, an AD3552R dual DAC and an ATmega328P companion. The data path is one-way: G-code becomes ticks, ticks become DAC codes and gate edges, and those are clocked out by hardware.

1. **ESP32-S3 (Primary Controller — `src/`)**

   * **Core 0 — `dac_task` (priority 10)**: the **only** owner of the SPI bus and the AD3552R. It clocks chunks out as one hardware-paced quad-SPI stream (SPI clock = point rate, CS held low across chunks). It also drives the laser gate from the SPI callbacks and a GPTimer alarm, services barriers (power and PRR changes), and does `dac_task_readback()` (only while the stream is closed).
   * **Core 1 — host RX (priority 10), `grbl` (9), `motion` (8), ATmega link `atmega_link` (5)**:
     * `host_serial.c`: USB-Serial-JTAG RX, realtime bytes, TX mutex.
     * `grbl.c`: G-code, `$` commands, status, reset handling.
     * `motion_task.c`: owns the trajectory generator (`motion.c`) and calls `galvo_out_tick()`.
     * `atmega_link.c`: UART0 protocol v2 client, heartbeat.
   * **Hand-off between tasks**: `motion_submit()` queue (512 segments) from `grbl` to `motion`. `galvo_out` chunk pool (8 chunks, `galvo_chunk.h`) from `motion` to `dac_task`. Never perform SPI from any task other than `dac_task`, and never block `dac_task` on a core-1 task.

2. **Analog Devices AD3552R (Dual 16-bit DAC)**

   * Quad SPI (single-lane for bring-up). Hardware mode select is `BOARD_PIN_SPI_OP1` (GPIO 7), which drives the chip's QSPI pin. It must be low before any single-lane SPI activity.
   * Single writes go through `ad3552r_board_write_volts()` to `AD3552R_REG_ADDR_CH_DAC_24B(ch)`. Streaming goes through `ad3552r_board_stream_open()` / `_close()`.
   * CH0 drives X, CH1 drives Y. The output is differential to the galvo drivers, ±10 V.

3. **ATmega328P (`ArduinoCompanion/`)**

   * Owns the slow DB25 lines: 8-bit power word (D0–D7, DB25 pins 1–8) with a LATCH strobe (pin 9), arm / MO / emission enable (pin 18), guide laser (pin 22), AUX OFF held high (pin 23), status inputs (pins 11, 12, 16) and the 5 V detect (pin 17).
   * UART0 at 250 000 baud 8N1 through a level shifter (TX = GPIO 44, RX = GPIO 43, swapped from the IOMUX defaults). Protocol v2 is framed and CRC-8 protected.
   * The ATmega disarms and zeroes power if it hears nothing for 1 s (heartbeat). The ESP32 polls every 200 ms.
   * Commands are fire-and-forget from `grbl`'s point of view. `atmega_link` does the round trips.

4. **Fast lines are owned by the ESP32, not the ATmega**: EMISSION MODULATION (the gate, DB25 pin 19) is GPIO 4 and SYNC / PRR (DB25 pin 20) is GPIO 3 (LEDC). `laser_io.c` owns them.

---

## 2. Hard Real-Time Constraints (Do Not Break)

* **Streaming path (`dac_task`, gate ISRs)**:
  * Never call `vTaskDelay()`, sleep or busy-wait inside the stream loop or any ISR. A 1-tick delay in the stream causes visible flat spots and laser stutter.
  * `dac_task` sleeps on SPI completion. It does not spin.
  * `CONFIG_FREERTOS_HZ` is **1000** (1 ms tick), set in `sdkconfig.defaults`. Timeouts in core-1 tasks are in ms.
* **Interrupt context**:
  * SPI pre/post callbacks, the GPTimer alarm, and `laser_io_gate()` must be IRAM-safe: `IRAM_ATTR`, and only IRAM-safe calls.
  * Those options are enabled in `sdkconfig.defaults`: `CONFIG_SPI_MASTER_ISR_IN_IRAM`, `CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM`, `CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM`, `CONFIG_GPTIMER_ISR_CACHE_SAFE`. Keep them. The last one keeps gate edges from being delayed by NVS flash writes.
  * No logging, no mutexes and no allocation from an ISR.
* **The laser must never be left on**: every abort, underrun, close and reset path must gate the laser off. Those are `galvo_out_abort()`, `laser_io_gate_kill()`, the `motion_gen_finish()` underrun path, and the soft reset (Ctrl-X). The wiring guide calls for a 10 kOhm pull-down on DB25 pin 19 (see `docs/WIRING.md`). Do not invert `$230` without checking it.
* **Never block in the streaming path**. Blocking is only allowed where it is intended backpressure: `motion_submit()` from `grbl` and `galvo_out_tick()` waiting on the chunk pool.
* **USB-Serial-JTAG vs UART0**:
  * The host (Rayforge) uses the native **USB-Serial-JTAG** port.
  * UART0 is dedicated to the ATmega link and must not carry anything else.
* **Logs**: all firmware logs go out as `[MSG:...]` lines through `host_serial` (esp_log is redirected there). Do not `printf` to stdout. Use `host_puts()` / `host_printf()`, which take a mutex, and not from ISRs.
* **Heap**:
  * Internal RAM is about 300 KB. Do not add large buffers.
  * The chunk pool is fixed: 8 chunks of 2048 ticks × 6 bytes = 12 KB DMA each, allocated once.
  * Never allocate multi-megabyte coordinate buffers. Motion is generated in chunks.

---

## 3. Workflow & Verification Strategy

When modifying firmware code:

1. **Static build check**:
   ```bash
   pio run
   ```
   Compilation must pass with zero warnings in `src/`. The firmware uses `espressif32@6.13.0` (ESP-IDF 5.5).

2. **ATmega companion**: if you change the companion firmware or the protocol:
   ```bash
   cd ArduinoCompanion && pio run
   ```
   The protocol is in `ArduinoCompanion/README.md`. Change the ESP32 side (`src/atmega_link.c`) and the companion together.

3. **Verify against register readback**:
   * Do not assume SPI success equals correct voltage. Use the built-in `$RB` command, which reads the two DAC output registers. It only works while the stream is closed.
   * `ad3552r_board_volts_to_code()` is the same conversion used for writes. Compare it with what `$RB` reads back.

4. **Preserve calibration semantics**:
   * Settings live in one table. `src/calib.h` has the enum and the authoritative comment table, and `src/calib.c` has `s_defs`: number, alias, NVS key, default, min, max.
   * `$$` is generated from the table, so there is no separate dump code to update.
   * To add a setting: add an enum entry in `calib.h` and a row in `s_defs` in `calib.c`, then document it in the comment table in `calib.h`. The NVS key must be 15 characters or fewer.
   * Current anchors: scale `$100` / `$101` (V/mm, default 0.1), offsets `$140` / `$141` (V), delays `$210`–`$216` (us; `$213` is us per mm, `$216` is degrees), power mode `$223`, gate polarity `$230`.
   * Do not reuse an existing `$` number. Keep the GRBL-compatible numbers that Rayforge reads: `$30`, `$110`, `$120`, `$130`, `$131`, `$12`, `$13`.

5. **Host unit tests**: run these after changing `src/motion.c`, `src/motion.h` or `src/gcode_parse.c`:
   ```bash
   make -C test/host              # trajectory generator: 58808 checks, expect 0 failures
   make -C test/host/gcode        # G-code parser: expect "gcode tests passed"
   ```
   The tests build with `-std=c99 -Wall -Wextra -Werror`. `motion.c` has no ESP-IDF dependencies, so keep it that way.

6. **Hardware changes**: hardware bring-up is in `STATUS.md`. Do not claim a hardware behaviour is verified until it has been checked on the scope or the laser.
