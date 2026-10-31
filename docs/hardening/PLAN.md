# WaveMaster Firmware Hardening & Rayforge Integration Plan

## Executive Summary

This plan covers the whole-firmware audit of the WaveMaster ESP32-S3 galvo controller (`src/`, `components/`, `ArduinoCompanion/`, `CompanionZ/`, `platformio.ini`, `partitions.csv`, `sdkconfig*`) and a capability/protocol gap analysis against the Rayforge host application (`/Volumes/Projects/rayforge`).

### Root-Cause Summary for the Open Field Issues (`docs/hardening/BRIEF.md`)

1. **Crash A — Silent reboot during jobs (`int_wdt` / `unknown`, no coredump):**
   - **Primary root cause (C2):** On Core 0, the GPTimer gate ISR (`on_alarm`, `src/dac_task.c:121`) is registered at **interrupt priority level 3** (`src/dac_task.c:554`), whereas the SPI2 DMA ISR (`stream_pre` / `stream_post`, `src/dac_task.c:139,155`) is registered at default **level 1** (`components/ad3552r/no_os_spi_esp32.c:94`). `on_alarm()` preempts `stream_pre()` and `stream_post()` mid-execution while mutating shared state (`s_cur`, `s_edge_idx`, `s_gate_state`, `s_alarm_armed`, `s_next_edge_time`) with zero critical-section protection. In `isr_arm_next()` (`src/dac_task.c:114-115`), `gptimer_set_alarm_action()` is called *before* `s_alarm_armed = true`; when `t` is clamped to `now + 2 us` (`src/dac_task.c:108`), the level-3 timer ISR fires immediately inside `gptimer_set_alarm_action()`, sees `s_alarm_armed == false`, and returns without advancing `s_edge_idx`, or loops inside `on_alarm()` (`src/dac_task.c:126-129`). Under Pulse Density mode (`$223=1`, up to 256 gate edges per 20.48 ms chunk), level-3 ISR storms + `sample_cb` (`src/debug_tools.c:99-127`, running every 100 ms on Core 0's `esp_timer` task walking 9 task stacks) + unpinned WiFi ISRs on Core 0 trip the Interrupt Watchdog (`int_wdt`).
   - **Why `task_wdt` escalates into `int_wdt` / `unknown` with no coredump (C3):** `esp_task_wdt_isr_user_handler()` (`src/debug_tools.c:38-63`) runs in ISR context (`IRAM_ATTR`) and calls `snprintf()`, `strlcat()`, and `esp_task_wdt_print_triggered_tasks()` (`src/debug_tools.c:61`). `esp_task_wdt_print_triggered_tasks()` logs via `esp_log_writev` -> `host_log_vprintf` (`src/host_serial.c:171`), which calls `xSemaphoreTake(s_tx_mutex, portMAX_DELAY)` (`src/host_serial.c:186`) and `espfleet_log_push()` (`components/espfleet/src/EspFleet.cpp:881`) **inside an ISR**, triggering a double-fault / `int_wdt` before the panic handler can write a flash coredump. In addition, `sdkconfig.debug:20-22` leaves `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y` active instead of `CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT=y`, and `sdkconfig.defaults` omits `CONFIG_GPIO_CTRL_FUNC_IN_IRAM=y`.
   - **Memory corruption contributor (C1):** `ad3552r_write_reg()` (`components/ad3552r/ad3552r.c:578`) and `ad3552r_read_reg()` (`components/ad3552r/ad3552r.c:618`) allocate `uint8_t buf[3]` (`AD3552R_MAX_REG_SIZE = 3`) on the stack, but with `param.single_transfer = 1` (`src/ad3552r_board.c:53`), `ad3552r_single_transfer()` (`components/ad3552r/ad3552r.c:528-535`) writes `data[3] = data[2]` and performs a 4-byte SPI transfer on `buf[0..3]` whenever a 24-bit register (`AD3552R_REG_ADDR_CH_DAC_24B`, length 3) is written or read—corrupting 1 byte past `buf` on `dac_task`'s stack on every single-sample write and `$RB` read.

2. **Crash B & Brief Item 3 — `task_wdt` after `ad3552r_board_stream_open: failed (-5)` and DAC register loss (`0x19: 0x44 -> 0x00`, then all `0xff`):**
   - **Why the DAC loses config and wedges into `0xff` (C4):** While streaming (`s_session_open == true`), `CS` is held low continuously and `QSPI` (`BOARD_PIN_SPI_OP1`, GPIO 7) is held high (`src/ad3552r_board.c:362-367`). When a power or PRR barrier arrives (`apply_barrier()` at `src/dac_task.c:332-341` or `apply_power()` at `src/dac_task.c:322-330`), `dac_task` leaves the quad-SPI stream open (`CS` low) while blocking up to 50 ms on the ATmega UART round-trip. Any clock/CS glitch during that window desynchronizes the AD3552R 4-byte stream counter or wedges the chip in Quad-SPI / DDR mode. When `guard_check_and_repair()` (`src/ad3552r_board.c:260-281`) runs, it only attempts single-lane register writes (`ad3552r_write_reg`) and never checks `TRANSFER_REGISTER` (`0x0F`, which controls `MULTI_IO_MODE`) and **never pulses hardware `RESET` (`BOARD_PIN_RST`, GPIO 9)**. Once the AD3552R is wedged in QSPI mode or latched up, single-lane reads return `0xff` and single-lane writes are ignored until power-cycle.
   - **Laser safety violation & log storm (C4):** In `try_start()` (`src/dac_task.c:396-410`), `do_arm(true)` (`src/dac_task.c:400`) and `apply_power()` (`src/dac_task.c:403`) execute **before** `session_open(s_prr_hz)` (`src/dac_task.c:407`). When `session_open()` fails (`-5`), `drop_paced(c)` (`src/dac_task.c:409`) paces chunk disposal, **but leaves the laser armed (`s_armed = true`) with marking power latched on the ATmega**, logs two `ESP_LOGE` lines per chunk (100 lines/s taking `s_tx_mutex` from Core 0), and never aborts or faults the job.

3. **Brief Item 4 — Z-Bro TCP link drops ~10 s into homing and logs `C3 not answering (waiting for reply type 2/3: 5 s)` every 1–4 min (C5):**
   - Neither the S3 (`src/net_wifi.c`) nor the C3 (`CompanionZ/main/zb_net.c`) disables WiFi modem power save (`esp_wifi_set_ps(WIFI_PS_NONE)`), leaving `WIFI_PS_MIN_MODEM` active on both endpoints.
   - During homing, the C3 sends unsolicited `POS` lines at 10 Hz (`CompanionZ/main/zb_net.c:520-528`). Whenever WiFi power-save or `espfleet` background HTTP log-shipping (`components/espfleet/src/EspFleet.cpp:478,834,976`, every 5 s) stalls TCP ACKs for >500 ms (`SEND_WAIT_MS`), `send_line()` on the C3 (`CompanionZ/main/zb_net.c:269-280`) calls `shutdown(s_cfd, SHUT_RDWR)` and drops the client, which immediately triggers `stop_orphan_op("client disconnected")` (`CompanionZ/main/zb_net.c:535` -> `zb_motion_stop()`) and **aborts the homing cycle mid-move**.
   - On the S3 side, when `session()` (`src/zbro_link.c:315,334-345`) reconnects 1 s later, `autohome_tried` is a local variable in `session()`, so if boot auto-home (`s_auto == ZBRO_AUTO_HOMING`) was interrupted before `EVT HOMED`, it re-issues `HOME` on every reconnect in an infinite loop. Furthermore, while `exp != EXP_NONE` (`src/zbro_link.c:347`), the S3 stops sending `PING`, which can trip the C3's 3-second `PING_TIMEOUT_MS` (`CompanionZ/main/zb_net.c:516`).

4. **Brief Item 5 — `$231` Auto Guide (`CAL_AUTO_GUIDE`) turns on mid-job under `M3` and stalls the motion pipeline (H1):**
   - In `emit_linear()` (`src/grbl.c:541-545`), whenever `s_m.laser == 3 && !(s_m.s > 0.0f) && !s_m.preview && calib_get(CAL_AUTO_GUIDE) > 0.5f`, `grbl` immediately calls `atmega_link_set_guide(true)` (without `wait_idle()`, turning the guide laser on while up to 512 `S>0` cutting segments are still queued in `motion_task`!) and sets `s_auto_guide = true`. Then, on the next `S > 0` move, `sync_laser_on()` (`src/grbl.c:449-454`) sees `s_auto_guide` and calls `wait_idle()`, draining the entire motion pipeline to zero before every cut move in an `M3` job.

5. **Brief Item 6 — `dac_task: stream underrun with the laser on (host too slow?)` (H2 & H4):**
   - Three code defects trigger this warning even when the host is keeping up:
     1. In `motion_task_loop()` (`src/motion_task.c:208-218`), when `s_queue` is momentarily empty while `motion_gen_has_pending(&s_gen)` is true (`in_polyline == true`), line 209 calls `galvo_out_flush()` *before* checking `allowed_us` (line 212). That flushes a partial chunk with `c->gate_end = true` to `dac_task`. When `allowed_us` is small, `dac_task` finishes that partial chunk before `motion_gen_finish(&s_gen)` is flushed, so `stream_post()` (`src/dac_task.c:165`) sees `left <= 0` with `c->gate_end == true` (`s_isr_drain_lit = true`).
     2. In `service()` (`src/dac_task.c:477-487`), when `s_isr_drained` is set, `reap()` is called at line 477 *before* `xQueueReceive(g_galvo.ready_q, &s_next, 0)` at line 487. Inside `reap()` (`src/dac_task.c:240`), `s_next` is `NULL` even when `ready_q` already has chunks waiting, firing a false warning.
     3. On Core 1, `motion` runs at priority **8** (`src/motion_task.c:339`), **below** `grbl` (9, `src/grbl.c:1353`), `host_rx` (10, `src/host_serial.c:428`), `net_srv` (10, `src/net_server.c:292`), and `net_tx` (10, `src/net_server.c:293`), allowing G-code parsing and log flushing to preempt real-time chunk generation.

---

## 1. Whole-Firmware Audit

Every finding below cites verified `file:line` locations in this repository, states what is wrong, explains the failure mode, and specifies the smallest safe fix.

### 1.1 Architecture & RTOS Overview (Tasks, Priorities, Stacks, Cores, Spinlocks, Mutexes, Queues)

| Task Name | File:Line | Core | Current Prio | Proposed Prio | Stack (Bytes) | TWDT Watched? | Role & Blocking Profile |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| `dac` | `src/dac_task.c:605` | 0 | 10 | 10 | 4096 | **No** -> **Yes** | Sole SPI2 & AD3552R owner; sleeps on `s_done_sem` or `ready_q` (20 ms). Currently blocks on `atmega_link` (up to 50 ms) and `s_tx_mutex` (via `ESP_LOG`). |
| `esp_timer` | ESP-IDF internal | 0 | 22 | 22 | 3584 | No | Runs `sample_cb` (`src/debug_tools.c:99`) every 100 ms on Core 0. |
| `wifi` / `tiT` | ESP-IDF internal | **0** | 23 / 18 | Pin to **1** | IDF default | No | WiFi driver & lwIP TCP/IP tasks default to Core 0 unless pinned in `sdkconfig.defaults`. |
| `host_rx` | `src/host_serial.c:428` | 1 | 10 | **8** | 4096 | No | Polls USB-Serial-JTAG RX (20 ms timeout), intercepts `Ctrl-X`, pushes to `s_rx_ring`. |
| `net_srv` | `src/net_server.c:292` | 1 | 10 | **8** | 4096 | No | `select()` on TCP port 23 (50 ms timeout), intercepts `Ctrl-X`, pushes to `s_rx_ring`. |
| `net_tx` | `src/net_server.c:293` | 1 | 10 | **5** | 3072 | No | Drains `s_tx_ring` (8 KB) to TCP client socket with 20 ms backpressure sleep. |
| `grbl` | `src/grbl.c:1353` | 1 | 9 | **7** | 6144 | **No** -> **Progress** | Parses G-code / `$` commands, blocks on `motion_submit()`, `wait_idle()`, `zbro_link`, NVS. |
| `motion` | `src/motion_task.c:339` | 1 | 8 | **9** | 4096 | **No** -> **Progress** | Runs trajectory generator (`motion.c`), pushes ticks to `galvo_out`, blocks on `free_q`. |
| `atmega` | `src/atmega_link.c:465` | 1 | 5 | **10** | 4096 | **No** -> **Yes** | UART0 v2 link + 200 ms safety heartbeat; unblocks `dac_task` power/arm barriers. |
| `zbro` | `src/zbro_link.c:646` | 1 | 5 | **5** | 4096 | No | TCP client to Z-Bro C3 (`z-bro.local:23`), 1 s ping, status cache. |
| `espfleet` | `components/espfleet/src/EspFleet.cpp:277` | Any (`tskNO_AFFINITY`) | 3 | Pin to **1**, prio **3** | 6144 | No | Unpinned (`xTaskCreate`); can migrate to Core 0 during HTTP/mDNS/OTA operations. |

---

### 1.2 Critical Findings

#### [CRITICAL-1] Stack Buffer Overflow and Corrupted Readback in `ad3552r` 24-Bit Single-Transfer Register Access
- **Location:** `components/ad3552r/ad3552r.c:78`, `components/ad3552r/ad3552r.c:232-233`, `components/ad3552r/ad3552r.c:498-544`, `components/ad3552r/ad3552r.c:578`, `components/ad3552r/ad3552r.c:618`; called from `src/ad3552r_board.c:53`, `src/ad3552r_board.c:186`, `src/ad3552r_board.c:195`.
- **What is wrong:**
  1. `ad3552r_board_init()` sets `param.single_transfer = 1` (`src/ad3552r_board.c:53`).
  2. Both `ad3552r_write_reg()` (`components/ad3552r/ad3552r.c:578`) and `ad3552r_read_reg()` (`components/ad3552r/ad3552r.c:618`) declare `uint8_t buf[AD3552R_MAX_REG_SIZE] = { 0 };` on the stack, where `AD3552R_MAX_REG_SIZE` is `3` (`components/ad3552r/ad3552r.c:78`).
  3. For 24-bit registers (`AD3552R_REG_ADDR_CH_DAC_24B(ch)` = `0x4B` / `0x4E`), `ad3552r_reg_len(addr)` (`components/ad3552r/ad3552r.c:232-233`) returns `3` (`msg.len = 3`).
  4. `ad3552r_single_transfer()` (`components/ad3552r/ad3552r.c:528-535`) calls `ad3552r_convert_to_single_msg(data->data, 3)` (`components/ad3552r/ad3552r.c:503`), which shifts `data[0..2]` into `data[1..3]`—writing 1 byte **past the end of `buf[3]`** on `dac_task`'s stack—and then executes a 4-byte SPI transfer (`bytes_number = data->len + 1 = 4`) into `buf[0..3]`, overwriting the same out-of-bounds stack byte with the last MISO byte.
  5. On reads (`components/ad3552r/ad3552r.c:537-538`), `ad3552r_single_transfer()` only copies `data->data[0] = data->data[1];` and leaves `data->data[1]` unchanged (still holding MSB `data[1]`), so `ad3552r_read_reg()` (`components/ad3552r/ad3552r.c:661`) reconstructs `(MSB << 8) | MSB` instead of `(MSB << 8) | LSB` (directly explaining why `$RB` read back `0x8080` for `0x8000` in `STATUS.md:45`).
- **How it fails:** Every call to `ad3552r_board_write_code()` (`src/ad3552r_board.c:186`, used at boot, after every stream close at `src/ad3552r_board.c:423`, and by `$DV`) and `ad3552r_board_read_dac_code()` (`src/ad3552r_board.c:195`, used by `$RB`) corrupts 1 byte of caller stack memory in `dac_task` and returns a corrupted LSB on `$RB`.
- **Smallest fix:**
  1. In `components/ad3552r/ad3552r.c:578` and `components/ad3552r/ad3552r.c:618`, change `uint8_t buf[AD3552R_MAX_REG_SIZE] = { 0 };` to `uint8_t buf[AD3552R_MAX_REG_SIZE + 1] = { 0 };`.
  2. In `components/ad3552r/ad3552r.c:537-538`, replace `if (data->is_read) data->data[0] = data->data[1];` with:
     ```c
     if (data->is_read)
         memmove(data->data, &data->data[1], data->len);
     ```

#### [CRITICAL-2] Level-3 GPTimer ISR Preempts Level-1 SPI DMA ISR on Core 0 Without Synchronization (Root Cause of Crash A `int_wdt` & Gate Glitches)
- **Location:** `src/dac_task.c:79-181`, `src/dac_task.c:554`, `components/ad3552r/no_os_spi_esp32.c:94`, `sdkconfig.defaults:22-35`.
- **What is wrong:**
  1. `s_timer` is allocated with `.intr_priority = 3` (`src/dac_task.c:554`), whereas `SPI2_HOST` is allocated with `ESP_INTR_FLAG_IRAM` (priority 1, `components/ad3552r/no_os_spi_esp32.c:94`). Both interrupts are registered on Core 0 (`src/dac_task.c:605`).
  2. Because level 3 preempts level 1 on Xtensa LX7, `on_alarm()` (`src/dac_task.c:121`) preempts `stream_pre()` (`src/dac_task.c:139`) and `stream_post()` (`src/dac_task.c:155`) mid-execution while both read and write `s_cur`, `s_edge_idx`, `s_gate_state`, `s_alarm_armed`, and `s_next_edge_time` without a spinlock or interrupt mask.
  3. In `isr_arm_next()` (`src/dac_task.c:114-115`), `gptimer_set_alarm_action(s_timer, &s_alarm_cfg)` is called **before** `s_alarm_armed = true`. When `t` is clamped to `now + ARM_MARGIN_COUNTS` (2 us, `src/dac_task.c:36,108`), the alarm can fire before `gptimer_set_alarm_action()` returns, preempting `stream_pre()` while `s_alarm_armed` is still `false`. `on_alarm()` (`src/dac_task.c:124`) sees `!s_alarm_armed || !s_cur` and returns `false` without driving the gate edge or advancing `s_edge_idx`; `isr_arm_next()` then sets `s_alarm_armed = true` for an alarm that already fired, dropping all remaining gate edges in the chunk.
  4. In `on_alarm()` (`src/dac_task.c:126-129`), `if (timer_now() < s_next_edge_time) { isr_arm_next(); return false; }` re-arms the timer if `timer_now()` reads slightly behind `s_next_edge_time`, causing back-to-back level-3 interrupts.
  5. `sdkconfig.defaults` enables `CONFIG_GPTIMER_ISR_CACHE_SAFE=y` (`sdkconfig.defaults:25`), which places `on_alarm` in `ESP_INTR_FLAG_IRAM`, so `on_alarm` runs even while flash cache is disabled (e.g. NVS write or flash operations). However, `on_alarm` calls `laser_io_gate()` -> `gpio_set_level()` (`src/laser_io.c:116`), and `CONFIG_GPIO_CTRL_FUNC_IN_IRAM=y` is **missing** from `sdkconfig.defaults` (it was added to `CompanionZ/sdkconfig.defaults:24` but omitted from WaveMaster's `sdkconfig.defaults`). Calling a flash-resident `gpio_set_level()` from an IRAM ISR while flash cache is disabled causes an immediate cache-error panic / `int_wdt`.
- **How it fails:** Under heavy gate edge activity (especially Pulse Density mode `$223=1`, where chunks carry up to 256 edges per 20.48 ms) or concurrent flash/WiFi activity, Core 0 either faults on non-IRAM `gpio_set_level`, drops/inverts gate edges when `on_alarm` preempts `stream_pre`/`stream_post`, or starves level-1 interrupts on Core 0 until `int_wdt` reboots the chip.
- **Smallest fix:**
  1. In `src/dac_task.c:554`, change `.intr_priority = 3` to `.intr_priority = 1` so `on_alarm`, `stream_pre`, and `stream_post` run at the same interrupt level on Core 0 and cannot preempt each other, and guard shared state in `stream_pre`, `stream_post`, and `on_alarm` with a `static portMUX_TYPE s_isr_mux = portMUX_INITIALIZER_UNLOCKED;` (`portENTER_CRITICAL_ISR` / `portEXIT_CRITICAL_ISR`).
  2. In `isr_arm_next()` (`src/dac_task.c:113-115`), set `s_alarm_armed = true;` **before** `gptimer_set_alarm_action(s_timer, &s_alarm_cfg);`.
  3. In `on_alarm()` (`src/dac_task.c:126-129`), remove the `if (timer_now() < s_next_edge_time)` re-arm branch; when the hardware alarm for `s_next_edge_time` fires, apply the edge immediately.
  4. In `sdkconfig.defaults`, add `CONFIG_GPIO_CTRL_FUNC_IN_IRAM=y`.

#### [CRITICAL-3] Non-IRAM / Non-ISR-Safe Calls Inside `esp_task_wdt_isr_user_handler()` Cause Double-Fault `int_wdt` & Suppress Coredumps
- **Location:** `src/debug_tools.c:38-63`, `src/debug_tools.c:99-127`, `sdkconfig.debug:18-22`, `sdkconfig.defaults:59`.
- **What is wrong:**
  1. `esp_task_wdt_isr_user_handler()` (`src/debug_tools.c:38-63`) executes inside the Task Watchdog Timer interrupt (`IRAM_ATTR`), yet calls:
     - `snprintf()` and `strlcat()` (`src/debug_tools.c:50,55,56`) — ROM/flash libc functions that are not guaranteed IRAM-safe on all paths.
     - `esp_task_wdt_print_triggered_tasks(NULL, NULL, NULL)` (`src/debug_tools.c:61`) — this function logs via `ESP_LOGE`, which invokes `host_log_vprintf()` (`src/host_serial.c:171`) -> `host_write()` -> `xSemaphoreTake(s_tx_mutex, portMAX_DELAY)` (`src/host_serial.c:186`) and `espfleet_log_push()` -> `xSemaphoreTake(log_lock_, ...)` (`components/espfleet/src/EspFleet.cpp:881`).
  2. Calling `xSemaphoreTake(..., portMAX_DELAY)` from an ISR triggers a FreeRTOS `configASSERT` / double exception inside the TWDT ISR. Instead of cleanly entering the panic handler to write the flash coredump, Core 0 faults inside the ISR until the hardware Interrupt Watchdog (`int_wdt`) forces a raw hardware reset, leaving no coredump in flash and corrupting the reset reason.
  3. In `sdkconfig.debug:20-22`, `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y` is enabled and `# CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT=y` is commented out, contradicting the comment on line 18 ("Panic handler writes a full backtrace + task state to the coredump partition in flash and reboots silently").
  4. In `src/debug_tools.c:99-127`, `sample_cb()` runs every 100 ms in the `esp_timer` task on **Core 0** (priority 22, higher than `dac_task` priority 10!) and calls `uxTaskGetStackHighWaterMark(h)` across 9 tasks (`src/debug_tools.c:112`), scanning kilobytes of stack memory on Core 0 while `dac_task` is streaming.
- **How it fails:** Any `task_wdt` event in the debug build crashes inside `esp_task_wdt_isr_user_handler()` before the panic handler can save a coredump, turning `task_wdt` into an unrecorded `int_wdt` / `unknown` reboot (Crash A & Crash B symptoms).Meanwhile, `sample_cb` steals Core 0 CPU cycles from `dac_task` every 100 ms.
- **Smallest fix:**
  1. In `esp_task_wdt_isr_user_handler()` (`src/debug_tools.c:38-63`), first call `laser_io_gate_kill()`, then copy `pcTaskGetName(xTaskGetCurrentTaskHandleForCore(0))` and `pcTaskGetName(xTaskGetCurrentTaskHandleForCore(1))` into `s_rtc.wdt_starved` using a simple `IRAM_ATTR` byte loop—remove `snprintf`, `strlcat`, and `esp_task_wdt_print_triggered_tasks()`.
  2. In `sdkconfig.debug:20-22`, set `CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT=y` (and `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=n`), and enable `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y` and `CONFIG_ESP_TASK_WDT_PANIC=y` in `sdkconfig.defaults` as well so production units also capture coredumps and reboot cleanly on TWDT.
  3. In `src/debug_tools.c:112`, do not call `uxTaskGetStackHighWaterMark()` inside the 100 ms `esp_timer` callback on Core 0; sample only `esp_get_free_internal_heap_size()` in `sample_cb`, or move stack watermark checks to a 5 s low-priority check when `!dac_task_is_streaming()`.

#### [CRITICAL-4] DAC Configuration Loss (`0x19: 0x44 -> 0x00`, Then All `0xff`), Missing Hardware Reset Recovery, and Laser Left Armed on Stream-Open Failure (Root Cause of Crash B & Brief Item 3)
- **Location:** `src/ad3552r_board.c:228-281`, `src/ad3552r_board.c:333-427`, `src/dac_task.c:258`, `src/dac_task.c:322-341`, `src/dac_task.c:372-415`.
- **What is wrong:**
  1. **Stream held open across slow ATmega barriers:** During streaming (`s_session_open == true`), `CS` is held low continuously (`SPI_TRANS_CS_KEEP_ACTIVE`) and `QSPI` (`BOARD_PIN_SPI_OP1`, GPIO 7) is held high (`src/ad3552r_board.c:362-367`). When a power or PRR barrier chunk arrives (`src/dac_task.c:390-393`), `dac_task` waits for `s_outstanding == 0` (`src/dac_task.c:388`) and calls `apply_barrier(c)` (`src/dac_task.c:332-341`) **without calling `session_close()`**. `CS` remains asserted low in quad-SPI stream mode for up to 50 ms while the ATmega switches DB25 power lines (`STATUS.md:110`). Any transient noise on `SCK` or `CS` shifts the AD3552R's 4-byte stream address loop (`STREAM_MODE = 4`) out of byte alignment, causing subsequent stream bytes to write to control registers instead of `0x4B`..`0x4E`!
  2. **No hardware reset (`BOARD_PIN_RST`) when wedged in QSPI/0xFF state:** `guard_check_and_repair()` (`src/ad3552r_board.c:260-281`) only checks 5 registers (`0x00`, `0x14`, `0x16`, `0x19`, `0x1A`) using single-lane SPI (`ad3552r_read_reg` / `ad3552r_write_reg`). It never checks `TRANSFER_REGISTER` (`0x0F`, where `MULTI_IO_MODE` bits `[7:6]` lock the chip into Quad SPI), and if the chip is stuck in Quad-SPI mode or brownout-reset (`0x19: 0x00` or all reads `0xff`), single-lane writes are ignored (`guard: 0x19 was 0x00, rewrite gave 0xff`). `ad3552r_board.c` never toggles `BOARD_PIN_RST` (GPIO 9) after initial boot!
  3. **Laser armed before `session_open()` and left armed when `session_open()` fails:** In `try_start()` (`src/dac_task.c:396-410`), `do_arm(true)` (`src/dac_task.c:400`) and `apply_power()` (`src/dac_task.c:403`) execute **before** `session_open(s_prr_hz)` (`src/dac_task.c:407`). When `session_open()` fails (`-5`), `try_start()` calls `drop_paced(c)` (`src/dac_task.c:409`), leaving the laser **armed** (`s_armed = true`) with marking power latched on the ATmega while dropping every chunk! Furthermore, `ad3552r_board_stream_open()` (`src/ad3552r_board.c:357`) and `session_open()` (`src/dac_task.c:258`) emit two `ESP_LOGE` lines on **every dropped chunk** (every 20.48 ms = ~100 error log lines/sec from Core 0 taking `s_tx_mutex`).
- **How it fails:** If an EMI glitch corrupts the AD3552R during a barrier or between segments, the DAC enters `0x00` / `0xff` readback state and cannot recover without a power cycle; `try_start()` arms the laser, fails `session_open()`, keeps the laser armed, and floods the serial/log path on every chunk.
- **Smallest fix:**
  1. In `src/dac_task.c:390-393` (and before any `apply_barrier` / `apply_power` while `s_session_open` is true), call `session_close()` first so `CS` is deasserted high and `QSPI` (GPIO 7) is driven low before talking to the ATmega; `try_start()` will cleanly re-open the stream on the next marking chunk.
  2. Add `ad3552r_board_recover(void)` in `src/ad3552r_board.c` that drives `BOARD_PIN_SPI_OP1` (GPIO 7) low, releases any held CS transaction, pulses `BOARD_PIN_RST` (GPIO 9) low for 1 ms then high, waits 5 ms, re-runs the full register configuration (`0x00`, `0x0F`, `0x14`..`0x1A`), and restores `s_last_code[0..1]`. Call `ad3552r_board_recover()` whenever `guard_check_and_repair()` fails to repair a register or when `ad3552r_board_stream_open()` fails its `STREAM_MODE`/`TRANSFER` readback check (`src/ad3552r_board.c:344-359`), and retry `stream_open` once.
  3. In `try_start()` (`src/dac_task.c:396-410`), open the stream (`session_open(s_prr_hz)`) **before** `do_arm(true)`, or if `session_open()` fails even after `ad3552r_board_recover()`, immediately call `laser_io_gate_kill()`, `do_arm(false)`, `apply_power(0)`, set a sticky DAC fault (`dac_task_has_fault()`), and suppress repeated per-chunk `ESP_LOGE` logs.

#### [CRITICAL-5] Z-Bro TCP Link Drops ~10 s into Homing and Logs `waiting for reply type 2/3: 5 s` Every 1–4 Min (Brief Item 4)
- **Location:** `src/zbro_link.c:121-177`, `src/zbro_link.c:213`, `src/zbro_link.c:228-286`, `src/zbro_link.c:314-391`, `src/net_wifi.c:164-243`, `CompanionZ/main/zb_net.c:216-285`, `CompanionZ/main/zb_net.c:482-538`.
- **What is wrong:**
  1. **WiFi modem power-save (`WIFI_PS_MIN_MODEM`) is left enabled on both S3 and C3:** Neither `src/net_wifi.c` nor `CompanionZ/main/zb_net.c` calls `esp_wifi_set_ps(WIFI_PS_NONE)`. With `WIFI_PS_MIN_MODEM` active on both endpoints and `espfleet` running periodic blocking HTTP POSTs (`timeout_ms = 6000` / `4000` at `components/espfleet/src/EspFleet.cpp:478,976`) and 2000 ms mDNS queries (`EspFleet.cpp:834`) on the shared radio, TCP round-trips periodically stall for several seconds.
  2. **C3 drops client & aborts homing when 10 Hz `POS` stream fills TCP send buffer for >500 ms (`CompanionZ/main/zb_net.c:256-285,520-536`):** During homing, the C3 sends unsolicited `POS` lines at `rpt_hz = 10` (every 100 ms, `CompanionZ/main/zb_net.c:525`). In `send_line()` (`CompanionZ/main/zb_net.c:269-280`), if `send(..., MSG_DONTWAIT)` returns `EAGAIN` for >`SEND_WAIT_MS` (500 ms), the C3 logs `send failed ... dropping the client`, calls `shutdown(s_cfd, SHUT_RDWR)` (closing the TCP connection from the C3 side), and at `zb_net.c:535` calls `stop_orphan_op("client disconnected")` -> `zb_motion_stop()`, **aborting the in-progress homing cycle ~10 s in**!
  3. **S3 stops sending `PING` whenever `exp != EXP_NONE` (`src/zbro_link.c:347`), tripping C3's 3 s `PING_TIMEOUT_MS` (`CompanionZ/main/zb_net.c:516`):** On the C3, `zb_net.c:516` aborts any active client operation if no bytes are received for `PING_TIMEOUT_MS` (`3000` ms). On the S3, `session()` (`src/zbro_link.c:347`) only sends `PING` when `exp == EXP_NONE`. If any reply (`OK` or `POS`) is delayed >3 s, the S3 withholds `PING`, causing the C3 to trigger `stop_orphan_op("no PING for 3 s")`, or at 5 s (`REPLY_MS = 5000`, `src/zbro_link.c:375`) the S3 logs `C3 not answering (waiting for reply type 2/3: 5 s), dropping the link` (`EXP_DISCARD = 2`, `EXP_POS_DISCARD = 3`).
  4. **Infinite boot auto-home loop on reconnect (`src/zbro_link.c:315,334-345,385-391`):** `autohome_tried` is a local variable inside `session()` (`src/zbro_link.c:315`). When the TCP link drops ~10 s into boot auto-home (`s_auto == ZBRO_AUTO_HOMING`), `session()` exits without clearing `s_auto`, reconnects 1 s later, resets `autohome_tried = false`, and immediately sends `"HOME"` again (`src/zbro_link.c:340`), repeating the 10 s home -> drop -> abort -> reconnect -> home cycle forever.
  5. **Blocking `send()` without `SO_SNDTIMEO` on S3 (`src/zbro_link.c:213,262-286`):** `tcp_connect()` clears `O_NONBLOCK` (`src/zbro_link.c:213`) and sets no `SO_SNDTIMEO`, so `send(fd, ...)` in `send_line()` (`src/zbro_link.c:267`) can block `zbro_link` indefinitely on a half-open TCP socket (making the `errno == EAGAIN` branch at line 271 unreachable dead code).
- **How it fails:** Homing aborts ~10 s in when the C3 drops the TCP socket or times out `PING`; under normal idle/job use, WiFi power-save + `espfleet` stalls cause `waiting for reply type 2/3: 5 s` drops every 1–4 minutes.
- **Smallest fix:**
  1. Call `esp_wifi_set_ps(WIFI_PS_NONE)` after `esp_wifi_start()` in `src/net_wifi.c:207,242` (and in `CompanionZ/main/zb_net.c:235` when building CompanionZ).
  2. In `src/zbro_link.c:315`, make `s_autohome_tried` a `static bool` at file scope (only reset via explicit `$HZ`), and if `session()` exits while `s_auto == ZBRO_AUTO_HOMING`, set `s_auto = ZBRO_AUTO_FAILED` (`src/zbro_link.c:385-391`).
  3. In `src/zbro_link.c:267`, pass `MSG_DONTWAIT` to `send(fd, p, left, MSG_DONTWAIT)` so the existing `EAGAIN` retry loop (`src/zbro_link.c:271-276`) actually works.
  4. In `src/zbro_link.c:347-356`, continue sending `PING` every `PING_MS` (1000 ms) even while waiting on a previous `EXP_DISCARD` / `EXP_POS_DISCARD` so the C3's 3 s `PING_TIMEOUT_MS` never starves, and increase `REPLY_MS` (`src/zbro_link.c:32`) from `5000` to `8000` ms to ride through any WiFi retransmission window.
  5. (If CompanionZ is also updated later: in `CompanionZ/main/zb_net.c:525`, skip unsolicited 10 Hz `send_pos()` if the socket send buffer is non-empty instead of calling `shutdown(s_cfd, SHUT_RDWR)` on `EAGAIN`.)

---

### 1.3 High-Severity Findings

#### [HIGH-1] `$231` Auto Guide (`CAL_AUTO_GUIDE`) Activates Mid-Job Under `M3` and Stalls the Motion Pipeline (Brief Item 5)
- **Location:** `src/grbl.c:449-454`, `src/grbl.c:541-545`, `src/calib.c:58`.
- **What is wrong:**
  1. In `emit_linear()` (`src/grbl.c:541-545`), on every `G0`/`G1` move where `s_m.laser == 3 && !(s_m.s > 0.0f) && !s_m.preview && calib_get(CAL_AUTO_GUIDE) > 0.5f`, `grbl` immediately calls `atmega_link_set_guide(true)` and sets `s_auto_guide = true`.
  2. Because `emit_linear()` does **not** call `wait_idle()` before `atmega_link_set_guide(true)`, the red guide laser turns on immediately over UART while up to 512 segments of `S > 0` cutting moves are still queued in `motion_task` and `dac_task`.
  3. Worse, on the very next `S > 0` move in an `M3` job, `sync_laser_on()` (`src/grbl.c:449-454`) sees `s_auto_guide == true` and calls `wait_idle()`, which **drains the entire motion pipeline to zero** before turning the guide off and submitting the next cutting segment!
- **How it fails:** Any job or macro using `M3` (Constant Power mode, `Grbl (Compat)` dialect, or `M3` with `G0`/`G1 S0` travel moves) turns on the guide laser while previous cuts are still executing and stalls the galvo motion pipeline to a full stop before every cut segment, causing severe dwell burns and underruns.
- **Smallest fix:**
  1. Track whether any burning segment (`S > 0`) has been emitted since the last `M5` or idle state (`s_m.m3_burned_since_m5`). Only allow `emit_linear()` (`src/grbl.c:541-545`) to enter `s_auto_guide` when `!s_m.m3_burned_since_m5` (i.e. pure `M3 S0` framing where `M3` was never used with `S > 0`), and call `wait_idle()` before `atmega_link_set_guide(true)` when transitioning `s_auto_guide` from `false` to `true`.
  2. Change the default of `CAL_AUTO_GUIDE` (`$231` in `src/calib.c:58`) from `1.0f` to `0.0f` if the owner prefers explicit `M66`/`M67` or Rayforge's dry-run pointer offset (see Open Question 1).

#### [HIGH-2] Premature `galvo_out_flush()` and Early `reap()` Trigger False and Real `stream underrun with the laser on` (Brief Item 6)
- **Location:** `src/motion_task.c:208-227`, `src/dac_task.c:164-175`, `src/dac_task.c:240`, `src/dac_task.c:476-494`, `src/galvo_out.c:277-293`, `src/galvo_out.c:367-381`.
- **What is wrong:**
  1. **Premature flush with `gate_end = true` (`src/motion_task.c:208-218`):** When `s_queue` is temporarily empty while `motion_gen_has_pending(&s_gen)` is true (`in_polyline == true`), the last tick pushed into `galvo_out` had `power > 0` (`s_cur_gate == true`). Line 209 calls `galvo_out_flush()` **before** computing `allowed_us` (line 212). This submits a partial chunk to `dac_task` with `c->gate_end = true`. If `allowed_us < 1000` us (`wait_ms < 1`), `motion_task` loops around to line 185 (`motion_gen_finish(&s_gen); galvo_out_flush();`), but `dac_task` (on Core 0) is already clocking out the short partial chunk and can hit `left <= 0` in `stream_post()` (`src/dac_task.c:165`) with `c->gate_end == true` (`s_isr_drain_lit = true`) before the closing chunk arrives!
  2. **`reap()` checks `!s_next` before `ready_q` is dequeued (`src/dac_task.c:477` vs `487`):** In `service()` (`src/dac_task.c:476-488`), when `s_isr_drained` is set, `reap()` runs at line 477 *before* `xQueueReceive(g_galvo.ready_q, &s_next, 0)` runs at line 487. Inside `reap()` (`src/dac_task.c:240`), `s_next` is still `NULL` even when `g_galvo.ready_q` has chunks ready, logging a false `stream underrun with the laser on` warning.
  3. **Barrier chunks drain `s_outstanding` while `s_out_gate` is still true (`src/galvo_out.c:367-381`, `src/dac_task.c:387-394`):** When power changes between two adjacent marks (`power != 0` and `want != s_atmega_power`), `request_barrier()` (`src/galvo_out.c:379`) calls `submit_cur()` while `s_cur_gate` is still `true` (because the gate-off edge lags by `laser_off_delay` = 120 us in the delay FIFO). When `dac_task` sees `s_next->barrier` (`src/dac_task.c:387`), it stops pipelining and lets `s_outstanding` drain to 0 (`stream_post` sees `left <= 0` and `c->gate_end == true`), setting `s_isr_drain_lit = true` on a planned power barrier!
- **How it fails:** The laser gate is killed mid-polyline and `[MSG:W: dac_task: stream underrun with the laser on (host too slow?)]` is logged during normal jobs (especially multi-layer jobs, constant-power `M3` moves, or dynamic-power `M4` raster/vector passes with power transitions).
- **Smallest fix:**
  1. In `src/motion_task.c:208-227`, compute `allowed_us` **before** calling `galvo_out_flush()`. If `wait_ms < 1`, call `motion_gen_finish(&s_gen)` and then `galvo_out_flush()` once so the partial chunk and the polyline close (`laser_off_delay` + `mark_delay`, `gate_end = false`) go out together without exposing an unclosed `gate_end = true` chunk at low buffer margin.
  2. In `src/dac_task.c:476-488`, dequeue `s_next` from `g_galvo.ready_q` (if `s_next == NULL`) **before** calling `reap()`, and in `reap()` (`src/dac_task.c:240`), do not log an underrun when `s_next != NULL`.
  3. In `src/galvo_out.c:367-381`, before calling `request_barrier()` when `s_out_gate` is true, flush the gate-off delay (`s_delay_off_ticks`) or only emit power barriers after the polyline's `laser_off_delay` has brought `s_out_gate` low.

#### [HIGH-3] Laser Safety Invariant Gaps on ATmega Faults, Disarm Failures, and GRBL Alarm States
- **Location:** `src/atmega_link.c:421-451`, `src/dac_task.c:322-370`, `src/grbl.c:605-607`, `src/grbl.c:650-653`, `src/grbl.c:1274-1299`.
- **What is wrong:**
  1. **ATmega fault / link timeout does not kill ESP32 gate or abort motion (`src/atmega_link.c:421-443`):** When the ATmega reports `ATMEGA_ST_EMERGENCY` (DB25 pin 16 fault, pin 17 5V loss, or watchdog timeout) or when UART0 fails 5 polls in a row (`s_consecutive_fail == 5`, `src/atmega_link.c:436`), `atmega_link` only logs an `ESP_LOGE`. It never calls `laser_io_gate_kill()`, never aborts `motion_task`, and never sets a GRBL `ALARM`.
  2. **Failed `do_arm(false)` / `apply_power(0)` is never retried (`src/dac_task.c:322-370`):** In `do_arm()` (`src/dac_task.c:363-366`), if `atmega_link_set_arm(false)` returns `false` (UART timeout/CRC error), `dac_task` still sets `s_armed = false` (`src/dac_task.c:366`) and never retries disarming until the ATmega's 1 s heartbeat watchdog expires (or longer if polls still succeed). Similarly, `apply_power()` (`src/dac_task.c:327`) updates `s_power_applied = p` even when `atmega_link_set_power(p)` returns `false`.
  3. **Entering `ALARM` in `grbl.c` leaves the laser armed (`src/grbl.c:605-607,650-653,1282`):** When `s_alarm` is set (`ALARM:2` soft limit at `src/grbl.c:605` or `ALARM:3` Z-Bro abort at `src/grbl.c:651`), `grbl` rejects subsequent G-code lines (`E_ALARM`, skipping the job's `M5` / `M11` postscript) without calling `laser_io_gate_kill()` or disarming the laser (`s_m.armed` remains `true`).
- **How it fails:** A single corrupted UART frame on disarm or a soft-limit / Z-axis alarm leaves the laser armed (`MO` high on DB25 pin 18) or leaves the ESP32 gate toggling during an ATmega fault.
- **Smallest fix:**
  1. In `src/atmega_link.c:421-443`, whenever `ATMEGA_ST_EMERGENCY` is set or `s_consecutive_fail >= 5`, immediately call `laser_io_gate_kill()`.
  2. In `src/dac_task.c:322-370`, only update `s_armed = arm` and `s_power_applied = p` when `atmega_link_set_arm()` / `atmega_link_set_power()` returns `true`; if `do_arm(false)` fails, keep `s_armed = true` so `service()` (`src/dac_task.c:525`) retries on the next 20 ms tick, and immediately call `laser_io_gate_kill()`.
  3. Add a helper `enter_alarm(int code)` in `src/grbl.c` (used at lines 605, 651, etc.) that sets `s_alarm = code`, calls `laser_io_gate_kill()`, `galvo_out_set_arm(false)`, `galvo_out_set_power(0)`, `atmega_link_set_guide(false)`, and sets `s_m.armed = false`, `s_m.laser = 0`, `s_m.s = 0.0f`.

#### [HIGH-4] Core 1 Task Priority Inversion Starves `motion_task` and `atmega_link`, and Unpinned WiFi/`espfleet` Tasks Preempt Core 0
- **Location:** `src/host_serial.c:428`, `src/net_server.c:292-293`, `src/grbl.c:1353`, `src/motion_task.c:339`, `src/atmega_link.c:465`, `src/zbro_link.c:646`, `components/espfleet/src/EspFleet.cpp:277`, `sdkconfig.defaults`.
- **What is wrong:**
  1. On Core 1, `motion` (which must generate 100 kHz ticks continuously to keep the 8-chunk / 163.8 ms DMA pool fed) runs at priority **8** (`src/motion_task.c:339`), which is **lower** than `grbl` (9, `src/grbl.c:1353`), `host_rx` (10, `src/host_serial.c:428`), `net_srv` (10, `src/net_server.c:292`), and `net_tx` (10, `src/net_server.c:293`).
  2. `atmega` runs at priority **5** (`src/atmega_link.c:465`), even though `dac_task` (Core 0, priority 10) blocks synchronously on `atmega_link_set_power()` / `atmega_link_set_arm()` (`src/dac_task.c:326,363`) with a 50 ms timeout (`CMD_TIMEOUT_MS`, `src/atmega_link.c:29`)! When `motion` (8) and `grbl` (9) are busy on Core 1, `atmega` (5) is starved, delaying or timing out `dac_task` barriers and the 200 ms safety heartbeat.
  3. `EspFleet::begin()` (`components/espfleet/src/EspFleet.cpp:277`) creates the `espfleet` task with `xTaskCreate()` (`tskNO_AFFINITY`) instead of `xTaskCreatePinnedToCore(..., 1)`, allowing it to run on Core 0. Similarly, `sdkconfig.defaults` does not set `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1=y` or `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU1=y`, so WiFi (prio 23) and lwIP `tiT` (prio 18) run on Core 0 and preempt `dac_task` (prio 10)!
- **How it fails:** WiFi/lwIP tasks on Core 0 preempt `dac_task` between 20.48 ms chunks; on Core 1, G-code parsing (`grbl`) and TCP/USB I/O preempt `motion_task` and `atmega_link`, causing chunk pool underruns and ATmega barrier timeouts.
- **Smallest fix:**
  1. Reorder Core 1 priorities strictly by deadline: `atmega` = **10** (`src/atmega_link.c:465`), `motion` = **9** (`src/motion_task.c:339`), `host_rx` = **8** (`src/host_serial.c:428`), `net_srv` = **8** (`src/net_server.c:292`), `grbl` = **7** (`src/grbl.c:1353`), `net_tx` = **5** (`src/net_server.c:293`), `zbro` = **5** (`src/zbro_link.c:646`).
  2. In `sdkconfig.defaults`, add `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1=y` and `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU1=y`.
  3. In `components/espfleet/src/EspFleet.cpp:277`, change `xTaskCreate(taskEntry, "espfleet", 6144, this, 3, &task_)` to `xTaskCreatePinnedToCore(taskEntry, "espfleet", 6144, this, 3, &task_, 1)`.

#### [HIGH-5] Zero Application Tasks Subscribed to the Task Watchdog Timer (TWDT)
- **Location:** `sdkconfig.defaults:59`, `src/dac_task.c:568-601`, `src/atmega_link.c:383-453`, `src/freeze_watch.c:112-160`.
- **What is wrong:**
  1. Only the FreeRTOS IDLE tasks are subscribed to TWDT. Neither `dac_task` nor `atmega_link` calls `esp_task_wdt_add(NULL)`.
  2. If `dac_task` or `atmega_link` deadlocks while blocked on a semaphore/queue (`s_done_sem`, `s_tx_mutex`, `s_cmd_q`), the IDLE task continues to run, so TWDT **never triggers** even though the real-time DAC loop or ATmega safety heartbeat task is permanently hung.
  3. `freeze_watch` (`src/freeze_watch.c`) is compiled only under `CONFIG_WM_DEBUG`, and even then it only logs a warning after 2 s without resetting or killing the gate if `dac_task` is wedged.
- **How it fails:** A semaphore deadlock in `dac_task`, `atmega_link`, or `grbl` hangs the machine indefinitely without triggering a watchdog reset.
- **Smallest fix:** Subscribe `dac_task` (`src/dac_task.c:570`, which wakes at least every `IDLE_POLL_MS = 20 ms`) and `atmega_link_task` (`src/atmega_link.c:389`, which wakes at least every `POLL_PERIOD_MS = 200 ms`) to TWDT via `esp_task_wdt_add(NULL)` and call `esp_task_wdt_reset()` once per main loop iteration.

---

### 1.4 Medium-Severity Findings

#### [MEDIUM-1] `host_log_vprintf()` Blocks `dac_task` on `s_tx_mutex` and Feedback-Loops with `espfleet`
- **Location:** `src/host_serial.c:171-225`, `src/host_serial.c:89-99`, `components/espfleet/src/EspFleet.cpp:878-1003`.
- **What is wrong:**
  1. `host_log_vprintf()` (`src/host_serial.c:171-225`) redirects all `ESP_LOG*` calls to `host_write()`, which calls `xSemaphoreTake(s_tx_mutex, portMAX_DELAY)` (`src/host_serial.c:186`) and writes to USB-Serial-JTAG with a 50 ms timeout (`src/host_serial.c:96`). Whenever `dac_task` on Core 0 calls `ESP_LOGI`/`ESP_LOGW`/`ESP_LOGE` (`src/dac_task.c:235,241,258,285,303,365,530`), `dac_task` blocks on `s_tx_mutex` held by Core 1 tasks (`grbl`, `zbro_link`, `espfleet`), violating the `AGENTS.md` rule: *"never block `dac_task` on a core-1 task"*.
  2. `host_log_vprintf()` also forwards every log line to `espfleet_log_push()` (`src/host_serial.c:207`). Inside `EspFleet.cpp`, `shipLogs()`, `sendHeartbeat()`, `discoverHub()`, and `esp_http_client` call `ESP_LOGI`/`ESP_LOGW` (`components/espfleet/src/EspFleet.cpp:543,805,873,1114,1120,1163`). Each `ESP_LOG` call inside `espfleet` re-enters `host_log_vprintf()`, which emits `[MSG:espfleet: ...]` to Rayforge's console and pushes another log entry back into `espfleet_log_push()`.
- **How it fails:** Core 1 USB/TCP stalls block `dac_task` whenever `dac_task` logs; `espfleet` HTTP failures create self-amplifying log traffic and spam the GRBL serial stream with internal `[MSG:...]` lines.
- **Smallest fix:**
  1. In `host_log_vprintf()` (`src/host_serial.c:186`), if `xPortGetCoreID() == 0` (or if called from `dac_task`), use a non-blocking `xSemaphoreTake(s_tx_mutex, 0)` (or push to the lock-free `net_server_tx_bytes` ring / skip blocking USB write if contended) so Core 0 never blocks on Core 1.
  2. Suppress `espfleet_log_push()` re-entrancy when `strcmp(pcTaskGetName(NULL), "espfleet") == 0` (or via a thread-local / task-handle guard in `host_log_vprintf()`), and set `esp_log_level_set("espfleet", ESP_LOG_WARN)` and `esp_log_level_set("HTTP_CLIENT", ESP_LOG_ERROR)` in `src/main.c`.

#### [MEDIUM-2] `galvo_out_abort()` Race With `motion_task_loop()` Leaves Stale Ticks or Corrupts `s_cur`
- **Location:** `src/galvo_out.c:238-256`, `src/galvo_out.c:538-563`, `src/motion_task.c:115-133`, `src/grbl.c:1251-1256`.
- **What is wrong:**
  1. When `Ctrl-X` is processed by `grbl_task` (`src/grbl.c:1251-1256`), `grbl_task` calls `motion_abort()` (`src/motion_task.c:309`, which only sets `s_abort_req = true`) and **immediately** calls `galvo_out_abort()` (`src/galvo_out.c:538`) from `grbl_task` **without waiting for `motion_task` to acknowledge `s_abort_req`**.
  2. While `galvo_out_abort()` is mutating `s_cur`, `s_cur_idx`, `s_fifo_head`, `s_fifo_count`, and draining `g_galvo.ready_q` (`src/galvo_out.c:544-557`) from `grbl_task`, `motion_task` (on the same core) can be preempted mid-execution inside `galvo_out_tick()` (`src/galvo_out.c:391-505`) or blocked inside `alloc_chunk()` (`src/galvo_out.c:238`).
  3. Worse, if `motion_task` was blocked inside `alloc_chunk()` waiting on `g_galvo.free_q` (`src/galvo_out.c:238`), draining `ready_q` into `free_q` (`src/galvo_out.c:553-554`) immediately wakes `motion_task`, which then finishes its interrupted `galvo_out_tick()` and pushes residual ticks from the aborted move into the newly cleared chunk pool *before* `motion_task_loop()` returns to the top of its loop to check `s_abort_req` (`src/motion_task.c:115`)!
- **How it fails:** A soft reset (`Ctrl-X`) during a heavy raster/vector stream can race on `s_cur` / `s_fifo` pointers or leak one post-abort chunk into `ready_q` after `galvo_out_abort()` has supposedly drained the pipeline.
- **Smallest fix:** Perform `galvo_out_abort()` **inside `motion_task`** when `s_abort_req` is handled (`src/motion_task.c:115-132`), or have `motion_abort()` post `s_abort_req = true`, unblock `free_q` by draining `ready_q`, and wait on an ack semaphore/flag from `motion_task` before resetting `galvo_out` state. Also check `s_abort_req` inside `motion_task_loop`'s inner `while (motion_gen_tick(...))` loop (`src/motion_task.c:167`).

#### [MEDIUM-3] Data Races on Cross-Task Shared State (`s_idle`, `s_pos_x_mm`, `s_pos_y_mm`, `s_status`, `s_calib`)
- **Location:** `src/motion_task.c:39-41`, `src/motion_task.c:292-322`, `src/atmega_link.c:46-54`, `src/atmega_link.c:285-296`, `src/calib.c:118-127`, `src/calib.c:199-212`.
- **What is wrong:**
  1. In `src/motion_task.c:39-41`, `s_pos_x_mm`, `s_pos_y_mm`, and `s_idle` are plain `volatile float` / `volatile bool` written by `motion_task` and read/written by `grbl_task` (`motion_get_position()` at `src/motion_task.c:314`, `motion_abort()` at `src/motion_task.c:309`). In `motion_abort()`, `grbl_task` writes `s_pos_x_mm = 0.0f; s_pos_y_mm = 0.0f; s_idle = true;` while `motion_task` is still running and updating `s_pos_x_mm = x_mm` (`src/motion_task.c:171`), so `motion_task` overwrites the zeroed position one tick later.
  2. In `src/atmega_link.c:285-291`, `atmega_link_get_status()` does a struct copy `*out = s_status;` without taking a lock while `atmega_link_task` updates `s_status` (`src/atmega_link.c:411-418`), allowing a torn read across `flags`, `applied_power`, `fault_code`, and `rx_crc_errors`.
  3. In `src/calib.c:199-203`, `calib_set()` writes `s_values[id] = value;` from `grbl_task` while `motion_task`, `galvo_out`, and `dac_task` call `calib_get()`. (Note: 32-bit aligned `float` loads/stores on Xtensa LX7 are single-instruction `lsi`/`ssi`, so individual floats do not tear, but multi-field reads in `atmega_link_get_status` and `motion_get_position` can tear.)
- **How it fails:** Status reports (`?`) can report inconsistent `(X, Y)` pairs or torn `atmega_status_t` structs, and `Ctrl-X` position reset races with `motion_task`'s last tick.
- **Smallest fix:** Move the `s_pos_x_mm = 0.0f; s_pos_y_mm = 0.0f;` reset inside `motion_task`'s `s_abort_req` handler (`src/motion_task.c:121`), and protect `s_status` (`src/atmega_link.c`) and `(s_pos_x_mm, s_pos_y_mm)` (`src/motion_task.c`) with a lightweight `portMUX_TYPE` spinlock.

#### [MEDIUM-4] NVS Calibration Persistence Opens/Commits Flash on Every `$n=val` Without Flash-Cache Impact Guard
- **Location:** `src/calib.c:26-41`, `src/calib.c:199-255`, `src/net_wifi.c:49-83`.
- **What is wrong:**
  1. Every `calib_set()` (`src/calib.c:205`) and `calib_reset_defaults()` (`src/calib.c:241`, which loops 32 times!) opens NVS (`nvs_open`), writes one float (`nvs_set_u32`), calls `nvs_commit()`, and calls `nvs_close()`. In `calib_reset_defaults()` (`$RST=$`), this performs **32 separate NVS open + flash write + commit + close cycles** back-to-back!
  2. Each NVS flash write disables the SPI0/1 instruction/data cache on BOTH cores during the flash page program/erase window unless all active ISRs on Core 0 are 100% IRAM-safe (which currently fails because `CONFIG_GPIO_CTRL_FUNC_IN_IRAM=y` is missing—see CRITICAL-2).
  3. Worse, if a host sends `$n=val` while `dac_task_is_streaming()` is true, `grbl.c:892` calls `calib_set()` immediately without `wait_idle()`, both disabling flash cache during active galvo streaming and mutating live calibration values mid-stream!
- **How it fails:** `$RST=$` takes hundreds of milliseconds of flash writes (risking WDT/cache stalls), and `$n=val` during a job mutates calibration parameters mid-stream while stalling flash cache.
- **Smallest fix:**
  1. In `src/grbl.c:892` and `src/grbl.c:812` (`$RST=$`), call `wait_idle()` before `calib_set()` or `calib_reset_defaults()`.
  2. In `calib_reset_defaults()` (`src/calib.c:236-255`), open the `"calib"` NVS namespace once, call `nvs_erase_all(h)` (or set all keys in one handle), call `nvs_commit(h)` once, and `nvs_close(h)`.

#### [MEDIUM-5] Telnet/TCP Server Lacks Socket Keepalive and Can Hold `NET_OWNER_TCP` Indefinitely After Silent WiFi Disconnect
- **Location:** `src/net_server.c:150-241`, `src/host_serial.c:41-55`.
- **What is wrong:**
  1. When a TCP client connects on port 23 (`src/net_server.c:167-189`), `net_server_task` sets `TCP_NODELAY` (`src/net_server.c:177`) but **never enables `SO_KEEPALIVE`** (`TCP_KEEPIDLE`, `TCP_KEEPINTVL`, `TCP_KEEPCNT`).
  2. As soon as the TCP client sends 1 byte (e.g. `?`), `host_try_claim(HOST_OWNER_TCP)` (`src/net_server.c:218`) claims exclusive host ownership (`s_owner = HOST_OWNER_TCP`, `src/host_serial.c:49`), locking out USB (`HOST_OWNER_USB`) except for `Ctrl-X`.
  3. If the WiFi client laptops sleeps, crashes, or roams off WiFi without sending a TCP `FIN`/`RST`, `recv()` in `net_server_task` (`src/net_server.c:195`) never sees EOF (`r == 0`). `s_client_fd` stays open forever and `s_owner` remains `HOST_OWNER_TCP` unless someone manually sends `Ctrl-X` over USB.
- **How it fails:** A dropped WiFi session leaves port 23 occupied (rejecting reconnects with `"busy: another client is connected\r\n"`, `src/net_server.c:172`) and locks out USB control.
- **Smallest fix:** Enable `SO_KEEPALIVE` with `TCP_KEEPIDLE = 10`, `TCP_KEEPINTVL = 3`, `TCP_KEEPCNT = 3` (~19 s dead-peer detection) on `cfd` in `src/net_server.c:177-180`.

#### [MEDIUM-6] Blocking DNS / mDNS Resolution in `zbro_link` Holds Up Reconnects and `grbl_task`
- **Location:** `src/zbro_link.c:181-200`, `src/zbro_link.c:553-574`, `src/grbl.c:640-655`.
- **What is wrong:**
  1. `resolve_host()` (`src/zbro_link.c:188`) calls `mdns_query_a(host, 2000, &addr)` (2 s blocking timeout) and then falls back to `getaddrinfo()` (`src/zbro_link.c:194`, lwIP DNS timeout up to several seconds) on every reconnect attempt (`RECONNECT_MS = 1000 ms`, `src/zbro_link.c:28`).
  2. When `$300=1` (`CAL_Z_ENABLE = 1`) and the Z-Bro C3 is powered off or unreachable, `grbl.c:640` (`emit_z_move` for any G-code with `Z` words, e.g. multi-pass Z steps) calls `zbro_move_Sync()` -> `zbro_request()` (`src/zbro_link.c:553`), which waits 1500 ms (`CONNECT_WAIT_MS`) before failing and putting GRBL into `ALARM:3` (`src/grbl.c:651`).
- **How it fails:** Constant mDNS queries every 1–3 s add multicast WiFi chatter when the C3 is offline; if the C3 IP is already known from a prior connection, re-running `mdns_query_a` on every transient TCP reconnect wastes 2 s before `connect()` even starts.
- **Smallest fix:** Cache the last resolved `struct in_addr` in `src/zbro_link.c:181-200` and try `connect()` to the cached IP first on reconnect; only re-run `mdns_query_a` if `connect()` to the cached IP fails, and use exponential backoff (1 s -> 2 s -> 5 s -> 10 s) when the C3 is offline.

---

### 1.5 Low-Severity Findings

#### [LOW-1] `gcode_parse_line()` Accepts NaN/Inf Floats and Overflowing Integers Without Validation
- **Location:** `src/gcode_parse.c:164-174`, `src/grbl.c:881-895`.
- **What is wrong:** `strtof(p, &end)` (`src/gcode_parse.c:165` and `src/grbl.c:883`) accepts `"NAN"` and `"INF"` (or huge exponents that overflow to `INFINITY`). Passing `NaN` into `calib_set()` (`src/calib.c:200`: `if (value < d->min_val || value > d->max_val)` evaluates to `false` for `NaN` in IEEE-754!) allows `NaN` to be stored into `s_values[]` and NVS!
- **How it fails:** `$100=nan` bypasses `value < d->min_val || value > d->max_val` in `calib_set()` (`src/calib.c:200`), poisoning calibration in NVS and producing `NaN` coordinates in `motion_gen`.
- **Smallest fix:** Check `!isfinite(val)` in `src/gcode_parse.c:167`, `src/grbl.c:884`, and `src/calib.c:200` (`if (!isfinite(value) || value < d->min_val || value > d->max_val) return false;`).

#### [LOW-2] `partitions.csv` Has No OTA Slot Despite `espfleet` Linking `esp_https_ota`
- **Location:** `partitions.csv:1-7`, `components/espfleet/src/EspFleet.cpp:740-776`.
- **What is wrong:** `partitions.csv` defines only a single `factory` app partition (`0x1F0000` bytes) and `coredump` (`64K`), with no `ota_0`/`ota_1`/`otadata` partitions.
- **How it fails:** Not a runtime crash (if OTA is triggered via `espfleet`, `esp_ota_get_next_update_partition(NULL)` returns `NULL` and logs an error at `EspFleet.cpp:743`), but worth noting that 2 MB remains unused in the 4 MB flash (`0x200000`..`0x400000`).
- **Smallest fix:** Leave `partitions.csv` unchanged unless OTA is explicitly desired; ensure `EspFleet` OTA is disabled or cleanly rejected.

#### [LOW-3] `ArduinoCompanion` `apply_power()` Takes ~10 us Busy-Wait Inside UART Command Processing
- **Location:** `ArduinoCompanion/src/main.cpp:89-99`.
- **What is wrong:** `apply_power()` delays 5 us + 5 us = 10 us (`delayMicroseconds(5)` twice) while latching the 8-bit power word on DB25 pins 1–9. At 250 000 baud (40 us per byte), a 10 us delay is completely safe (the hardware UART has a 2-byte RX FIFO), so no bytes are lost.
- **How it fails:** Does not fail; documented here for completeness of the whole-firmware audit.
- **Smallest fix:** No change required in `ArduinoCompanion/src/main.cpp`.

---

## 2. Rayforge Controller Feature Gap Analysis

This section compares the WaveMaster firmware (`src/grbl.c`, `docs/rayforge/wavemaster-*.yaml`) against the Rayforge host codebase at `/Volumes/Projects/rayforge` (`rayforge/machine/driver/grbl/`, `rayforge/machine/models/`, `rayforge/pipeline/encoder/`).

| # | Feature / Capability | How Rayforge Uses It (`file:line` in `/Volumes/Projects/rayforge`) | Current WaveMaster State (`file:line` in this repo) | Verdict | Reason & Recommended Action |
| :- | :--- | :--- | :--- | :---: | :--- |
| **G1** | **Status Report `<State\|MPos:x,y,z\|FS:f,s\|Pn:...>`** | `parse_state()` in `rayforge/machine/driver/grbl/grbl_util.py:216-291` parses state (`Idle`, `Run`, `Hold`, `Jog`, `Alarm`, `Door`, `Home`, `Sleep`, `Check`) and `MPos:x,y,z`. | `send_status()` in `src/grbl.c:304-370` emits `<State\|MPos:x,y,z\|FS:0,s\|Pn:...>`. However, during Z-Bro moves/homing (`$HZ` or `G0 Z...`), `grbl_task` blocks inside `zbro_home()` / `zbro_move_sync()` (`src/grbl.c:640,966`), so `?` is still answered immediately via `s_status_req` (`src/grbl.c:1244`), **but** `state` reports `Idle` instead of `Home` or `Run` while Z is moving! | **worth-doing** | When Z-Bro is actively homing (`zbro_is_busy()` / `ZBRO_ST_HOMING`), `send_status()` (`src/grbl.c:320-325`) should report `Home` (or `Run` when `ZBRO_ST_MOVING`) so Rayforge's UI and `wait_for_idle` in `grbl_serial.py` / `grbl_network.py` do not think the machine is `Idle` while the Z column is still moving. |
| **G2** | **Single-Axis Homing (`$HX`, `$HY`, `$HZ`) vs All-Axis (`$H`)** | `GrblSerialDriver.home(axes)` (`rayforge/machine/driver/grbl/grbl_serial.py:328-347`) and `Dialect.home_axis_cmd` (`docs/rayforge/wavemaster-dialect.yaml:40` = `$H{axis}`) send `$HX`, `$HY`, or `$HZ` when the user clicks "Home X", "Home Y", or "Home Z" in Rayforge! | `handle_dollar()` (`src/grbl.c:962-975`) implements `$H` and `$HZ`, **but rejects `$HX` and `$HY` with `error:3` (`E_BAD_SYS_CMD`)!** Furthermore, `$H` (`src/grbl.c:971-974`) zeroes X/Y without homing Z even when `$300=1`. | **worth-doing** | Clicking "Home X" or "Home Y" in Rayforge currently fails with `error:3`. Accepting `$HX` (move X to 0 V), `$HY` (move Y to 0 V), and `$HZ` in `src/grbl.c:962-975` is a ~15-line change that makes all Rayforge homing buttons work cleanly. |
| **G3** | **Work Coordinate Offsets (`G54`–`G59`, `G10 L2`/`L20`, `G92`, `WCO:` in `?`)** | `Machine` (`rayforge/machine/models/machine.py:390-450`), `GrblSerialDriver.set_wcs_offset()` / `read_wcs_offsets()` (`grbl_serial.py:420-475`), `$#` parser (`grbl_util.py:92-120`), and `WCO:` parser (`grbl_util.py:258-281`). Note `docs/rayforge/wavemaster-machine.yaml:22` sets `supports_wcs: false`. | `grbl.c` parses `G54` (`src/grbl.c:568` is a no-op), `G92` (`src/grbl.c:693-702` overwrites `s_m.x`/`s_m.y` directly without moving galvos or storing an offset!), and `$#` (`src/grbl.c:756` prints hardcoded `[G54:0.000,0.000,0.000]` + `[G92:0.000,0.000,0.000]`). Worse, `G92 X0 Y0` lies to `s_m.x`/`s_m.y` so subsequent moves go to the wrong DAC voltage! | **worth-doing** | Store a real `s_wcs_x`, `s_wcs_y`, `s_wcs_z` (or `s_g92_x/y/z`) offset in `src/grbl.c` applied in `emit_linear()` / `send_status()` (`WCO:`), and support `G10 L20 P1` + `G92` + `G92.1` + accurate `$#` output. Without this, `G92` corrupts the galvo voltage mapping (`s_m.x` diverges from actual mirror position)! |
| **G4** | **Bidirectional Raster Scan Offset Table (`BidirectionalOffsetTable`) vs `$216` Phase** | `BidirectionalOffsetTable` (`rayforge/machine/models/bidir_offset.py:1-28`) and `rust_helpers.py:140-175` apply speed-dependent X-coordinate offsets to alternating raster scanlines on the **host side** before emitting G-code (`G1 X... S...`). | WaveMaster implements `$210`/`$211` (`CAL_LASER_ON_DELAY_US` / `CAL_LASER_OFF_DELAY_US`, `src/calib.c:49-50`) in the gate delay FIFO (`src/galvo_out.c:309-330`) and `$216` (`CAL_BI_RASTER_PHASE_DEG`, `src/calib.c:55`, which currently has **zero references** in `src/*.c`—it is a dead setting!). | **skip** (document) | Rayforge already computes bidirectional raster scan offsets in its Rust pipeline (`BidirectionalOffsetTable`) and shifts the `G1 X` coordinates directly, while WaveMaster's hardware FIFO (`$210`/`$211`) handles constant galvo-to-laser lag in microseconds. Implementing a second bidirectional shift in firmware would double-compensate. Document `$216` as reserved/unused and configure `BidirectionalOffsetTable` or `$210`/`$211` in `docs/RAYFORGE.md`. |
| **G5** | **Framing / Outline Preview (`build_frame_ops`, `M3 S0`, Guide Laser)** | `Driver.run_frame()` / `build_frame_ops()` (`rayforge/machine/driver/driver.py:211-265`) builds a closed framing path using `PowerMode.CONSTANT` (`M3`) at `head.frame_power_percent` (default `0%` in `wavemaster-machine.yaml:41`), wrapped in job preamble/postscript or `pointer_offset` dry-run (`driver.py:240`). | WaveMaster provides `M64`/`M65` (guide trace mode, `src/grbl.c:663-668`), `M66`/`M67` (guide on/off, `src/grbl.c:669-676`), and `$231` (`CAL_AUTO_GUIDE`, `src/grbl.c:541-545`). Because `$231` triggers on *any* `M3 S0` move, it breaks normal `M3` cutting jobs (see HIGH-1). | **worth-doing** | Fix `$231` (`CAL_AUTO_GUIDE`) so it only activates when an `M3` stream has not emitted any `S > 0` moves (pure `M3 S0` frame from Rayforge's Frame button when `frame_power_percent: 0`), and always `wait_idle()` before toggling the guide laser. |
| **G6** | **Laser Power Modes (`M3` Constant vs `M4` Dynamic Velocity-Scaled) & `$223` Gate Mode** | Rayforge's `wavemaster-dialect.yaml:13-14` emits `M4 S{power:.0f}` for dynamic cuts and `M3 S{power:.0f}` for constant power (`PowerMode.DYNAMIC` vs `PowerMode.CONSTANT`, `rayforge/pipeline/encoder/gcode.py`). | `emit_linear()` (`src/grbl.c:622`) passes `seg.dynamic = (s_m.laser == 4)`. However, in `src/motion.c:218-266` (`motion_gen_tick`), `seg.dynamic` is **never read (`(void)dynamic`)** because `motion.c` is constant-velocity per segment (no trapezoidal accel/decel ramp—galvo dynamics are handled by jump/mark/poly delays `$212`–`$215`)! | **skip** | Galvo scanners do not ramp velocity down to 0 at corners like stepper stages; instead they trace at constant `F` and dwell for `poly_delay` (`$214`) / `mark_delay` (`$213`) with the gate managed by `laser_on_delay`/`laser_off_delay` (`$210`/`$211`). Differentiating `M3` vs `M4` for `$231` framing (G5 / HIGH-1) is sufficient. |
| **G7** | **Jogging (`$J=G91 G21 X... Y... Z... F...` and `0x85` Jog Cancel)** | `GrblSerialDriver.jog()` (`grbl_serial.py:349-388`) sends `$J=G91 G21 X... F...` and `cancel_jog()` sends `0x85`. | `handle_jog()` (`src/grbl.c:765-799`) supports `$J=` for X, Y, and Z, and `0x85` (`RT_JOG_CANCEL`, `src/host_serial.c:130`, `src/grbl.c:1258-1267`) aborts X/Y motion and calls `zbro_stop()`. **Gap:** During an X/Y jog, `send_status()` (`src/grbl.c:322`) reports `"Run"` instead of `"Jog"`, and `handle_jog()` does not reject `$J=` when the laser is armed/firing or when already in `Hold`/`Alarm`. | **worth-doing** | Track `s_jogging` in `src/grbl.c` so `send_status()` reports `"Jog"` while a `$J=` move is in flight (which Rayforge's `parse_state` expects for `DeviceStatus.JOG` and `0x85` jog-cancel eligibility). |
| **G8** | **Z-Axis Probe (`G38.2` / `G38.3`, `[PRB:x,y,z:1]`, `$HZ` Autofocus)** | `GrblProbeRunner` (`rayforge/machine/driver/grbl/grbl_probe.py:1-238`) runs `G38.2` probing and parses `[PRB:x,y,z:1]`. | Not implemented in `src/grbl.c` (`G38.x` returns `error:20`). Z-Bro C3 has a hardware focal sensor (`IO3`) used by `HOME` (`$HZ`) and `FOCUS` (`$ZF`). | **skip** | WaveMaster's Z-Bro companion uses a dedicated optical/inductive focus sensor on the C3 with `$HZ` (Home Z to focus) and `$ZF` (`G0 Z0`), exposed as macros in `wavemaster-machine.yaml:44-89`. Rayforge's XY/Z touch-plate `G38.2` probing is designed for CNC routers and not applicable to a galvo scanner. |
| **G9** | **Air Assist (`M7` / `M8` / `M9`, `A` in `Ov:` / `A:` status field)** | `wavemaster-dialect.yaml:16-17` maps `air_assist_on: M8` and `air_assist_off: M9`. `grbl_util.py:283` parses `A:` accessory state in `?` reports. | `src/grbl.c:660` accepts `M7`, `M8`, `M9` as silent no-ops (`break;`). Neither the ESP32 nor the ATmega (`ArduinoCompanion/src/main.cpp`) drives an air-assist relay pin. | **skip** | Kept as no-ops in `src/grbl.c:660` so Rayforge G-code with `M8`/`M9` executes without error; no hardware relay pin exists on the current board revision. |
| **G10** | **Realtime Overrides (Feed `0x90`–`0x94`, Rapid `0x95`–`0x97`, Spindle/Power `0x99`–`0x9D`)** | `rayforge/machine/driver/grbl/grbl_util.py` defines status parsing for `Ov:100,100,100`, though Rayforge's current UI does not require live override bytes. | `is_realtime_byte()` (`src/host_serial.c:127-134`) only intercepts `?`, `!`, `~`, `0x18` (`Ctrl-X`), and `0x85` (Jog Cancel). Bytes `>= 0x80` arrive inside G-code lines and cause `error:1` (`E_EXPECTED_LETTER`) in `gcode_parse_line()` (`src/gcode_parse.c:160`)! | **worth-doing** | Strip/ignore standard GRBL extended realtime override bytes (`0x84`, `0x87`, `0x90`–`0x9F`, `0xA0`–`0xA1`) in `is_realtime_byte()` (`src/host_serial.c:127-134`) so any host sending override bytes does not corrupt an in-flight G-code line with `error:1`. |
| **G11** | **Line Number / Planner Buffer Status (`Bf:15,128`, `Ln:`)** | `grbl_util.py:249-256` parses `Bf:` and `Ln:`. `GrblSerialDriver` uses character-counting flow control (`GRBL_RX_BUFFER_SIZE = 128` in `grbl_serial.py:55`, `grbl_network.py:67`). | `send_status()` (`src/grbl.c:350-369`) does not emit `Bf:`. `host_serial.c:30` has a 2048-byte `RX_RING_SIZE`, which easily absorbs Rayforge's 128-byte character-counting window. | **worth-doing** | Emit `|Bf:<free_segs>,<free_rx>` in `send_status()` (`src/grbl.c:350-369`) when `$10` (`CAL_STATUS_MASK`) bit 1 (`value & 2`) is set, matching standard GRBL 1.1 behavior (`$10=1` default stays unchanged). |
| **G12** | **Safety Door / Interlock (`Door:0`..`3`, `Pn:D`)** | `grbl_util.py:224-246` maps `Door:0`..`3` to `DeviceStatus.DOOR`. | `send_status()` (`src/grbl.c:331-346`) reports `Pn:E` when `ATMEGA_ST_EMERGENCY` is active, `Pn:P` when 5V is missing, and `Pn:Z` when Z-Bro is faulted, and reports `Alarm` when `s_alarm != 0`. | **worth-doing** | Once HIGH-3 is applied (setting `s_alarm` on `ATMEGA_ST_EMERGENCY` / 5V loss), `send_status()` will immediately transition from `Run`/`Idle` to `Alarm` with `Pn:EP`, which Rayforge surfaces directly in its status bar and job monitor. |
| **G13** | **Controller Fingerprinting & mDNS Discovery (`$I`, `[VER:1.1f...]`, `[OPT:...]`, `_http._tcp`)** | `grbl_fingerprint.py:1-85` sends `$I` and matches `[VER:...]` / `[OPT:...]`. `mdns_channel.py:1-258` discovers network controllers via mDNS. `GrblNetworkDriver` (`grbl_network.py:100-150`) probes HTTP `/command` first and falls back to raw TCP/Telnet (`ws_port` / port 23) when HTTP fails (`wavemaster-machine-wifi.yaml:12-16`). | `$I` (`src/grbl.c:752-755`) responds `[VER:1.1f.20260412:WaveMaster]` and `[OPT:VN,32,2048]`, and `net_wifi.c:96-111` advertises `wavemaster.local` over mDNS with `_telnet._tcp` port 23. | **skip** | Already matches `docs/rayforge/wavemaster-machine.yaml` and `wavemaster-machine-wifi.yaml`. |

---

## 3. Ordered Work Packages

Work packages are ordered by criticality and structured with **disjoint file sets** wherever possible so multiple implementers can execute in parallel without merge conflicts. Every work package includes an observable acceptance check that runs **without hardware**.

### Parallel Execution Dependency Graph

```
  [WP-1: AD3552R Driver & Board Recovery]   [WP-2: Debug/WDT ISR & sdkconfig]   [WP-3: ATmega Link & Host Serial/Log]
   (components/ad3552r/ad3552r.c,            (src/debug_tools.c,                 (src/atmega_link.*, src/host_serial.*,
    src/ad3552r_board.*)                      sdkconfig.defaults,                 src/net_server.c,
                                              sdkconfig.debug)                    components/espfleet/src/EspFleet.cpp)
                 \                                   |                                   /
                  v                                  v                                  v
            [WP-4: DAC Task ISR Sync, Barrier Close & Fault Safety]    [WP-5: Z-Bro & WiFi Reliability]
             (src/dac_task.*)                                           (src/zbro_link.c, src/net_wifi.c)
                                  \                                    /
                                   v                                  v
             [WP-6: Motion Pipeline Underrun Fix & Abort Sync]   [WP-7: GRBL Safety, $231, $HX/$HY & WCS]
              (src/motion_task.*, src/galvo_out.*)                (src/grbl.c, src/calib.c, src/gcode_parse.c)
```

- **Wave 1 (100% disjoint files — up to 4 implementers in parallel):** **WP-1**, **WP-2**, **WP-3**, **WP-5**.
- **Wave 2 (100% disjoint files — up to 3 implementers in parallel after Wave 1):** **WP-4**, **WP-6**, **WP-7**.

---

### WP-1: AD3552R 24-Bit Stack Overflow Fix, Readback Fix & Hardware Reset Recovery
- **Covers Findings:** CRITICAL-1, CRITICAL-4 (board layer).
- **Files May Edit (Disjoint):**
  - `components/ad3552r/ad3552r.c`
  - `src/ad3552r_board.h`
  - `src/ad3552r_board.c`
- **Dependencies:** None (Wave 1).
- **Risk:** Low-Medium (touches vendor driver buffer sizing and board recovery helper).
- **Changes Required:**
  1. In `components/ad3552r/ad3552r.c:578` (`ad3552r_write_reg`) and `components/ad3552r/ad3552r.c:618` (`ad3552r_read_reg`), change `uint8_t buf[AD3552R_MAX_REG_SIZE] = { 0 };` to `uint8_t buf[AD3552R_MAX_REG_SIZE + 1] = { 0 };`.
  2. In `components/ad3552r/ad3552r.c:537-538` (`ad3552r_single_transfer`), replace `if (data->is_read) data->data[0] = data->data[1];` with `if (data->is_read) memmove(data->data, &data->data[1], data->len);`.
  3. In `src/ad3552r_board.c:236-242` (`s_guard_regs`), add `{ AD3552R_REG_ADDR_TRANSFER_REGISTER, 0x00, 0xC0 }` (or verify `0x0F` is `0x00` in single-lane mode so `MULTI_IO_MODE` is not stuck in Quad SPI).
  4. Implement `int32_t ad3552r_board_recover(void)` in `src/ad3552r_board.c` (declared in `src/ad3552r_board.h`):
     - Drive `BOARD_PIN_SPI_OP1` (GPIO 7) low (`s_qspi_mode = false`).
     - Pulse `BOARD_PIN_RST` (GPIO 9) low for 1 ms, then high, wait 5 ms (`vTaskDelay(pdMS_TO_TICKS(5))`).
     - Re-apply all registers configured in `ad3552r_board_init()` (`0x00`, `0x14`..`0x1A`) via `ad3552r_write_reg()`, re-capture `guard_capture()`, and restore `s_last_code[0]` and `s_last_code[1]`.
  5. In `guard_check_and_repair()` (`src/ad3552r_board.c:260-281`) and `ad3552r_board_stream_open()` (`src/ad3552r_board.c:344-359`), if any register readback returns `0xff` or repair fails, call `ad3552r_board_recover()` and retry once before returning `-EIO` (`-5`).
- **Observable Acceptance Check (No Hardware):**
  1. `pio run` and `pio run -e esp32-s3-devkitc-1-debug` both succeed with 0 warnings in `src/`.
  2. Static check: `grep -n "AD3552R_MAX_REG_SIZE + 1" components/ad3552r/ad3552r.c` shows both `ad3552r_write_reg` and `ad3552r_read_reg` updated, and `grep -n "ad3552r_board_recover" src/ad3552r_board.c` confirms hardware reset recovery on `0xff` / stream-open failure.

---

### WP-2: TWDT ISR Double-Fault Fix, Core-0 `sample_cb` Fix & `sdkconfig` Hardening
- **Covers Findings:** CRITICAL-2 (`sdkconfig.defaults` GPIO IRAM), CRITICAL-3, HIGH-4 (`sdkconfig.defaults` WiFi/lwIP core pinning).
- **Files May Edit (Disjoint):**
  - `src/debug_tools.c`
  - `sdkconfig.defaults`
  - `sdkconfig.debug`
- **Dependencies:** None (Wave 1).
- **Risk:** Low (configuration & debug instrumentation only).
- **Changes Required:**
  1. In `src/debug_tools.c:38-63` (`esp_task_wdt_isr_user_handler`), call `laser_io_gate_kill()` first (`#include "laser_io.h"`), copy `pcTaskGetName(xTaskGetCurrentTaskHandleForCore(0))` and `pcTaskGetName(xTaskGetCurrentTaskHandleForCore(1))` into `s_rtc.wdt_starved` using an inline `IRAM_ATTR` byte loop, and **remove** all calls to `snprintf()`, `strlcat()`, and `esp_task_wdt_print_triggered_tasks()`.
  2. In `src/debug_tools.c:99-118` (`sample_cb`), remove the `for (int i = 0; i < s_watched_n; i++) uxTaskGetStackHighWaterMark(h)` loop from the 100 ms Core 0 `esp_timer` callback (only update `min_free_internal` in `sample_cb`; sample stack watermarks at most when `!dac_task_is_streaming()` or on `$DBG`).
  3. In `sdkconfig.defaults`, add:
     - `CONFIG_GPIO_CTRL_FUNC_IN_IRAM=y`
     - `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1=y`
     - `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU1=y`
     - `CONFIG_ESP_TASK_WDT_PANIC=y`
     - `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y`
     - `CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF=y`
     - `CONFIG_ESP_COREDUMP_CHECKSUM_CRC32=y`
  4. In `sdkconfig.debug:20-22`, replace `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y` with `CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT=y`.
- **Observable Acceptance Check (No Hardware):**
  1. Delete `sdkconfig.esp32-s3-devkitc-1-debug` and `sdkconfig.esp32-s3-devkitc-1` (if present so ESP-IDF regenerates them from `sdkconfig.defaults` + `sdkconfig.debug`, per `AGENTS.md` §3.1) and run `pio run` and `pio run -e esp32-s3-devkitc-1-debug` with 0 warnings.
  2. Static check: verify `esp_task_wdt_print_triggered_tasks`, `snprintf`, and `strlcat` do not appear inside `esp_task_wdt_isr_user_handler` in `src/debug_tools.c`, and `CONFIG_GPIO_CTRL_FUNC_IN_IRAM=y` and `CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT=y` are active in the generated `sdkconfig.esp32-s3-devkitc-1-debug`.

---

### WP-3: ATmega Fault Safety, Core-1 Task Priorities, Non-Blocking Core-0 Logging & Telnet Keepalive
- **Covers Findings:** HIGH-3 (ATmega fault gate kill & spinlock), HIGH-4 (`atmega`, `host_rx`, `net_srv`, `net_tx`, `espfleet` priorities/pinning), HIGH-5 (`atmega_link` TWDT), MEDIUM-1 (non-blocking Core-0 log & `espfleet` log recursion guard), MEDIUM-3 (`s_status` spinlock), MEDIUM-5 (TCP keepalive), G10 (ignore GRBL realtime override bytes).
- **Files May Edit (Disjoint):**
  - `src/atmega_link.h`
  - `src/atmega_link.c`
  - `src/host_serial.c`
  - `src/net_server.c`
  - `components/espfleet/src/EspFleet.cpp`
  - `test/host/hostlink/test_hostlink.c`
- **Dependencies:** None (Wave 1).
- **Risk:** Low-Medium.
- **Changes Required:**
  1. In `src/atmega_link.c:421-443`, whenever `(flags & ATMEGA_ST_EMERGENCY)` is set, `!(flags & ATMEGA_ST_5V_PRESENT)`, or `s_consecutive_fail >= 5`, immediately call `laser_io_gate_kill()` (`#include "laser_io.h"`). Expose `bool atmega_link_has_fault(void)` in `src/atmega_link.h` (returns `true` when `s_consecutive_fail >= 5` or `ATMEGA_ST_EMERGENCY` is active).
  2. Protect `s_status` reads/writes in `src/atmega_link.c:285-291,411-418` with a `static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;`.
  3. In `src/atmega_link.c:389,450,465`, raise `atmega` task priority from `5` to **`10`**, subscribe `atmega_link_task` to TWDT via `esp_task_wdt_add(NULL)` at task entry, and call `esp_task_wdt_reset()` each loop iteration.
  4. In `src/host_serial.c:127-134` (`is_realtime_byte`), also recognize standard GRBL extended realtime override/toggle bytes (`c >= 0x84`) and discard unrecognized `>= 0x84` bytes in `host_ingest_bytes()` (`src/host_serial.c:152`) so they never enter `s_rx_ring` and corrupt G-code lines with `error:1`.
  5. In `src/host_serial.c:186` (`host_log_vprintf`), if `xPortGetCoreID() == 0` (Core 0 / `dac_task`), use `xSemaphoreTake(s_tx_mutex, 0)` instead of `portMAX_DELAY` (if contended, push to `net_server_tx_bytes` only and return) so `dac_task` never blocks on a Core-1 task. Also skip `espfleet_log_push()` when `pcTaskGetName(NULL)` is `"espfleet"` to break the `espfleet` -> `ESP_LOG` -> `host_log_vprintf` -> `espfleet_log_push` recursion loop.
  6. In `src/host_serial.c:428`, set `host_rx` priority to **`8`**. In `src/net_server.c:292-293`, set `net_srv` priority to **`8`** and `net_tx` priority to **`5`**.
  7. In `src/net_server.c:177-180`, enable `SO_KEEPALIVE` on accepted client sockets (`keepalive = 1`, `TCP_KEEPIDLE = 10`, `TCP_KEEPINTVL = 3`, `TCP_KEEPCNT = 3`).
  8. In `components/espfleet/src/EspFleet.cpp:277`, change `xTaskCreate(taskEntry, "espfleet", 6144, this, 3, &task_)` to `xTaskCreatePinnedToCore(taskEntry, "espfleet", 6144, this, 3, &task_, 1)`.
  9. Add a unit test in `test/host/hostlink/test_hostlink.c` verifying that GRBL extended realtime bytes (`0x85`, `0x90`–`0x9F`, `0xA0`) mixed into a G-code line are stripped cleanly without corrupting the line.
- **Observable Acceptance Check (No Hardware):**
  1. `make -C test/host/hostlink` passes with 0 failures (including the new extended realtime byte test).
  2. `pio run` succeeds with 0 warnings in `src/`.

---

### WP-4: `dac_task` ISR Synchronization, Barrier Stream-Close, Safe Arm Ordering & Underrun Reap Fix
- **Covers Findings:** CRITICAL-2 (`dac_task` ISR race & priority), CRITICAL-4 (`dac_task` barrier close, arm-after-open, sticky fault), HIGH-2 (`reap()` order in `dac_task`), HIGH-3 (`do_arm`/`apply_power` retry on failure), HIGH-5 (`dac_task` TWDT).
- **Files May Edit (Disjoint):**
  - `src/dac_task.h`
  - `src/dac_task.c`
- **Dependencies:** WP-1 (`ad3552r_board_recover` declaration in `src/ad3552r_board.h`).
- **Risk:** Medium-High (core real-time streaming task on Core 0; must preserve all IRAM and zero-delay invariants).
- **Changes Required:**
  1. **Fix Core 0 ISR preemption & race (CRITICAL-2):**
     - In `src/dac_task.c:554`, change `s_timer` `.intr_priority = 3` to `.intr_priority = 1` (same level as `SPI2_HOST` ISR so `on_alarm`, `stream_pre`, and `stream_post` cannot preempt each other on Core 0), and guard `stream_pre()`, `stream_post()`, and `on_alarm()` with `static portMUX_TYPE s_isr_mux = portMUX_INITIALIZER_UNLOCKED;` (`portENTER_CRITICAL_ISR(&s_isr_mux)` / `portEXIT_CRITICAL_ISR(&s_isr_mux)`).
     - In `isr_arm_next()` (`src/dac_task.c:113-115`), set `s_alarm_armed = true;` **before** `gptimer_set_alarm_action(s_timer, &s_alarm_cfg);`.
     - In `on_alarm()` (`src/dac_task.c:126-130`), remove the `if (timer_now() < s_next_edge_time) { isr_arm_next(); return false; }` early-return loop.
  2. **Close stream before ATmega barriers & open stream before arming (CRITICAL-4):**
     - In `service()` (`src/dac_task.c:390-393`), when `s_next->barrier` is processed after `s_outstanding == 0`, call `session_close()` **before** `apply_barrier(s_next)` so `CS` is high and `QSPI` is low while waiting on the ATmega UART round-trip.
     - In `try_start()` (`src/dac_task.c:396-410`), open the SPI stream (`session_open(s_prr_hz)`) **before** calling `do_arm(true)`! If `session_open()` fails even after `ad3552r_board_recover()` (from WP-1), immediately call `laser_io_gate_kill()`, `do_arm(false)`, `apply_power(0)`, set `s_dac_fault = true` (exposed via `bool dac_task_has_fault(void)` and cleared on `dac_task_clear_fault(void)` in `src/dac_task.h`), and log `ESP_LOGE` only on the transition into `s_dac_fault` (not on every chunk!).
  3. **Retry failed disarm / power-zero (HIGH-3):**
     - In `do_arm()` (`src/dac_task.c:356-370`) and `apply_power()` (`src/dac_task.c:322-330`), only update `s_armed = arm` / `s_power_applied = p` when `atmega_link_set_arm(arm)` / `atmega_link_set_power(p)` returns `true`. If `do_arm(false)` fails, call `laser_io_gate_kill()` and leave `s_armed = true` so `service()` (`src/dac_task.c:525`) retries on the next 20 ms tick.
  4. **Fix false `stream underrun with the laser on` in `service()` (HIGH-2):**
     - In `service()` (`src/dac_task.c:476-488`), dequeue `s_next` (`if (!s_next) xQueueReceive(g_galvo.ready_q, &s_next, 0);`) **before** calling `reap()`, and in `reap()` (`src/dac_task.c:240`), suppress the warning when `s_next != NULL`.
  5. **Subscribe `dac_task` to TWDT (HIGH-5):**
     - Call `esp_task_wdt_add(NULL)` at the start of `dac_task_fn()` (`src/dac_task.c:570`) and `esp_task_wdt_reset()` once per loop iteration (`src/dac_task.c:599`).
- **Observable Acceptance Check (No Hardware):**
  1. `pio run` and `pio run -e esp32-s3-devkitc-1-debug` succeed with 0 warnings in `src/`.
  2. Static check: verify `.intr_priority = 1` in `src/dac_task.c`, `s_alarm_armed = true` precedes `gptimer_set_alarm_action`, `session_open` precedes `do_arm(true)` in `try_start()`, and `session_close()` precedes `apply_barrier()`.

---

### WP-5: Z-Bro TCP Link & WiFi Power-Save Hardening
- **Covers Findings:** CRITICAL-5, MEDIUM-6.
- **Files May Edit (Disjoint):**
  - `src/net_wifi.c`
  - `src/zbro_link.c`
- **Dependencies:** None (Wave 1).
- **Risk:** Low.
- **Changes Required:**
  1. In `src/net_wifi.c:207` (`start_ap`) and `src/net_wifi.c:242` (`start_sta`), call `esp_wifi_set_ps(WIFI_PS_NONE);` immediately after `esp_wifi_start()` so the S3 WiFi radio never enters modem sleep during TCP/Z-Bro traffic.
  2. In `src/zbro_link.c:315`, move `autohome_tried` to file scope (`static bool s_autohome_tried = false;`) so a dropped TCP connection mid-autohome does not re-trigger `HOME` in an infinite loop on reconnect. When `session()` exits while `s_auto == ZBRO_AUTO_HOMING` (`src/zbro_link.c:385-391`), set `s_auto = ZBRO_AUTO_FAILED`.
  3. In `src/zbro_link.c:267` (`send_line`), pass `MSG_DONTWAIT` to `send(fd, p, left, MSG_DONTWAIT)` so the existing `EAGAIN`/`EWOULDBLOCK` bounded wait loop (`src/zbro_link.c:271-276`) functions instead of blocking forever.
  4. In `src/zbro_link.c:347-356`, allow `PING` to be sent every `PING_MS` (`1000` ms) even when `exp == EXP_DISCARD` or `exp == EXP_POS_DISCARD` (keeping `t_sent` anchored to the first unanswered request while still feeding the C3's 3 s `PING_TIMEOUT_MS`), and increase `REPLY_MS` (`src/zbro_link.c:32`) from `5000` to `8000` ms.
  5. In `src/zbro_link.c:181-200` (`resolve_host`), cache the last resolved `struct in_addr s_cached_addr` and reuse it on reconnect unless `tcp_connect()` to the cached IP fails, avoiding a 2 s blocking `mdns_query_a()` stall on every transient reconnect.
- **Observable Acceptance Check (No Hardware):**
  1. `make -C test/host/zbro` passes with 0 failures.
  2. `pio run` succeeds with 0 warnings in `src/`.
  3. Static check: `grep -n "WIFI_PS_NONE" src/net_wifi.c` and `grep -n "MSG_DONTWAIT" src/zbro_link.c` both match.

---

### WP-6: Motion Pipeline Underrun Fix, Abort Synchronization & Priority Fix
- **Covers Findings:** HIGH-2 (`motion_task.c` premature flush & `galvo_out.c` barrier gate-off), HIGH-4 (`motion` priority 9), MEDIUM-2 (`galvo_out_abort` race with `motion_task`), MEDIUM-3 (`s_pos_x_mm`/`s_pos_y_mm` spinlock).
- **Files May Edit (Disjoint):**
  - `src/motion_task.c`
  - `src/galvo_out.c`
- **Dependencies:** WP-3 (Core 1 priority ordering).
- **Risk:** Medium.
- **Changes Required:**
  1. **Fix premature `galvo_out_flush()` in `motion_task_loop()` (`src/motion_task.c:208-227`, HIGH-2):**
     - Do **not** call `galvo_out_flush()` before checking `allowed_us` when `motion_gen_has_pending(&s_gen)` is true (`in_polyline == true`, where flushing mid-polyline emits a partial chunk with `c->gate_end = true`).
     - Compute `buffered_us = galvo_out_buffered_us()` (including `s_cur_idx * 10` us in the current unsubmitted chunk) and `allowed_us = buffered_us - MOTION_UNDERRUN_MARGIN_US`.
     - If `wait_ms < 1`, immediately call `motion_gen_finish(&s_gen)` and `galvo_out_flush()` together so the chunk handed to `dac_task` already includes the polyline close (`laser_off_delay` + `mark_delay`, `gate_end = false`).
  2. **Drain `laser_off_delay` before power/PRR barrier (`src/galvo_out.c:367-381`, HIGH-2):**
     - In `galvo_out_tick()` (`src/galvo_out.c:367-381`), if a power or PRR barrier is required while `s_out_gate` is still `true`, clock `s_delay_off_ticks` ticks of `raw_gate = false` at `(s_last_x, s_last_y)` so `s_out_gate` transitions to `false` (`c->gate_end = false`) before `request_barrier()` submits the pre-barrier chunk.
  3. **Eliminate `galvo_out_abort()` race between `grbl_task` and `motion_task` (`src/motion_task.c:115-133,167,304-312`, `src/galvo_out.c:538-563`, MEDIUM-2 & MEDIUM-3):**
     - Check `if (s_abort_req) break;` inside the inner `while (motion_gen_tick(...))` loop (`src/motion_task.c:167`).
     - In `motion_abort()` (`src/motion_task.c:304-312`), set `s_abort_req = true`, call `laser_io_gate_kill()`, drain `g_galvo.ready_q` into `g_galvo.free_q` (to immediately unblock `motion_task` if it is waiting inside `alloc_chunk()`), and wait up to 20 ms on a completion notification/semaphore from `motion_task`'s `if (s_abort_req)` block (`src/motion_task.c:115-132`), where `motion_task` itself calls `galvo_out_abort()`, resets `s_pos_x_mm = 0.0f; s_pos_y_mm = 0.0f;`, and sets `s_idle = true`.
  4. **Protect `(s_pos_x_mm, s_pos_y_mm)` with a spinlock (MEDIUM-3) and raise `motion` priority to 9 (HIGH-4):**
     - Guard updates/reads of `s_pos_x_mm` and `s_pos_y_mm` with a `portMUX_TYPE` spinlock.
     - In `src/motion_task.c:339`, change `motion` task priority from `8` to **`9`**.
- **Observable Acceptance Check (No Hardware):**
  1. `make -C test/host` passes all 58,808 checks with 0 failures.
  2. `pio run` succeeds with 0 warnings in `src/`.

---

### WP-7: GRBL Safety Alarms, `$231` Auto-Guide Fix, NVS Batching & Rayforge G-Code Enhancements (`$HX`/`$HY`, WCS, Jog/Home Status)
- **Covers Findings:** HIGH-1 (`$231` auto-guide under `M3`), HIGH-3 (`ALARM` disarms laser, checks `dac_task_has_fault()` / `atmega_link_has_fault()`), HIGH-4 (`grbl` priority 7), MEDIUM-4 (`wait_idle` before `calib_set` & single-commit `calib_reset_defaults`), LOW-1 (`isfinite` validation), and Rayforge Gaps G1, G2, G3, G5, G7, G11, G12.
- **Files May Edit (Disjoint):**
  - `src/grbl.c`
  - `src/calib.c`
  - `src/gcode_parse.c`
  - `test/host/gcode/test_gcode.c`
- **Dependencies:** WP-3 (`atmega_link_has_fault`), WP-4 (`dac_task_has_fault`, `dac_task_clear_fault`).
- **Risk:** Low-Medium.
- **Changes Required:**
  1. **Fix `$231` (`CAL_AUTO_GUIDE`) under `M3` (`src/grbl.c:449-454,541-545`, HIGH-1 & G5):**
     - Add `bool m3_burned` to `modal_t s_m` (set `true` whenever `emit_linear()` runs with `s_m.laser != 0 && s_m.s > 0.0f`; reset to `false` on `M5`, `M30`, `M2`, soft reset, and when `motion_is_idle() && !dac_task_is_streaming()` at the start of a new job).
     - In `emit_linear()` (`src/grbl.c:541-545`), only enter `s_auto_guide` when `s_m.laser == 3 && !(s_m.s > 0.0f) && !s_m.m3_burned && !s_m.preview && calib_get(CAL_AUTO_GUIDE) > 0.5f`. When transitioning `s_auto_guide` from `false` to `true`, call `wait_idle()` first before `atmega_link_set_guide(true)`.
  2. **Centralize `enter_alarm(int code)` & propagate DAC / ATmega faults (`src/grbl.c:605,651,1240-1285`, HIGH-3 & G12):**
     - Add `static void enter_alarm(int code)` that sets `s_alarm = code`, calls `laser_io_gate_kill()`, `motion_abort()`, `galvo_out_set_arm(false)`, `galvo_out_set_power(0)`, `atmega_link_set_guide(false)`, and resets `s_m.armed = false`, `s_m.laser = 0`, `s_m.s = 0.0f`.
     - In `grbl_task` (`src/grbl.c:1270`), poll `dac_task_has_fault()` and `atmega_link_has_fault()`; if either is true and `s_alarm == 0`, call `enter_alarm(1)` and emit `ALARM:1\r\n`. Clear `dac_task_clear_fault()` on `$X` / `Ctrl-X`.
  3. **Support `$HX` and `$HY` single-axis homing (`src/grbl.c:962-975`, G2):**
     - Accept `$HX` (move X to 0 mm at current Y, zero WCS X), `$HY` (move Y to 0 mm at current X, zero WCS Y), `$HZ` (existing Z-Bro home), and `$H` (move X and Y to 0 mm).
  4. **Implement real Work Coordinate Offsets (`G54`, `G10 L2`/`L20`, `G92`/`G92.1`, `$#`, `WCO:`) (`src/grbl.c:304-370,568,693-702,756`, G3):**
     - Maintain `s_wco_x`, `s_wco_y`, `s_wco_z` (sum of `G54` + `G92` offsets) separately from machine position `s_m.x`, `s_m.y`, `s_m.z` so `G92 X0 Y0` updates the offset instead of corrupting the galvo's physical voltage coordinates! Report `[G54:...]` and `[G92:...]` accurately in `$#` (`src/grbl.c:756`) and `|WCO:...` in `send_status()`.
  5. **Report accurate `Home`, `Jog`, and `Bf:` states in `send_status()` (`src/grbl.c:315-369`, G1, G7, G11):**
     - Report `"Home"` when Z-Bro status is `ZBRO_ST_HOMING`, `"Jog"` while a `$J=` move is active (`s_jogging` cleared when `motion_is_idle() && !dac_task_is_streaming()`), and append `|Bf:<free_segs>,<free_rx>` when `((int)calib_get(CAL_STATUS_MASK)) & 2` is non-zero.
  6. **NVS safety & `isfinite` validation (`src/calib.c:200,236-255`, `src/gcode_parse.c:167`, `src/grbl.c:812,884,892`, MEDIUM-4 & LOW-1):**
     - Call `wait_idle()` before `calib_set()` (`src/grbl.c:892`) and `calib_reset_defaults()` (`src/grbl.c:812`).
     - In `calib_reset_defaults()` (`src/calib.c:236-255`), open NVS once, write all defaults, commit once, and close once.
     - Reject `!isfinite(val)` in `src/gcode_parse.c:167`, `src/grbl.c:884`, and `src/calib.c:200`.
     - In `src/grbl.c:1353`, set `grbl` task priority to **`7`**.
  7. **Host unit test additions (`test/host/gcode/test_gcode.c`):**
     - Add test cases for `NaN`/`Inf` rejection (`G1 Xnan`, `G1 Xinf`, `G10 L20 P1 X10 Y20`, `G92.1`).
- **Observable Acceptance Check (No Hardware):**
  1. `make -C test/host/gcode` passes (`gcode tests passed`).
  2. `pio run` and `pio run -e esp32-s3-devkitc-1-debug` succeed with 0 warnings in `src/`.

---

## 4. What Can Only Be Verified on Hardware

Per `AGENTS.md` §6 and `docs/hardening/BRIEF.md`, the following behaviors depend on physical electrical signals, the AD3552R DAC, the ATmega328P companion, the Z-Bro ESP32-C3 companion, or the JPT MOPA fiber laser and must be verified on hardware after the work packages above land:

1. **AD3552R 24-Bit Register Readback (`$RB`) Accuracy (WP-1 / CRITICAL-1):**
   - After flashing WP-1, run `$RB` at idle (`0.000 V` -> `0x8000`) and after `$DV X=2.5 Y=-2.5`. Confirm that `$RB` now reads back the exact 16-bit code (`0x8000` instead of the `0x8080` MSB-duplication bug documented in `STATUS.md:45`).
2. **AD3552R Hardware Reset Recovery (`ad3552r_board_recover`) Under Induced Glitch (WP-1 & WP-4 / CRITICAL-4):**
   - Verify on the bench that pulsing `BOARD_PIN_RST` (GPIO 9) restores `0x00=0x81, 0x14=0x03, 0x16=0x01, 0x19=0x44, 0x1A=0x00` and DAC output voltages within <10 ms, and that closing the quad-SPI stream across ATmega power/PRR barriers eliminates the `0x19: 0x44 -> 0x00 -> 0xff` register loss during multi-power jobs.
3. **Core 0 Gate ISR Jitter & Absence of `int_wdt` Under Pulse Density Mode (`$223=1`) (WP-2 & WP-4 / CRITICAL-2 & CRITICAL-3):**
   - Run a 30-minute continuous multi-layer raster + vector job in both `$223=0` (PRR Duty) and `$223=1` (Pulse Density) modes with WiFi/Telnet and `espfleet` active.
   - Verify with `$DBG` that `resets` does not increment, `min_free_internal` stays stable, and scope-check DB25 pin 19 (GPIO 4 gate) vs galvo X/Y differential outputs to confirm moving `s_timer` to priority 1 with `s_isr_mux` adds no visible gate timing jitter (<1 tick = 10 us).
4. **TWDT Coredump Capture Verification (WP-2 / CRITICAL-3):**
   - Deliberately trigger a test watchdog timeout (or panic test hook in the debug build) once on the bench and confirm the ESP32-S3 reboots silently within ~5 s and `$DBG` reports `coredump: present` with a valid backtrace and `wdt_starved` task names.
5. **Laser Gate-Off Guarantee on Faults & Resets (WP-3, WP-4, WP-6, WP-7 / HIGH-3):**
   - Probe DB25 pin 19 (gate, GPIO 4) and pin 18 (MO/arm, ATmega D6) with an oscilloscope during:
     - Soft reset (`Ctrl-X`) mid-mark
     - Simulated ATmega UART disconnect (unplugging UART0 RX/TX) mid-mark
     - Soft limit (`ALARM:2`) trigger
   - Confirm pin 19 drops low within <10 us and pin 18 drops low within 1 UART round-trip (or 1 s ATmega watchdog on wire pull).
6. **Z-Bro Full 60 s Homing Cycle & Long-Run Link Stability Over WiFi (WP-5 / CRITICAL-5):**
   - Run `$HZ` from the top of the Z column (full >15 s travel to the focal sensor) 10 times consecutively and keep the link idle/polling for >30 minutes. Confirm zero `the C3 closed the link` mid-home drops and zero `waiting for reply type 2/3: 5 s` timeouts.

---

## 5. Open Questions for the Owner (Ranked, Max 5)

1. **`$231` (`CAL_AUTO_GUIDE`) Default Value (`1` vs `0`):**
   - With WP-7 fixing `$231` so it only turns on the guide laser when an `M3` stream has emitted zero `S > 0` moves (pure `M3 S0` framing from Rayforge's Frame button), do you want `$231` to remain enabled by default (`1.0`), or should we change the factory default in `src/calib.c:58` to `0.0` so users opt in explicitly (or use `M66`/`M67`)?
2. **Updating the CompanionZ (ESP32-C3) Firmware Alongside S3 (WP-5):**
   - WP-5 hardens the S3 side (`src/zbro_link.c` and `src/net_wifi.c`) so the S3 disables WiFi power-save, keeps sending `PING` every 1 s during pending replies, and does not loop `HOME` on reconnect. May a follow-up task also touch `CompanionZ/main/zb_net.c` (to add `esp_wifi_set_ps(WIFI_PS_NONE)` on the C3 and drop unsolicited 10 Hz `POS` frames on `EAGAIN` instead of calling `shutdown(s_cfd)` which aborts homing)?
3. **Behavior of `$H` (All-Axis Home) When Z-Bro Is Enabled (`$300=1`):**
   - Currently `$H` (`src/grbl.c:971`) only zeroes the galvo X/Y mirrors and never homes Z, whereas `$HZ` homes the Z-Bro column. When `$300=1`, should `$H` continue to only home X/Y (leaving Z homing strictly to `$HZ`), or should `$H` also home Z?
4. **Work Coordinate System (`supports_wcs`) in `docs/rayforge/wavemaster-machine.yaml`:**
   - Currently `docs/rayforge/wavemaster-machine.yaml:22` sets `supports_wcs: false` while `src/grbl.c` partially stubs `G54` and `G92`. Once WP-7 implements proper `G54`/`G92` offsets in `src/grbl.c`, should `wavemaster-machine.yaml` keep `supports_wcs: false` (simple center-origin galvo workflow) or enable `supports_wcs: true`?
5. **`espfleet` Telemetry in Production Builds:**
   - `espfleet` (`components/espfleet/src/EspFleet.cpp`) runs an unpinned background task doing periodic mDNS queries, HTTP heartbeats, and HTTP log shipping every 5 s. After WP-3 pins it to Core 1 at priority 3 and stops its log feedback loop, should `espfleet` remain unconditionally active whenever WiFi STA connects, or should it be gated behind a `$` setting / build flag?
