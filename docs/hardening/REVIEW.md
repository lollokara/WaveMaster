# Review of docs/hardening/PLAN.md

Reviewer pass on branch `hardening/audit-and-features` (HEAD 49f72b5). No source edited, no hardware touched, PLAN.md untouched.
Line numbers below are the ones I opened in the current tree (`file:line`).

## 0. Headline

**Overall: NO-GO as written. Re-plan a much smaller set.**

PLAN.md was clearly not written against the tree it reviews. About half of the critical/high findings and most of the Rayforge gap list describe code that does not exist here, or things that are already done:

- It cites symbols that are not in the source: `s_armed`, `s_prr_hz`, `do_arm(true)`, `session_open(s_prr_hz)`, `dac_task_has_fault`, `ATMEGA_ST_EMERGENCY`, `s_consecutive_fail`, `s_status`, `s_pos_x_mm`, `IDLE_POLL_MS` in `dac_task.c`, `autohome_tried`, `SEND_WAIT_MS`, `BOARD_PIN_RST` use in recovery. The real `try_start()` is `src/dac_task.c:382`, not 396; `session_open()` and `do_arm()` take no arguments (`dac_task.c:250,355`).
- It proposes settings that are already in `sdkconfig.defaults`. WiFi/lwIP core-1 pinning is at the `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1`/`LWIP_TCPIP_TASK_AFFINITY_CPU1` lines. `GPTIMER_*_IN_IRAM` and `SPI_MASTER_ISR_IN_IRAM` are there too. The coredump-to-flash and `ESP_TASK_WDT_PANIC` settings are already in `sdkconfig.debug:12-21`.
- It proposes features that are already implemented. WCO/G92/G54 offsets are in `grbl.c:120-125,659,1121-1128`. `Jog` state is at `grbl.c:1116`. `Bf:` is at `grbl.c:1134`. `$HX`/`$HY` are accepted at `grbl.c:999-1001`. Bytes >= 0x80 are dropped at `host_serial.c:293`. NaN is rejected at `calib.c:172`.

**Build confirmation (clean-branch check, before any change):** both `pio run -e esp32-s3-devkitc-1-debug` (SUCCESS, was incremental, 0 warnings) and `-e esp32-s3-devkitc-1` (SUCCESS, 27 s) build. The release log has 12 warnings, all in `components/ad3552r/ad3552r.c` (unused functions and variables of the vendor driver), none in `src/`. So "0 warnings in `src/`" is a runnable acceptance check. "0 warnings" overall is not, and WP-1 edits that file.

## 1. Verdicts on critical and high findings

Legend: CONFIRMED = I read the code and the claim holds. WRONG = the code contradicts it. UNVERIFIABLE = plausible, cannot be settled from source or the brief.

### CRITICAL-1: 24-bit register transfer overflows `buf[3]` — **CONFIRMED (real UB), severity overstated**
- `ad3552r.c:78` `AD3552R_MAX_REG_SIZE 3`. `ad3552r.c:225-230` `ad3552r_convert_to_single_msg(data, len)` runs `for (i = len; i; i--) data[i] = data[i-1]`, so with `len = 3` it writes `data[3]`. `ad3552r_reg_len()` returns 3 for `addr > AD3552R_REG_ADDR_HW_LDAC_24B` (0x37). `ad3552r.h:140`: `CH_DAC_24B(ch) = 0x3D - (1-ch)*3`, i.e. 0x3A/0x3D. The plan says 0x4B/0x4E, which is wrong. `ad3552r.c:~565-600` declares `uint8_t buf[AD3552R_MAX_REG_SIZE]` in both `write_reg` and `read_reg`. The 4-byte transfer is `msgs[0].bytes_number = len + 1` (`ad3552r.c:~531`). So one byte past the array is written on every DAC code write/readback (`ad3552r_board.c:194`, `dac_task.c:525,614`).
- Not proven: that it corrupts anything. Xtensa pads a 3-byte local, so the extra byte very likely lands in padding. It cannot cause Crash A (that is a tick stall with an idle core). Cheap and correct to fix, so do it.
- The plan's second claim, "`$RB` reads 0x8080 for 0x8000", cites `STATUS.md:45`. That line is about WiFi, and `grep 8080 STATUS.md` finds nothing. **UNVERIFIABLE** as stated. The code reading (`data[0]=data[1]` at `ad3552r.c:~537`) does look like it drops the LSB on 16-bit-and-longer reads, but where the shim puts the RX bytes depends on `do_transaction()`, which I did not trace. The boot self-test (`dac_task.c:521-533`) compares written vs read codes and STATUS.md says it reads `self-test OK` on hardware, which argues the readback works today. Treat the readback change as needing a `$RB` bench check, not a free fix.

### CRITICAL-2: level-3 GPTimer ISR vs SPI ISR, "root cause of Crash A" — **PARTLY CONFIRMED as a race, WRONG as the Crash A cause**
- CONFIRMED real race: `dac_task.c:114-115` calls `gptimer_set_alarm_action()` and only then sets `s_alarm_armed = true`. A critical section (`portENTER_CRITICAL` in gptimer) masks the level-3 ISR on the S3, so the alarm cannot fire inside the call. It fires the moment the section exits, before line 115 runs. `on_alarm` then sees `!s_alarm_armed` (`dac_task.c:124`) and returns "stale". Line 115 then arms an alarm that has already fired, so every later edge in that chunk is lost. Effect: the gate stays in its previous state until `stream_post()` (`dac_task.c:161-175`). Max one chunk (20 ms). A gate stuck on for up to 20 ms is a laser-safety bug in itself. The similar window in `isr_disarm()` (`dac_task.c:81-84`) is also real but benign.
- UNVERIFIABLE: "SPI ISR is level 1". `no_os_spi_esp32.c:94` passes `ESP_INTR_FLAG_IRAM` with no level bits. The allocator picks a level from 1..3 and I cannot see which. Check at runtime.
- WRONG: "ISR storm / loop starves the level-1 tick, so int_wdt". `on_alarm` (`dac_task.c:121-136`) cannot loop. `isr_arm_next`'s while loop is bounded by `n_edges` (<=256). 256 edges per 20 ms is about 13k ISRs/s of a few us each, a small fraction of one core. The brief's core dump shows IDLE0 asleep in `esp_cpu_wait_for_intr` with no lock held. That means core 0 was not busy in an ISR when the dump was taken. The tick interrupt was missing, which is the opposite of a storm.
- WRONG: "`gpio_set_level` in flash called from the IRAM ISR". `laser_io.c:57` (`gate_write_level`) uses inline `gpio_ll_set_level(&GPIO, ...)`. `CONFIG_GPIO_CTRL_FUNC_IN_IRAM` being unset (`sdkconfig.esp32-s3-devkitc-1:826`) does not matter on this path. Adding it is harmless but costs IRAM and is not a fix.
- The proposed fix has a design problem. Dropping the timer to level 1 removes the reason for level 3 (`dac_task.c:554` comment: gate edges must not wait behind the SPI ISR) and puts gate edges behind the tick and any other level-1 ISR. Better: keep level 3, set `s_alarm_armed = true` before the `gptimer_set_alarm_action()` call (the `timer_now() < s_next_edge_time` check at `dac_task.c:126` already handles an early wake), and wrap `stream_pre`/`stream_post` state changes in a short critical section. Deleting the re-arm branch (`dac_task.c:126-130`) is not needed and removes a safety net.

### CRITICAL-3: `esp_task_wdt_isr_user_handler` takes mutexes in ISR — **WRONG**
- The real code (`debug_tools.c:38-50`) calls `esp_task_wdt_print_triggered_tasks(twdt_collect, NULL, &cpus)`. The plan says `(NULL, NULL, NULL)` and that it logs through `host_log_vprintf`. With a callback supplied it does not go through the log path. The handler only does `strlcat`/`snprintf` on a RTC buffer and `pcTaskGetName`, which is fine in an ISR on the S3 (ROM libc). No mutex is taken.
- WRONG: "`sample_cb` walks 9 task stacks every 100 ms". `debug_tools.c:100-108` only reads `xTaskGetCurrentTaskHandleForCore(1)` and copies the name. `uxTaskGetStackHighWaterMark` appears once, at `debug_tools.c:174`, in the boot-time report.
- WRONG: "`PANIC_PRINT_REBOOT` contradicts the comment". `sdkconfig.debug:19-21` leaves SILENT_REBOOT commented out on purpose, with a stated reason (a held-open port can outlast the 10 s RTC watchdog while printing). It is a trade-off, not a bug. Making it silent would remove the panic log the hub shows. See the "removes logging" rule in the brief.
- WRONG: "no coredump because of double fault". Nothing shows that. `sdkconfig.debug` already enables coredump-to-flash and `TASK_WDT_PANIC`. Crash A is an `int_wdt` and cannot go through the TWDT hook anyway.

### CRITICAL-4: DAC 0xff wedge, no hardware RESET, laser left armed — **MIXED**
- CONFIRMED: `BOARD_PIN_RST` (GPIO 9) is defined (`ad3552r_board.h:14`) and only handed to `ad3552r_init` once (`ad3552r_board.c:25-27`). Nothing resets the chip at runtime. `guard_check_and_repair()` (`ad3552r_board.c:~256-280`) only rewrites single-lane registers. If the chip is truly stuck, repair does nothing and `stream_open` returns -EIO (`ad3552r_board.c:~353-358`, the "stream setup did not stick" branch). So "no hardware reset recovery" is real.
- CONFIRMED: `try_start()` runs `do_arm()` and `apply_power()` (`dac_task.c:396-406`) before `session_open()` (`dac_task.c:407`). On failure the chunk is dropped with the ATmega armed and the power word latched. Overstated: the gate is only driven by the stream ISRs, and `dac_task.c:572` plus `session_close`/`do_abort` leave it low, so nothing is emitted. Still, disarm and zero power on a failed open is a cheap, good fail-safe.
- CONFIRMED: log flood. `dac_task.c:260` `ESP_LOGE` per failed open, plus the `ad3552r_board.c` log, once per dropped chunk (about every 20 ms because of `drop_paced`, `dac_task.c:372-378`). Rate-limit it.
- Plan wrong in detail: the guard register list is not `0x00,0x14,0x16,0x19,0x1A` (see `s_guard_regs`, `ad3552r_board.c:228-238`); `0x0F` is read in `stream_open` (`ad3552r_board.c:~342`) so "never checks TRANSFER_REGISTER" is wrong. The plan's claimed `0x4B/0x4E` addresses are wrong too.
- UNVERIFIABLE (root cause of the DAC loss): "stream held with CS low across ATmega barriers (`try_start` 387-391, `apply_power` 322-330) lets a glitch desync the DAC". The code path is real: `apply_barrier()` and `apply_power()` run with the session open and CS low for up to 50 ms (`POWER_WAIT_MS`). It matches the brief (loss only in laser runs, since power barriers and EMI both only exist then; an S0 job has none). But `IDLE_CLOSE_US` is 30 ms, so the stream already sits open and idle by design. No evidence ties loss to this window. Treat as hypothesis.

### CRITICAL-5: Z-Bro 10 s homing drop — **MOSTLY WRONG; mechanism UNVERIFIABLE**
- WRONG: S3 `send()` without `SO_SNDTIMEO`. `zbro_link.c:215` sets it to 1 s, and keepalive options right after.
- WRONG: `autohome_tried` loops HOME forever. The real code uses `auto_try = (s_auto == ZBRO_AUTO_PENDING)` (`zbro_link.c:268`) and sets `s_auto = ZBRO_AUTO_FAILED` when the link drops mid-home (`zbro_link.c:383-386`).
- WRONG: S3 never calls `esp_wifi_set_ps(WIFI_PS_NONE)`. It does at `net_wifi.c:221`. (C3 side unchecked: no match in `CompanionZ/main/*.c`.)
- PARTLY RIGHT: the S3 sends no PING while `exp != EXP_NONE` (`zbro_link.c:~314`). But HOME and MOVE are replied OK at once (`zbro_link.c:573` comment), so `exp` returns to NONE and PINGs resume every 1 s during a home. The 3 s C3 timeout (`zb_net.c:35,516`) therefore does not explain a drop at about 10 s.
- The C3 `send_line` (`zb_net.c:255-282`) already has a bounded stall (`TX_STALL_MS`) before it shuts the socket. It is not an immediate drop on EAGAIN as the plan says.
- 10 s equals `SILENCE_MS` (`zbro_link.c:26`, checked at `373`) on the S3. That fits "C3 went silent for 10 s" and "the C3 closed the link" (`zbro_link.c:235`) equally. Settling evidence: the C3's own log line at the drop. Three candidates: `"no PING for 3 s"` (`zb_net.c:517`), `"send failed ... dropping the client"` (`zb_net.c:277`), or a reboot, plus the C3 reset reason and uptime from its ESPFleet record (a stepper-current brownout during a long home is a strong candidate), and an S3 log of the order "C3 closed the link" vs "C3 not answering".
- The `REPLY_MS` 5 s to 8 s change only hides the symptom and slows detection of a dead C3. Do not do it blind.

### HIGH-1: `$231` auto guide under M3 — **CONFIRMED**
- `grbl.c:541-545`: with `s_m.laser == 3`, `S == 0`, `!preview` and `$231 > 0.5`, it calls `atmega_link_set_guide(true)` with no `wait_idle()`. `grbl.c:~448-456`: once a line leaves "M3 with S0", `wait_idle()` then guide off. So an M3 job that alternates S0 and S>0 moves drains the pipeline on every return to S>0, and the guide lights early. This matches brief item 5. The `m3_burned` proposal is reasonable. Simpler and safer: only enter auto guide when `!motion_busy()` (the frame case: a frame starts on an idle machine). Keep the default. Changing `$231`'s default is the owner's call (the plan asks that as question 1).

### HIGH-2: false underruns — **MOSTLY WRONG, one part CONFIRMED**
- WRONG: "`reap()` before dequeue gives a false warning" (`dac_task.c:477` vs `487`, check at `240`). If `s_isr_drained` is set the SPI queue ran dry (`stream_post`, `dac_task.c:164-171`). A chunk in `ready_q` at that instant arrived too late. That is a real underrun with the gate lit. Dequeuing first would hide real underruns and break the stats. Do not change.
- WRONG/HARMFUL: "premature `galvo_out_flush()` before `allowed_us`" (`motion_task.c:~208-218`). The comment at the call says the flush is deliberate: without it a lone G1 would wait forever. A flushed partial chunk keeps the gate state across chunks (`dac_task.c:172-175`). The proposed reorder would reintroduce the hang.
- UNVERIFIABLE: barrier chunk draining while the gate is lit (`galvo_out.c:379-385`, `dac_task.c:387-391`). The path exists (a power change between adjacent marks forces a full SPI drain, with the gate lit and a short stop in the mark). It is a plausible real cause of underruns on multi-power jobs, but nobody measured it. The plan's fix (clock out `laser_off_delay` before the barrier) changes gate timing semantics. Only do it after a host test with a pulse-density/power-change fixture, and after counting how many barrier-lit drains happen in a recorded job (`g_galvo.underruns` vs `g_galvo.barriers`).
- CONFIRMED but not a cause: motion (8) runs below grbl (9), `host_rx` (10), `net_srv` (10) (`motion_task.c:31`, `grbl.c:62`, `host_serial.c:18`, `net_server.c:17`). `net_server.c:17` comment says prio 10 is intentional for `!` and Ctrl-X latency (AGENTS.md also fixes the numbers). Lowering `host_rx`/`net_srv` below `motion` risks Ctrl-X/feed-hold latency, which is a laser-safety path. Raising only `motion` to 9 would be fine if measured. Do not touch the others.

### HIGH-3: safety gaps on ATmega fault / alarm — **MOSTLY WRONG**
- WRONG: `atmega_link_set_arm()`/`set_power()` return `bool`. They are `void` (`atmega_link.c:~300-320`, `atmega_link.h`). They set a desired state under `s_lock`, and `decide()` (`atmega_link.c:~270-285`) keeps sending until the ATmega confirms. A lost UART frame on disarm is already retried; there is no `s_armed = false` assignment to fix. `s_consecutive_fail`, `ATMEGA_ST_EMERGENCY` and `do_arm(false)` do not exist.
- WRONG: no ALARM path exists for "soft limit ALARM:2". Only `ALARM:3` (Z) exists, as `s_z_alarm` (`grbl.c:275`), which does not drive the laser. The other alarm state is `s_fault` (`grbl.c:1325`), for dead motion hardware, where nothing runs.
- UNVERIFIABLE (real question hidden in the claim): what the ESP32 gate does if the ATmega link dies mid-job. The ATmega disarms itself after 1 s (AGENTS.md), but the S3 gate is not killed. `laser_io_gate_kill()` on link loss would be a real safety improvement. It needs a design decision (the link can flap) and a host test. Not a trivial one-liner.

### HIGH-4: priority inversion and unpinned tasks — **MOSTLY WRONG**
- WRONG: espfleet is unpinned (`EspFleet.cpp:277`). `main.c:41-56` sets `spawn_task = false` and creates `espfleet` pinned to core 1 at prio 1. `EspFleet.cpp:380` (`xTaskCreate`) is only reached when `spawn_task` is true.
- WRONG: WiFi and lwIP on core 0. Already pinned to core 1 in `sdkconfig.defaults`.
- The only real item is `atmega_link` at prio 5 (`atmega_link.c:44`) while `dac_task` waits on it up to 50 ms. Raising it is plausible but must be measured (`ATmega did not confirm power` warnings). The plan's "10, above everything" is not justified. 7 or 8 would do.

### HIGH-5: no task in the TWDT — **WRONG for dac_task; the fix is dangerous**
- `dac_task` sleeps with `portMAX_DELAY` when idle (`dac_task.c:504` and `511`, `service()` return values; the plan says 20 ms `IDLE_POLL_MS`, which does not exist in this file). Subscribing it to TWDT would reset the chip 5 s after boot or after the last job. A watchdog feed for `dac_task` needs a bounded wait first. That is a behaviour change on the streaming path. Reject as written. `atmega_link` alone is fine (bounded waits), but low value.

### Medium findings I opened (not requested, noted for completeness)
- MEDIUM-5 (no TCP keepalive on the telnet server): WRONG for the listener, `net_server.c:158` already sets `SO_KEEPALIVE`; only the idle/interval/count values are unverified.
- MEDIUM-1 (log under mutex from core 0): real in principle (`dac_task.c` logs via `host_log_vprintf`), but the `espfleet` feedback-loop story is unsupported and `xPortGetCoreID()==0` → "don't take the mutex" would drop log lines (violates "do not remove logging"). Use a non-blocking take + drop counter, only on the dac_task paths.
- MEDIUM-2 (`galvo_out_abort` races motion): WRONG. The abort already runs inside `motion_task` (`motion_task.c:102-117`) with an epoch, `s_abort_ack` and a semaphore, and `abort_pending()` is checked in the drain loop (`motion_task.c:~229`).
- MEDIUM-3 (torn `s_status`): WRONG. `atmega_link_get_status()` copies under `portENTER_CRITICAL(&s_lock)` (`atmega_link.c:455-467`).
- LOW-1 (NaN): WRONG. `calib.c:172` `value != value` rejects NaN; infinity fails the `max` check.

### Crash A: does the plan prove it?
**No. It guesses.** The plan supplies a real latent race (lost gate edge) and asserts without evidence that it produces a 300 ms stall of the level-1 tick. The brief's evidence (IDLE0 in WAITI, all tasks blocked, no lock) says core 0 was idle and the tick never arrived. A bounded ISR cannot do that. What can: the tick source (systimer alarm for core 0) was lost or mis-programmed, level-1 was masked (INTENABLE/INTLEVEL), or the chip hung with the core waiting. Candidates worth checking: an interrupt allocation clash at the chosen level (gptimer at 3, SPI IRAM ISR "any of 1-3", WiFi), a long critical section on core 0, or a power/EMI event.

Evidence that would settle it, using what the debug build already has:
1. `freeze_watch.c` already snapshots, on a 150 ms tick stall, the task per core and the raw interrupt-controller status for sources 0..127 per core (`freeze_watch.c:20-60`) into RTC. Retrieve the `freeze:` boot report after the next crash (the plan never mentions this tool). If core-0 status shows the systimer source pending-but-not-served, the tick is masked. If a SPI/GPTimer source is stuck pending, the ISR chain is implicated.
2. Read INTENABLE / INTERRUPT (PS.INTLEVEL) on core 0 in the same capture (`rsr` special registers); the plan could add them.
3. Log the interrupt level and CPU interrupt number actually allocated for the SPI ISR and the GPTimer at boot (`esp_intr_dump()` is available in IDF 5.5).
4. Reproduction with a variable changed: same laser job, same hardware, once with the timer at level 1 (the plan's fix) and once unchanged. Only a repeat-rate comparison over many runs counts, because the crash is rare.
5. Separate EMI/supply from software: run the same S>0 job with the laser output disabled but the power words changing (barriers still happen) vs the real job. The brief says an S0 job (no barriers, no emission) was clean, so the barrier/power-change path itself is the differential to test first.

### The plan's other root-cause claims, in brief
- "`sample_cb` steals Core 0": WRONG (above).
- "Crash B / log storm": CONFIRMED as log flood only. The Crash B starvation itself is fixed on this branch by `drop_paced()`.

## 2. Work packages

### File-set overlap and order
The plan says Wave 1 and Wave 2 are disjoint. They are not:
- `src/grbl.c` priority edit (HIGH-4 "grbl 7") belongs to WP-7, while WP-3 and WP-6 also move priorities. All priority edits touch different files, but they must be one coordinated change, otherwise ordering across files is arbitrary. Do it as one reviewed commit.
- WP-4 depends on WP-1 for `ad3552r_board_recover()` (declared in the plan) and WP-7 depends on WP-3/WP-4 for `*_has_fault()`. Those functions only exist in the plan. If the dependency work is dropped, WP-4/WP-7 shrink.
- WP-1 and WP-4 both claim CRITICAL-4 behaviour and must agree on who closes the stream and who disarms. They are only disjoint by file, not by design.
- WP-3 edits `components/espfleet/src/EspFleet.cpp`, a vendored shared component also used by CompanionZ. No finding justifies touching it.
- WP-6 touches `galvo_out.c` and `motion_task.c`, which have the host trajectory tests. `make -C test/host` covers `motion.c` only, so it cannot prove the `galvo_out` barrier change.

### Acceptance checks: runnable without hardware?
- `pio run` builds: runnable, I ran both envs. "0 warnings in src/": runnable. "0 warnings" without scope: not (12 vendor warnings today).
- WP-1, WP-4 acceptance is `grep` of the patch. That proves the edit exists, not that it works. There is no host test for ISR logic or recovery. Add a host-compilable check or accept that these are bench-only.
- WP-3: `make -C test/host/hostlink` exists, but the "strip 0x90-0x9F" test is moot (already dropped at `host_serial.c:293`).
- WP-5: `test/host/zbro` exists; ok for protocol logic, but the S3 session loop is not covered.
- WP-7: `test/host/gcode` exists; the new cases ("G10 L20", "G92.1", NaN) mostly test things already done.
- WP-2 `sdkconfig` check: deleting `sdkconfig.esp32-s3-devkitc-1*` (both are git-ignored and exist) is runnable. AGENTS.md only asks for the debug one.

### Laser safety and logging review
- WP-4 item "TWDT for dac_task": would reset an idle machine (see HIGH-5). Reject.
- WP-4 `intr_priority 3 -> 1` and deleting the stale re-arm: can delay gate-off edges behind level-1 ISRs. Reject (keep 3).
- WP-4/WP-1 DAC reset: a hardware RESET returns the DAC output registers to their power-on value. The plan does not require gate off, power zero and disarm first, nor a park at 0 V afterwards. As written this can move the galvo mid-mark with the laser possibly emitting. Must kill the gate and disarm before the pulse, and re-park outputs before the next stream open.
- WP-2: removing `esp_task_wdt_print_triggered_tasks` and `snprintf` from the user handler removes the diagnostic that names the starved task (a debug tool). That is removing logging. Reject. Making panic reboots silent also removes the panic log.
- WP-3 item 5 (drop core-0 logs when the mutex is busy) removes logging unless counted. Add a drop counter.
- WP-6 abort rewrite: abort is already safe; do not touch it.
- WP-7 `enter_alarm()` is a reasonable idea (gate kill, disarm on any alarm) but is built on non-existent faults.
- Size: WP-3 (nine unrelated changes in five files, two components) and WP-7 (nine changes across grbl.c, calib.c, gcode_parse.c) are too large for one implementer. WP-4 is "Medium-High" by the plan's own words on the real-time path and has five separate changes.

### Verdict per work package

| WP | Verdict | Exact changes |
|---|---|---|
| WP-1 AD3552R driver + recovery | **APPROVE-WITH-CHANGES** | Keep: `buf[AD3552R_MAX_REG_SIZE + 1]` in `write_reg`/`read_reg` (`ad3552r.c:~565,~605`). Hold the `memmove` readback change until `$RB` is checked on the bench (see CRITICAL-1). Drop the "re-apply registers 0x00,0x14.." list; reuse the init path (`ad3552r_init` already pulses RST via `s_reset_gpio_param`). Recovery only from `dac_task` context with the stream closed, gate killed, ATmega disarmed/power 0 first, outputs re-parked after. Rate-limit/aggregate the stream-open logs. Do not touch warnings in the vendored file beyond the buffer. Split into 1a (buffer) and 1b (recovery). |
| WP-2 TWDT hook + sdkconfig | **REJECT** | Nothing in it is a verified defect. `GPIO_CTRL_FUNC_IN_IRAM` is not on the gate path (`laser_io.c:57` uses `gpio_ll_*`). The wifi/lwIP pinning and coredump/TWDT-panic lines already exist. The ISR handler change removes diagnostics. Silent reboot removes the panic log. Replace with a read-only evidence package (below). |
| WP-3 ATmega/host/log/net | **REJECT as a package; salvage three items** | Salvage: non-blocking log take with a drop counter on core 0, optional modest `atmega_link` priority raise after measurement. Drop: `atmega_link_has_fault`/gate kill (design needed), priority lowering of host_rx/net_srv, EspFleet edits, override-byte test, `s_status` spinlock (exists), TWDT subscription. |
| WP-4 dac_task | **APPROVE-WITH-CHANGES (reduced)** | Keep: `s_alarm_armed = true` before `gptimer_set_alarm_action()` (`dac_task.c:114-115`), with a short critical section around the state updates in `stream_pre`/`stream_post`, timer stays at level 3. Keep: on `session_open()` failure, disarm and zero power once, set a sticky fault, log once per transition (`dac_task.c:407-409`). Optional, after bench evidence: close the stream before `apply_barrier()`/`apply_power()` (`dac_task.c:387-391,401-405`), and accept the extra reopen time on power changes. Drop: priority 1, deleting the stale re-arm, TWDT add, `reap()` reorder, "retry failed disarm" (retry already exists in `atmega_link`). |
| WP-5 Z-Bro/WiFi | **REJECT** | S3 side has `SO_SNDTIMEO`, PS_NONE, `s_auto` handling, mDNS handled; remaining items are guesses. First collect the C3 drop log (see CRITICAL-5). Only candidate worth keeping once the cause is known: send PING also while waiting for a reply, which is cheap and safe. |
| WP-6 motion/galvo_out | **REJECT** | Abort race and `s_pos_*` race do not exist (abort is in `motion_task`, `atmega_link` has the lock). The flush reorder reintroduces a hang. The `galvo_out` barrier change is plausible but unmeasured and has no host test. Replace with: add counters/logging for barrier-lit drains, then decide. |
| WP-7 grbl | **APPROVE-WITH-CHANGES (reduced)** | Keep only the `$231` fix: `grbl.c:541-545` guarded by `!motion_busy()` (and optionally the `m3_burned` flag), plus `wait_idle()` before turning the guide on. Add a host test only if the logic can be factored out of `grbl.c`, which has no host harness today. Drop: `$HX/$HY` (accepted already), WCS (implemented), `Jog`/`Home`/`Bf:` (exist except `Home` while Z is homing, see below), NaN (rejected), `enter_alarm()` on non-existent faults, `grbl` priority 7. |

## 3. Rayforge gap list

Every row was checked against `/Volumes/Projects/rayforge` (read-only) and this tree's `grbl.c`.

| # | Plan claim | Finding | Decision |
|---|---|---|---|
| G1 | status reports `Idle` during Z move/home | Z busy is `Run` via `s_z_busy` (`grbl.c:~1113`). Rayforge accepts `Home` in its state parser, but the plan's citations (`grbl_util.py:216-291`) are not verified. A `Home` state during `$HZ` is cosmetic. | skip (low value) |
| G2 | `$HX`/`$HY` rejected with error:3 | Rayforge does send `$H{axis}` (dialect `home_axis: $H{axis_letter}`; `grbl_network.py:556`, `grbl_serial_simple.py:674`). Firmware accepts `$HX`/`$HY` at `grbl.c:999-1001` as no-ops returning ok. Plan's claim wrong. | skip |
| G3 | WCS / G92 / `$#` stub | Implemented (`grbl.c:120-125,659,1121-1128`). `docs/rayforge/wavemaster-machine.yaml` has `supports_wcs: false` by choice. | skip |
| G4 | bidirectional raster `$216` | `bidir_offset.py` exists in Rayforge; skipping a second shift in firmware is right. | skip (agree) |
| G5 | framing with `M3 S0` and guide | Real, and the `$231` issue above. Rayforge frames with M3 (`focus_laser_on: M3 S{power}` in the dialect). | worth-doing (it is HIGH-1) |
| G6 | M3 vs M4 `dynamic` unused | Agree it is not needed for a galvo. | skip |
| G7 | `Jog` state, jog cancel | `Jog` exists (`grbl.c:1116`), 0x85 handled (`host_serial.c:276`). | skip |
| G8 | probing G38.2 | Not applicable. | skip (agree) |
| G9 | air assist M7/M8/M9 | Accepted as no-ops; Rayforge dialect sends M8/M9. No hardware pin. | skip (agree) |
| G10 | realtime override bytes | Already ignored: `host_serial.c:293`. | skip |
| G11 | `Bf:` in status | Already emitted: `grbl.c:1134`. | skip |
| G12 | door/interlock via ALARM | Needs a fault source that does not exist; see HIGH-3. | skip until a real fault model exists |
| G13 | fingerprinting/mDNS | Already matches. | skip (agree) |

Net: no Rayforge gap in the plan is worth a work package. If features are wanted, they need to start from `docs/rayforge/*.yaml` and the live `$I`/`?` output, not from this table.

## 4. What the plan missed, ranked

1. **It did not read the current source.** Many symbols and "missing" features exist. Any implementer following it would edit lines that are not there. Highest impact.
2. **The existing diagnostics for Crash A** (`freeze_watch.c` interrupt-status snapshot, `debug_tools.c` core-1 sampler) are not used. The cheapest, correct next step is to read those records after the next crash, not to change the ISR design.
3. **Real race at `dac_task.c:114-115`** (edge lost, gate stuck up to one chunk) is the one true laser-safety defect in the whole ISR code, and the plan buries it in a bundle with unsafe changes.
4. **Gate-off on DAC recovery/failure ordering.** Neither disarm on a failed `session_open` nor "park DAC and kill gate before a hardware reset" are specified safely. See WP-1/WP-4 above.
5. **Log flood on stream-open failure** (`dac_task.c:260` and the board log, about 50 pairs/s) is the one part of CRITICAL-4 that is cheap and certainly right.
6. **ATmega link loss with the gate active:** the ESP32 gate is not forced off when `link_ok` goes false mid-job (the ATmega disarms after 1 s). A design question for the owner, not a trivial patch.
7. **Observability for underruns:** nothing measures how many underruns coincide with power/PRR barriers vs real host starvation. Without it, H2 fixes are guesses.
8. **No Z-Bro evidence request:** the C3's log line at the drop and its reset reason would settle item 4 of the brief in one reading.
9. Hygiene: acceptance checks that are only `grep`; the vendored `ad3552r.c` already emits 12 warnings and the plan demands "0 warnings".

## 5. Recommended order (what I would run)

1. **Evidence first, no firmware change:** read the `freeze:`/`debug` reports and the C3 log after the next crash and the next Z-Bro drop (needs hardware, owner side). Add `esp_intr_dump()`/INTLEVEL capture to the debug build only if the first reads are inconclusive.
2. **WP-1a**: `buf[... + 1]` fix only. Build both envs. Smallest, certainly correct.
3. **WP-4 reduced**: arm-ordering fix and critical section; `session_open` failure handling (disarm, zero power, sticky fault, single log). Build both envs, review `stream_pre`/`stream_post` by hand, then bench with the scope on DB25 pin 19.
4. **WP-7 reduced**: `$231` fix with host test if the logic can be factored; otherwise bench.
5. **WP-1b** DAC hardware reset recovery, after (1) shows the DAC wedge really needs it and after the gate-off/disarm ordering above is agreed.
6. Optional later: net keepalive, non-blocking core-0 logging, `atmega_link` priority, Z-Bro PING during reply wait, ATmega-link-loss gate kill (needs owner decision).

Not to run: WP-2, WP-3 (as a package), WP-5, WP-6.

Overall: **NO-GO on PLAN.md as written; GO on steps 1-4 above after the plan is regenerated against the actual tree.**
