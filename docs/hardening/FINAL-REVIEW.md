# Final review: `hardening/audit-and-features` (HEAD dd1a4b4)

Reviewer: final pass, no hardware, nothing edited except this file. Line numbers are from the tree at HEAD.
Read first: BRIEF.md, REVIEW.md, DECISIONS.md. Reviewed diff: `git diff main...HEAD -- src components` (3 files).

**Build (clean, one after the other):** `-e esp32-s3-devkitc-1-debug` SUCCESS (25.8 s), `-e esp32-s3-devkitc-1` SUCCESS (25.8 s).
Each log has 12 warnings, all in `components/ad3552r/ad3552r.c` (vendor driver, same as before). **0 warnings in `src/`.**
Host tests were not required (no `motion.c`/`gcode_parse.c`/`host_serial.c` change).

## Verdicts

| # | Commit | Verdict |
|---|--------|---------|
| 1 | 49f72b5 dac_task: pace the drop of chunks | **APPROVE-WITH-CHANGES** (abort latency, see R2) |
| 2 | 8d5905c ad3552r: write_reg/read_reg buffers | **APPROVE** |
| 3 | 4a6db5c grbl: auto guide only until the first S>0 mark | **APPROVE** (one minor note, R3d) |
| 4 | 40a4a79 dac_task: alarm race, ISR critical section, session_open failure | **APPROVE-WITH-CHANGES** (R2 findings; R1 is OK) |
| 5 | dd1a4b4 grbl: `$H` also homes Z | **APPROVE** (behaviour change to be aware of, R4) |

(096e93e and 4263157 are docs only.)

### 1. 49f72b5 `drop_paced()` — APPROVE-WITH-CHANGES
- `dac_task.c:384-392` computes `n_ticks * tick_us / 1000 + 1` ms, recycles the chunk, `vTaskDelay`. This fixes Crash B: the producer (`motion`) now blocks on the chunk pool at real time, so IDLE1 runs. Delay is in the task, never in an ISR or the stream (no stream is open on this path), so it respects the AGENTS.md "no delay in the stream" rule.
- **Problem (abort latency):** the delay is not interruptible and the `while (!held)` loop in `service()` (`dac_task.c:~470-480`) does not look at `g_galvo.abort` between chunks. `galvo_out_abort()` waits only 300 ms for `abort_done` (`galvo_out.c:528`). `CAL_TICK_US` default is 10 µs (20 ms per 2048-tick chunk), but it can be set up to 200 µs (`calib.c:42`): one chunk is then 410 ms, already over the 300 ms ack timeout. At 40 µs a full ready queue (up to 8 chunks, `galvo_chunk.h:29`) is 650 ms. Result: `abort: dac_task did not acknowledge within 300 ms` (`galvo_out.c:529`), and the producer reset runs while `dac_task` still owns chunks (the "chunks not returned" error at `galvo_out.c:547`). The laser itself stays safe (gate killed at `galvo_out.c:512`, stream is closed), so this is not a safety bug, but Ctrl-X during a failed-open job misbehaves at slow tick settings. At the default tick it is fine (<=164 ms).
- **Change asked:** in the fault path replace `vTaskDelay` by a wait that ends on abort (e.g. `ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms))`, `galvo_out_abort` already calls `dac_task_notify()` at `galvo_out.c:516`), and make `service()` re-check `abort` after `try_start()` returns 0. Same for `held` (feed hold is ignored while dropping).

### 2. 8d5905c ad3552r buffers — APPROVE
- `ad3552r.c:490-495` `ad3552r_convert_to_single_msg` shifts `data[i]=data[i-1]` for `i=len..1`, so with `len=3` it writes `data[3]`; `ad3552r.c:526-532` then sends `len+1` bytes from the same buffer. The new size `AD3552R_MAX_REG_SIZE + 1` (`ad3552r.c:579,620`) covers exactly that. The only other `ad3552r_single_transfer` caller (`ad3552r.c:1902`) already uses `MAX_NUM_CH * MAX_REG_SIZE + 1` (`ad3552r.c:1875`), and the CRC paths use their own buffers (`ad3552r.c:337,414`). `src/ad3552r_board.c` uses `ad3552r_transfer`, not the single transfer. No behaviour change besides removing the 1-byte overflow. Needs `$RB`/boot self-test on the bench as REVIEW.md says, but nothing here changes the bytes sent.

### 3. 4a6db5c auto guide — APPROVE (see R3)

### 4. 40a4a79 alarm race / critical section / session_open failure — APPROVE-WITH-CHANGES (see R1, R2)
Safe to merge as is; the changes below are small hardening, not blockers.

### 5. dd1a4b4 `$H` homes Z — APPROVE (see R4)

## Risk areas

### R1. portMUX critical section (`s_edge_mux`) — **OK**
What I opened: `dac_task.c:63-190`, ESP-IDF `portmacro.h:460-461` and `port.c:495-517` (this tree's `framework-espidf`), `gptimer.c:233-344`, `laser_io.c:25-73`, sdkconfig for both envs.
- **Deadlock:** none. `portENTER_CRITICAL_ISR` is `vPortEnterCritical`, which raises INTLEVEL to the FreeRTOS max-syscall level (3 on the S3) and takes the spinlock. The gptimer ISR (`intr_priority = 3`, `dac_task.c:575`) and the SPI ISR (level 1) are both installed from `dac_task`, i.e. on core 0 (`dac_task_start` pins to `DAC_TASK_CORE`). While the SPI ISR holds the lock, level-3 is masked, so `on_alarm` cannot preempt it and spin forever. The reverse (level 1 cannot preempt a level-3 ISR) is trivial. `s_edge_mux` has no other users. `vPortEnterCritical` nesting is counted per core, so `on_alarm` nested in a task-level critical section is fine. `CONFIG_FREERTOS_CHECK_PORT_CRITICAL_COMPLIANCE` is off in both sdkconfigs, so the `_ISR` variants are not aborting.
- **Time held with interrupts masked:** short and bounded. Longest path is `stream_pre` (`dac_task.c:~148-158`): `isr_disarm` + `laser_io_gate` + `isr_arm_next`. The `isr_arm_next` loop applies edges already in the past; since `START_LATENCY_COUNTS` is 0 and edges are strictly increasing and >0 (`galvo_chunk.h:15`), it normally runs 0-1 iterations (hard bound is 512 `GALVO_CHUNK_MAX_EDGES`, each a GPIO write plus a counter capture, microseconds). This also masks the level-1 tick for those microseconds, far below the 300 ms INT_WDT, so it does not worsen Crash A. `vTaskNotifyGiveFromISR` and the yield are deliberately outside the critical section (`dac_task.c:~187-190`).
- **IRAM / no flash-resident call, no log, no block:** every function in the sections (`timer_now`, `isr_disarm`, `isr_toggle`, `isr_arm_next`, `on_alarm`, `stream_pre`, `stream_post`) is `IRAM_ATTR`. Callees: `gptimer_get_raw_count`, `gptimer_set_alarm_action` (both IRAM via `CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM=y`, sdkconfig:833; `s_alarm_cfg` is static internal RAM, which that function requires, `gptimer.c:313`), `laser_io_gate` (`IRAM_ATTR`, `laser_io.c:65`, only `gpio_ll_*` and atomics), `vPortEnterCritical/ExitCritical` (IRAM, `FREERTOS_PLACE_FUNCTIONS_INTO_FLASH` not set), `atomic_*`. No `ESP_LOG*`, no mutex, no allocation in the new region. Nothing new calls the flash.
- **Early-wake re-arm branch (`dac_task.c:~131-135`):** unchanged logic, now inside the lock. It sets `s_alarm_armed=false`, calls `isr_arm_next()` without advancing `s_edge_idx`, so the same edge is re-armed (or applied at once if it has meanwhile passed). Terminates (single recursion-free call). Correct.
- **`s_alarm_armed` consistency:** every write is inside the lock (or in `isr_disarm`, which is only called inside it). Cases: arm OK sets true (`:~114`); `gptimer_set_alarm_action` returning an error now sets false (`:~115-116`, the fix); `isr_disarm()` sets false after disabling; `on_alarm` clears it before toggle/re-arm; `stream_pre/post` start with `isr_disarm()`. Setting it true *before* the call is irrelevant now (no ISR can intervene) and makes the failure path correct. `gptimer_set_alarm_action` can only fail for a NULL timer or a config pointer outside internal RAM, or auto-reload with equal counts; none of these occur (static cfg, auto-reload false), so the error branch is defensive.
- Minor, no change needed: if the call ever did fail, that gate edge is lost and the gate keeps its level until `stream_post`, which forces it off when the queue empties (`dac_task.c:~175-181`), so the laser cannot stay on past the chunk/stream end.

### R2. `s_fault`, disarm, first chunk after recovery, `drop_paced` and abort — **findings**
Opened `dac_task.c:200,262-300,327-380,420-430,437-470`.
- **Disarm / zero power on failed open: OK.** `try_start` (`dac_task.c:~422-428`) sets `s_fault`, calls `atmega_link_set_armed(false)` then `atmega_link_set_power(0)`; both are non-blocking (lock + kick, `atmega_link.c:385-410`) and `set_armed(false)` already zeroes the desired power (`atmega_link.c:390-391`); the ATmega also disarms on the heartbeat. Once per fault (`!s_fault` guard), repeated `session_open` failures log nothing more (`:273-274`). `s_fault` is only read and written in `dac_task`, so no race.
- **Gate in this path:** no stream was ever started, so `stream_pre` never raised the gate; the gate was lowered by `session_close()` (`:307`), `do_abort` (`:~516`), or the producer. Not touched by the diff, but there is no explicit `laser_io_gate(false)` at the fault point. Suggestion (belt and braces, one line, ISR-safe): add `laser_io_gate(false);` next to the disarm. Not a blocker.
- **Unarmed first chunk after recovery (finding, minor):** `needs_arm()`/`needs_power()` are evaluated *before* `session_open()` in `try_start` (`:~404-418`), and `s_fault` is cleared only inside a successful `session_open()` (`:295`). So the first chunk streamed after the DAC comes back is evaluated with `s_fault` still true: no arm, no power, and it plays with the laser disarmed. Safe direction (no emission), but that chunk's marks are silently lost. The next chunk re-arms (`needs_arm` now true; `do_arm` drains and closes the session first) and re-applies power (`st.power` is 0). Same after a Ctrl-X: `do_abort` resets `s_arm_failed` (`:~511`) but not `s_fault`. Suggested change: clear `s_fault` before the `needs_arm`/`needs_power` checks (e.g. try `session_open()` first when `s_fault`), or `return 1`/re-evaluate once after a successful open.
- **`drop_paced` vs abort:** see verdict 1. Latency up to one chunk (410 ms at `$tickus=200`) per drop and up to the whole ready queue because the loop does not re-check `abort`; exceeds the 300 ms ack at `galvo_out.c:528` for ticks above ~15-40 µs. Not a laser-safety issue (kill is set before the wait, `galvo_out.c:511-515`), it is an abort-responsiveness bug. Fix as described in verdict 1.
- `abort` path itself (`do_abort`, `:~480-520`) is unchanged and still gates off, then `session_close`.

### R3. grbl.c auto guide — **OK, one minor note**
Opened `grbl.c:85-89,127-147,150-165,433-445,470-480,555-575,630-645`.
Mechanics: `auto_guide_should_enable` (`grbl.c:~156-164`) = `!s_auto_guide && M3 && !(S>0) && !preview && !m3_has_marked && $231`. In the G1 branch the guide is turned on after `wait_idle()` (`:~565`), and `s_m3_has_marked` is set when a G1/G2/G3 executes with `M3 && S>0` (`:~570`). The existing guide-off logic at `grbl.c:~471-476` still runs in the modal phase of the line and switches the guide off as soon as the line leaves "M3 with S0".
- (a) **Frame** (`G0` corner, `M3 S0`, `G1` moves, `M5`): the G0 is a `MOTION_JUMP` and never enters the guide branch. First `G1` at S0: should-enable true, `wait_idle()` drains the jump, guide on, `s_auto_guide=true`. Later G1: `s_auto_guide` already true, no re-entry; `s_m3_has_marked` stays false (S=0). `M5`: `s_m3_has_marked=false` (`:~440`), guide off after `wait_idle()` (`:471-476`). OK. The only difference vs before is one drain at the first G1, which is the stated intent.
- (b) **M4 job:** `laser==4` never satisfies the condition; `M4` resets `has_marked` (`:~440`). Guide never on. OK.
- (c) **M3 job: S0 travel first, marks later, S0 travel again:** first S0 G1 lights the guide (this is the existing behaviour for a job that opens with an S0 move, and is the unavoidable ambiguity: it is indistinguishable from a frame). The first `S>0` line goes through the modal block, `S>0` makes the guide-off condition true, so after `wait_idle()` the guide is turned off, then `s_m3_has_marked=true` is set in the G1 branch. A later `S0` travel: `m3_has_marked` true so it is not re-enabled, and `s_auto_guide` is false. This is the case the commit fixes. OK. (Edge: a mark line that has `S>0` but no XY movement does not set the flag; harmless.)
- (d) **M3 stream followed by a new M3 stream without M5:** `s_m3_has_marked` is cleared only by `M4`/`M5` (`:~440`), `$`-reset/`modal_reset` (`:136`) and program end `M2/M30` (`:637`); it is **not** cleared when a new `M3` arrives while it is already 3. So a second M3 stream that begins with S0 moves will not get the auto guide until M5 or a reset. Rayforge ends every job and frame with `M5` (and the frame has a leading `M5` from the previous one), so in practice this is fine and the failure direction is "no guide lit", which is the safe one. Accept; note it as a known limitation. If wanted: reset on a `M3` whose line is preceded by an idle drain, but I would not do it.
- Safety: guide off is still forced by every existing path (M5, program end, `modal_reset` at `:133-136`, `$X`-free). The flag cannot keep the guide on, since it only ever *suppresses* enabling.

### R4. grbl.c `$H` routing — **OK**
Opened `grbl.c:998-1000,1028-1036,907-950,1113-1120,204-212,281-296`.
- `$H` (exact, after upper-casing and trimming, `:~1005-1008`) now returns `calib_get(CAL_ZBRO_EN) > 0.5f ? home_z() : 0` (`:1031-1032`). `$250` default is 0 (`calib.c:63`), so with the stock configuration `$H` is still an accepted no-op: nothing changes for users who never enabled the Z-Bro.
- With `$250=1` a plain `$H` now moves Z. Rayforge triggers it from "Home All" (`rayforge/ui_gtk/machine/jog_widget.py:453`, dialect `home_all: $H`, `docs/rayforge/wavemaster-dialect.yaml:33`) and from `home_on_start` (`mainwindow.py:562`, machine default False). That is exactly the owner's decision 3 (`DECISIONS.md`), so it is intended. Nothing sends `$H` on connect by itself. Be aware: with the Z-Bro offline `home_z()` now returns an error (`z_fail` maps to 20, `ZBRO_ERR_OFFLINE` raises no alarm, `grbl.c:281-290`) instead of `ok`, so "Home All" shows an error in that case where it used to succeed. Acceptable, document it.
- `wait_idle` / ALARM: `home_z()` (`:895-905`) is the old `$HZ` body moved verbatim: `wait_idle()` first (returns `E_ABORT` on reset), `s_z_busy` around `zbro_home`, `z_fail_alarm` raises `ALARM:3` for real faults only (not for offline/busy/not-homed/range/arg). `$HZ` itself still routes via `exec_zbro` (`:999-1000`, `:940-941`) to the same function. While `s_z_alarm` is latched, `process_line` lets `$`-lines through (`:1115-1119`), so `$H` can be used to retry; G-code stays locked out. Unchanged.
- `$HX`, `$HY`, `$X`, `$C`, `$SLP` still hit the single no-op condition (`:1033-1036`). `$X` with a Z alarm is still handled before dispatch in `process_line` (`:1115-1117`). `$H` is only removed from that list. Verified by reading the full condition.
- Not checked: Rayforge's timeout waiting for the `$H` `ok` while `zbro_home` runs for up to `$ZBRO_TIMEOUT_S`. Same as `$HZ` today; bench item.

### R5. Laser-safety invariants on the touched paths — **OK**
- Gate off: `stream_post` forces it off when the queue empties (`dac_task.c:~175-181`) now atomically with `s_isr_drained`; `stream_pre` sets it from the chunk under the lock; `laser_io_gate` honours the kill flag twice (`laser_io.c:65-73`); `session_close` and `do_abort` call `laser_io_gate(false)` (`:307,~516`); the fault path never raises it (no stream). `galvo_out_abort` sets the kill before anything else (`galvo_out.c:511-515`). The race fix removes the window where `on_alarm` and `stream_post` could interleave and re-raise the gate after the drain.
- Disarm: fault path disarms and zeroes power (R2). `needs_arm`/`needs_power` are suppressed while `s_fault` so no arm can be issued while the DAC is dead (the intended effect), at the price of the unarmed first chunk after recovery (R2).
- Guide laser (grbl): can only be left on by the auto guide if the M5/program-end/reset paths fail, and those are unchanged and now preceded by `wait_idle()` (R3). `modal_reset` still clears it.
- `$H`: wait_idle drains and gates off before Z moves (R4), same as `$HZ`.

## Summary of changes requested (none blocking; all small)
1. `dac_task.c` `drop_paced`: make the wait interruptible by abort and have `service()` stop dropping on `abort`/`held` (verdict 1 / R2).
2. `dac_task.c` `try_start`: clear or re-evaluate `s_fault` before `needs_arm`/`needs_power` so the first chunk after recovery is armed and powered (R2).
3. Optional: `laser_io_gate(false)` at the fault point (R2).
4. Document: `$H` with `$250=1` returns an error if the Z-Bro is offline; an M3 stream after an M3 stream without M5 does not auto-light the guide.

Not verified (needs hardware/bench, per AGENTS.md section 3.6): gate-edge timing under the new critical section on the scope, `$RB` readback after the buffer fix, `$H` with the real Z-Bro.
