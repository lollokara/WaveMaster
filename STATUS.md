# WaveMaster Status

Last updated: 2026-07-23 (autonomous iteration session, part 2)

## What this is

An ESP32-S3 firmware for a fiber-laser galvo controller:
- Drives an AD3552R dual DAC over SPI to produce the X/Y analog voltages
  for the galvo drive.
- Speaks a GRBL 1.1-compatible serial protocol (including arcs and custom
  M-codes for guide-laser/power/arm - see below) so GRBL-aware senders
  (e.g. rayforge) can drive it with standard G-code.
- Talks to a separate ATmega328P board (`../arduino-laser-control/`) over
  a second serial link for guide-laser on/off, power setpoint, and system
  arming - this ESP32 firmware does NOT own those, only firing (Engrave
  GPIO) and PRR (PWM), which it drives directly.
- Runs on both ESP32-S3 cores: core 0 owns the SPI/DAC hardware, core 1
  owns the serial link and G-code parsing, connected by a FreeRTOS queue.

## Critical bug found and fixed this session (single-point DAC writes were a no-op)

**Every plain move and paced-stream point ever written via
`ad3552r_board_write_volts()` silently never reached the DAC's output
register - the single most severe correctness bug found this session,**
discovered while chasing a user report that the physical galvo/DAC output
wasn't changing at all despite `MPos`, `?`, and per-move log lines all
reporting correct-looking values.

**Root cause**: `ad3552r_board_write_volts()` called the vendored
driver's `ad3552r_set_ch_value(desc, AD3552R_CH_CODE, ch, code)`. That
function's `AD3552R_CH_CODE` case is entirely gated behind `#ifdef
XILINX_PLATFORM` in `components/ad3552r/ad3552r.c` - with no code outside
that `#ifdef` and no `break` statement, the switch silently falls through
into the next case (`AD3552R_CH_RFB`) on any non-Xilinx platform,
including this ESP-IDF build. That case just stores the value as a
software "feedback resistor" field and recalculates gain/offset - **not
a DAC write, and not even an SPI transaction of any kind.** It returns 0
(success) unconditionally, so nothing about this ever looked like a
failure: the computed voltage was correct, `moves=%lu` logged the right
target, `MPos` tracked correctly - the write itself just silently never
happened.

**Why it looked like it worked at boot**: the boot-time speed tests and
the "$RB" debug readback (added specifically to diagnose this - see
below) both showed sensible-looking DAC register values, because the
*streaming* path (`ad3552r_board_stream_xy()`, used by the boot
self-test, arcs, and hatch-fill outlines) builds its own raw
`ad3552r_transfer()` call directly - a completely different code path
that was never affected by this bug. Every plain `G0`/`G1` move, and
every paced-stream point (the feed-rate pacing and motion-planner output
added earlier this session), went through the broken function instead
and silently never reached the DAC.

**How it was actually found**: the user reported the physical DAC output
frozen at some arbitrary voltage no matter what was commanded. Rather
than guess, a new debug command was added - "$RB" (see `grbl_task.c`/
`dac_task.c`), which reads back the DAC's own output registers directly
over SPI (routed through `dac_task`'s queue, not called directly from
`grbl_task`, to avoid a cross-core SPI race that an earlier, wrong first
version of this same debug command introduced) - and confirmed the
registers stayed frozen at whatever the boot self-test's streaming call
last wrote, completely unchanged across several different commanded
positions. That pointed straight at `ad3552r_board_write_volts()`, and
reading the vendored driver source found the `#ifdef` fallthrough.

**Fix**: `ad3552r_board_write_volts()` now calls
`ad3552r_write_reg(g_dac, AD3552R_REG_ADDR_CH_DAC_24B(ch), code)`
directly - the same register `ad3552r_board_read_code()`'s readback
targets, and the same one the streaming path's raw transfer already
wrote correctly. `ad3552r_write_reg()`/`ad3552r_read_reg()` are not
gated behind the Xilinx macro. **Verified fixed on real hardware**: `$RB`
readback now correctly tracks every commanded position (`X5,Y-5` ->
`CH0=0xc000 CH1=0x4000`; `X-3,Y2` -> `CH0=0x599a CH1=0x999a`; `X0,Y0` ->
`CH0=0x8000 CH1=0x8000` - all matching hand-calculated expected codes for
the default 1V/mm calibration).

**Blast radius of this bug**: every plain `G0`/`G1` move (the entire
non-streaming motion path) and every feed-rate-paced or motion-planner-
paced point (added this session) were affected - meaning the extensive
hardware verification claimed earlier in this session for those features
(motion planner timing, feed pacing, wobble on plain `G1` runs) was
**verified only in software/log terms, not confirmed against actual DAC
output** until this fix. Arcs and hatch-fill outlines (the unpaced
streaming path) were NOT affected and their earlier hardware
verification stands. **Action item**: re-verify motion-planner/pacing/
wobble behavior against real DAC output (multimeter or scope) now that
the underlying write bug is fixed - the kinematics/timing logic itself
was never in question, but its actual electrical output was unconfirmed
until now.

## Second critical bug found and fixed this session

**Every voltage this firmware ever wrote to the DAC before this fix was
wrong**, saturating to the channel's max code regardless of the requested
voltage. Root cause: `ad3552r_get_offset()` follows the Linux IIO
convention this ported driver was written against - `offset` is in the
*code* domain (added before scaling: `Vout(mV) = (code + offset) * scale`),
not in volts/millivolts. The original `volts_to_code()` implementation
(written earlier this session, in what was then `cli_dac.c`, later
`ad3552r_board.c`) assumed `Vout = code*scale + offset` and computed
`code = (volts - offset) / scale`, which is wrong for this convention.

Caught by adding a register-readback correctness check to the DAC
streaming speed test (`dac_task.c`) - both channels reported `0xFFFF`
(max code) for two different requested voltages, which shouldn't be
possible and turned out to indicate the bug rather than a working
coincidence. Confirmed on real hardware after the fix: `+5V` on a ±10V
range now reads back `0xC000` (49152) and `-5V` reads back `0x4000`
(16384), both matching hand-calculated expected values from the
corrected formula `code = Vout(mV)/scale - offset`.

**Action item: any prior claims in this document or in commit messages
about specific voltage outputs being verified should be considered
unverified until re-checked against this fix.**

## Verified working on real hardware (this session)

- AD3552R SPI bring-up: confirmed product ID `0x4008`, register
  read/write, full `ad3552r_init()` success (CRC disabled - see below).
- **Volts→code conversion, register-readback verified** (see bug above).
- Dual-core task split: `dac_task` on core 0, `grbl_task` on core 1.
- Serial link: this board's single USB-C port is the ESP32-S3's native
  USB-Serial-JTAG peripheral, NOT physical UART0 - fixed by setting
  `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG` as the primary console
  (`sdkconfig.defaults`) and using stdio (`fgetc`/`fputs`) for the GRBL
  channel. Physical UART0 (GPIO43/44, "the default TX/RX pins") is instead
  used for the ATMEGA link (see below) - confirmed free for this purpose.
- End-to-end GRBL exchange over the actual serial port: boot banner,
  `?` status reports, `G1`/`G2` (arc) moves updating `MPos` correctly,
  `$100=`/`$101=` calibration settings persisting and taking effect,
  `M3`/`M5` engrave + `M62`/`M63` guide-laser + power (S-word) all
  dispatching. Now confirmed against the real ATMEGA board too (see
  "ATMEGA link" below) - genuine ACKs, not just correct NACK/timeout
  framing, once a TX/RX pin-assignment bug was found and fixed.
- DAC speed tests (see "Performance" below), including a working
  buffered-streaming write path verified via register readback.

## GRBL feature coverage

- Motion: `G0`/`G1` (linear), `G2`/`G3` (arcs, I/J center-offset form
  only - no R-radius form), `G90`/`G91` (absolute/relative), `G92` (set
  position), `G20`/`G21` (inch/mm units), `G4` (dwell, blocks the GRBL
  task for P seconds - core 0 keeps draining its queue normally).
- Laser mode (always-on, matching real GRBL's `$32=1` semantics): `M3`/`M4`
  record firing intent; `G0` always forces the Engrave output off
  regardless of intent (never marks during rapid traversal); `G1`/`G2`/`G3`
  restore it. This is what lets a bare raster stream of `G1 X.. S..` lines
  work without a fresh `M3` per line.
- `S` (0-1000, GRBL's standard laser-power convention) → sent to the
  ATMEGA as a 0-100% power setpoint (`SET_POWER`, command `0x04`).
- `Q` (custom, Hz) → pulse-repetition-rate, driven directly by this
  ESP32's own PWM (`prr_pwm.c`). Not `S`, since `S` means power here.
- `M62`/`M63` (repurposing GRBL's digital-output on/off M-codes) → guide
  laser on/off, sent to the ATMEGA (`SET_GUIDE`, `0x03`). Only one output
  channel exists, so the `P` word is accepted but not distinguished.
- `M10`/`M11` (custom - stock GRBL doesn't define these) → arm/disarm,
  sent to the ATMEGA (`SET_ARM`, `0x01`). `M10` (arm-on) blocks for ~2.5s
  waiting for the ATMEGA's ACK, matching that board's own 2-second
  blocking arm sequence (see `arduino-laser-control/src/main.cpp`).
- `M2`/`M30` (program end) → engrave off, back to absolute mode.
- Realtime: `?` (status report), `!`/`~` (feed hold/resume - acknowledged
  only, no motion queue exists to actually pause), Ctrl-X (soft reset).
- `$$` → settings dump: a handful of fixed/fake entries for sender
  handshake compatibility, plus the real, live, persisted calibration
  settings `$100`/`$101`/`$110`/`$111` (see Calibration below).
- Not implemented: `$H` real homing (no endstops on this system - `$H`/`$X`
  are acknowledged as no-ops), feed/spindle-override realtime bytes,
  arc `R`-radius form (repurposed as the repeat-count word - see the
  marking-quality features section), tool changes, probing (`G38.x`).

## Calibration (firmware-managed, persisted in NVS)

`src/calib.{h,c}`: per-axis volts-per-mm gain and volts offset, stored in
NVS flash (survives reboots), loaded at boot, live-settable via GRBL
`"$100=<value>"`-style commands:
- `$100` = X volts-per-mm, `$101` = Y volts-per-mm
- `$110` = X offset (volts), `$111` = Y offset (volts)

Defaults (`1.0 V/mm`, no offset) are **not calibrated to any real
optical setup** - set them via `$100=`/`$101=`/`$110=`/`$111=` once the
actual galvo/lens working-area scale is known; they persist automatically.

## ATMEGA link (`src/atmega_link.{h,c}`)

Talks the real binary protocol implemented in
`../arduino-laser-control/src/main.cpp` (not a placeholder): UART0,
250000 baud 8N1, single-byte commands with 1-byte ACK/NACK responses
(`SET_ARM`=0x01, `SET_FIRING`=0x02 [unused - firing stays on this ESP32],
`SET_GUIDE`=0x03, `SET_POWER`=0x04, `SET_PWM_EMIT`/`SET_PWM_SYNC`=0x05/0x06
[unused - not driven from GRBL words per the "ATMEGA doesn't own PRR"
requirement], `GET_STATUS`=0x07, `PULSE_LATCH`=0x08).

**Now verified end-to-end against the real ATMEGA board.** `M62`/`M63`
(guide laser), `G1 ... S<n>` (power), and `M10`/`M11` (arm/disarm) all
get genuine ACKs (`power = 30.0% (byte 0x4d) confirmed`, `system ARMED`/
`system disarmed`, no timeout warnings), with `?` staying responsive
through the ~2.5s arm sequence. This took three fixes stacked together,
found in this order:

1. **Level shifter on this link was unpowered** - every command timed
   out exactly as if the ATMEGA were disconnected. Fixed by powering it.
2. **The CH340 USB-serial chip on the Arduino board sits in parallel on
   the same TX/RX lines** and, while unpowered (USB unplugged), appeared
   to load/clamp the bus through its ESD protection diodes to a floating
   VCC rail - a known failure mode on CH340-based clones that lack series
   isolation resistors between the USB chip and the ATmega's UART pins
   (genuine Unos with a separate ATmega16U2 usually have these; cheap
   clones often don't). Fixed by powering the Arduino via USB (giving the
   CH340 a defined state) - though this alone briefly introduced a new
   problem (see below).
3. **TX/RX were swapped relative to what this firmware assumed.** UART0's
   IOMUX defaults are GPIO43=TX/GPIO44=RX; the actual wiring needed the
   opposite. Fixed in software rather than by re-wiring:
   `atmega_link_init()` calls `uart_set_pin(UART_NUM_0, 44, 43,
   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)`, telling the ESP32 to treat
   GPIO44 as TX and GPIO43 as RX. (This was tried once before fix #1/#2
   were in place and looked like it made things worse - with the level
   shifter unpowered, *nothing* worked regardless of pin direction, which
   is why the swap was reverted prematurely the first time. Once #1 and
   #2 were actually fixed, re-applying the swap was what finally made the
   link work.)

**A real instability surfaced during this debugging that's worth staying
alert for**: at one point (Arduino powered via USB, default/un-swapped
pins, continuous UART traffic running) the ESP32 hit a core-0 "Guru
Meditation Error: Interrupt wdt timeout" and rebooted mid-test - most
likely UART bus contention between the ESP32 and the Arduino's CH340
both driving the same lines. After settling on the swapped pin
assignment above, roughly 400 continuous toggle commands over ~100s
showed no further reboots, but this was not a long-duration soak test -
if a similar panic recurs during real use, suspect bus contention or
signal integrity on this link first, not a logic bug elsewhere.

### Fully async link, with an event queue for correctness ordering

Every command to the ATMEGA - ARM, GUIDE, and POWER - is now
fire-and-forget from `grbl_task`'s point of view: `atmega_link_set_*()`
just enqueues a request and returns immediately, and a dedicated
background task (`atmega_task_fn`, core 1, lower priority than
`grbl_task`/`dac_task`) does the actual UART round-trip. This means
`grbl_task` (owner of `?` status queries, new G-code lines, and Ctrl-X
soft reset) can never be blocked by the ATMEGA link, no matter how slow
any individual command is - previously POWER was synchronous and
blocked `grbl_task` directly.

- **ARM (M10) and GUIDE (M62/M63): pure fire-and-forget, no one waits.**
  `arduino-laser-control`'s `set_armed(1)` does a *blocking*
  `delay(2000)` before it even gets back to reading serial - up to
  ~2.5s. Nothing is tied to a specific galvo position for these, so no
  synchronization is needed at all. **Verified on hardware**: sent `M10`
  then immediately six `?` queries 300ms apart - all six returned
  instantly (`<Idle|MPos:...>`), with the ATMEGA's (expected, since
  disconnected) 2.5s timeout and "system ARMED" log line appearing
  later, in the background, after several status queries had already
  been answered.
- **POWER (S-word): async fire, synchronization relocated to `dac_task`.**
  For dithered/raster engraving the galvo must not reach the next
  position until the current pixel's power has actually taken effect on
  the ATMEGA, or position and power drift out of sync (smearing/
  ghosting) - that physical requirement doesn't go away just because the
  link is async. Rather than block `grbl_task` (which would reintroduce
  the exact freeze the ARM fix solved), the ordering constraint is
  enforced by a dedicated event: `atmega_task_fn` gives a binary
  semaphore (`s_power_settled_sem`) every time it finishes a POWER
  command, success or timeout. `grbl_task` marks the G0/G1 move on the
  same line with `gate_on_power` (see `laser_ctrl.h`) whenever that line
  also has an `S`-word; `dac_task` - which has nothing else to be
  responsive to - waits on `atmega_link_wait_power_settled()` (bounded,
  same 300ms ceiling as the old synchronous call) immediately before
  writing that specific move to the DAC. Net effect: the "stop" moved
  from the task that must stay responsive to the one that doesn't, while
  the pixel-level power-before-position guarantee is unchanged.
  **Verified on hardware**: `G1 X1 Y1 S500` returned `ok` immediately,
  `MPos` updated to `1.000,1.000` (move executed), with the ATMEGA's
  (expected, since disconnected) "no ACK for cmd 0x04 within 300ms" /
  "power = 50.0% ... NOT confirmed" log lines appearing right alongside
  it rather than blocking the line's `ok`.
- **Emergent benefit**: with POWER now going through the same 16-deep
  command queue as ARM/GUIDE, and moves going through the existing
  256-deep `laser_ctrl` queue, several G-code lines can be accepted
  ("ok"'d) well ahead of the ATMEGA actually catching up - a small
  lookahead buffer, similar in spirit to real GRBL's planner buffer,
  that falls out of the queueing rather than being separately designed.
- **What isn't handled yet**: the true end-to-end pixel-to-pixel latency
  of the POWER round-trip hasn't been measured against the real ATMEGA
  (not connected this session) - 300ms is a ceiling, not a measurement
  of typical latency. For high-rate dithering, that latency directly
  caps the achievable points/sec for jobs where `S` changes on every
  pixel, independent of how fast the DAC path is - measure it once
  hardware is available, and if it's too slow for the target engraving
  rate, look at either batching power+position changes into the
  ATMEGA's own firmware (a bigger change, out of scope here) or
  accepting a lower per-pixel rate for grayscale/dithered jobs
  specifically (vector/solid-fill jobs, which don't change `S` per
  point, are unaffected).

## Marking-quality features (closing the gap to BJJCZ/EZCAD-class galvo cards)

Added this session, all in service of one goal: real fiber-marking output
quality, not just "the galvo moves to the right voltage." **Not yet
re-verified on hardware after this change** (board was disconnected when
this batch landed - build is clean, logic below is a from-first-principles
implementation, re-flash and retest before trusting it in production).

- **Timing/delay compensation** (`calib.h` `$130`-`$134`, applied in
  `dac_task.c`): jump delay, mark delay, polygon delay (added on top of
  mark delay at every vertex - not angle-aware, unlike real corner-angle
  polygon delay), laser-on delay, laser-off delay. Applied as a plain
  `esp_rom_delay_us()` busy-wait on `dac_task` (core 0) after the relevant
  command - safe because this task has nothing else to stay responsive to
  (same reasoning as the ATMEGA power-settle wait). All default to small
  nonzero placeholders except radial/wobble/skywrite (default 0 =
  disabled, matching prior behavior exactly until tuned).
- **F-theta field correction** (`$140`, `laser_ctrl_mm_to_volts()`): a
  low-order radial distortion model, `r' = r*(1 + k1*r^2)` around the
  field center, applied in mm-space before the per-axis volts-per-mm/
  offset conversion. This is **not** a full per-point calibration-grid
  table (the harder, more accurate approach real high-end controllers
  use) - it's the standard low-order barrel/pincushion approximation,
  chosen because it needs one tunable number instead of a grid-calibration
  procedure/tooling that doesn't exist yet. Revisit if a single k1 doesn't
  correct the actual lens well enough.
- **Wobble** (`$150` diameter, `$151` pitch): circular kerf-widening offset
  applied perpendicular to travel direction, built into the same point-
  generation helper (`submit_run()` in `grbl_task.c`) used for pacing and
  skywriting - see "one pipeline" note below.
- **Skywriting** (`$160` margin, `mark_stroke()`): extends each marking
  stroke by the margin at both ends with the laser explicitly off (three
  separate queued primitives - laser-off lead-in, laser-on main run,
  laser-off lead-out - not one buffer with the GPIO toggled mid-stream,
  since this hardware has no way to toggle engrave state at a specific
  point inside a DAC burst). **Scope simplification**: applied per
  G-code-line stroke, not per continuous polyline - a sender's multi-line
  polyline gets a lead-in/out at every vertex (some redundant on/off
  toggling, each paying laser-on/off delay) rather than one continuously
  skywritten path. Real per-polyline skywriting would need look-ahead
  across multiple G-code lines, a bigger change. Default `$160=0` disables
  this entirely, reducing to the prior (pre-this-feature) always-on-
  between-marking-lines behavior.
- **Basic feed-rate pacing** (`F` word, sticky, `$130`-adjacent in spirit):
  when `F>0`, a marking run is subdivided into points spaced for
  constant-velocity timing and submitted via a new "paced" stream mode
  (`dac_task.c` writes points one at a time with a real per-point delay,
  instead of the fast quad-SPI burst). **This is constant-velocity only -
  no acceleration/deceleration ramp and no junction-deviation cornering
  logic** (both real GRBL features) - a deliberately scoped subset,
  documented as a known remaining gap below. `F=0` (default) skips pacing
  entirely and uses the fast burst path, identical to prior behavior.
  Arcs and hatch fill remain unpaced/fast regardless of `F` this round.
- **One unified pipeline, not four bolt-ons**: wobble, feed-pacing, and
  the skywriting main-run all go through one function (`submit_run()` in
  `grbl_task.c`), which builds one mm-space point array and converts it
  through the same `laser_ctrl_mm_to_volts()` (radial + per-axis
  correction) used everywhere else. `mark_stroke()` wraps that with the
  engrave on/off bracketing and optional skywriting lead-in/out. Arcs
  (`apply_arc()`) intentionally do *not* go through this pipeline this
  round - they stay on the pre-existing fast/unpaced streamed path
  (no wobble/skywrite/pacing for arcs yet - a known gap, see below).
- **Repeat count** (`R` word on a `G0`/`G1` line - repurposes the unused
  arc-radius letter, since only I/J-form arcs are supported): re-marks
  the same segment `R` times before advancing to the next line. The
  S-word/power-settle gate (`gate_on_power`) only applies to the first
  repetition; subsequent repeats assume power is already settled.
- **Red-light guide preview** (`M66` on / `M67` off): forces the marking
  laser off and the guide laser on for all motion (plain moves, marking
  runs, arcs, fills) regardless of `M3`/`M4` intent, so an operator can
  trace a full job's outline before it actually fires. Implemented as a
  single `s_preview_mode` flag consulted by
  `update_engrave_from_laser_mode()` and the G0/G1 dispatch in
  `process_gcode_line()`.
- **Hatch fill** (`M64` start / `M65` end region, `src/laser_fill.{h,c}`):
  every `G0`/`G1 X.. Y..` line between `M64` and `M65` is recorded as a
  polygon vertex instead of moving; `M65` traces the outline (as ordinary
  `mark_stroke()` runs, so it gets wobble/skywrite/pacing too) and fills
  the interior with hatch lines at `$170` spacing / `$171` angle, via a
  standard even-odd scanline polygon-fill algorithm
  (`laser_fill_generate()`). **Known limitations**: single
  non-self-intersecting contour only (no holes/multiple islands - a
  letter "O"'s inner ring isn't supported), and arcs are rejected while
  recording (`G2`/`G3` between `M64`/`M65` logs a warning and is ignored -
  only straight polygon edges can be recorded this round).

## On-device shape preview loop (`M68`)

Added while doing interactive real-hardware testing (`tools/jog.py`,
`tools/square_loop.py`): the simple one-line-per-point serial protocol
tops out around 5 moves/sec in practice (matches the "Serial bandwidth"
section below), nowhere near fast enough to watch a shape "live" with
the guide laser. `M68 P<hz>` (default 100Hz) is a narrow, immediately-
useful fix for exactly that: it takes the polygon recorded via `M64`
(without needing `M65`) and pre-builds the WHOLE repeated-outline path -
up to `PREVIEW_MAX_POINTS` (20000) points - as one burst, then submits it
through the existing paced-stream mechanism (added earlier this session
for feed-rate pacing) so the repeat loop runs entirely on-device with no
further serial traffic at all.

**Verified on real hardware**: recorded a 2mm square (`M64` + four `G0`
corners), then `M68 P100` logged `looping a 5-point shape 4000 times at
100.0Hz (20000 total points, ~40s)` - confirming the on-device loop
actually engaged and its timing math is correct (4000 reps x 5 points
= 20000, and 20000 points / 100Hz / 5 points-per-rep = 40s).

**Bug found and fixed while building this**: the first version allocated
a second pair of buffers to hold the volts-converted output alongside
the original mm-space buffers, hitting ~320KB of simultaneous allocation
against this board's ~300KB free heap at `PREVIEW_MAX_POINTS` scale -
failed with "out of memory for volts buffers" on real hardware. Fixed by
converting mm-to-volts in place (`laser_ctrl_mm_to_volts(xs[i], ys[i],
&xs[i], &ys[i])` - safe since the mm inputs are passed by value before
the function writes through the output pointers), halving peak memory.

**Known limitation**: bounded to one ~20000-point burst per `M68` call -
for a small shape at a reasonable Hz that's tens of seconds, but a
genuinely infinite/indefinite loop would need either a much larger
buffer or a repeat-refill mechanism, neither implemented this round;
call `M68` again once a burst finishes.

**Corner hotspots/wonky edges - found and fixed via direct visual
inspection on real hardware**: the first version jumped straight from
recorded vertex to recorded vertex with no intermediate points. A single
instantaneous voltage step between corners exceeds the galvo's
mechanical slew rate, causing visible overshoot/ringing ("hotspots") at
each corner and a non-straight ("wonky") path along what should be a
straight edge - a real physical effect, not a G-code/timing bug. Fixed
by linearly subdividing each recorded edge into a live-settable number of
intermediate points (`$190`, default 8 - see calib.h) before repeating
the template, so the galvo ramps smoothly along each edge instead of
snapping corner-to-corner. Verified on real hardware: `M68 P50` on a 2mm
square initially logged `looping a 32-point (4 edges x 8 subdivisions)
shape 625 times at 50.0Hz` (matching 4 corners x 8 subdivisions).

**Follow-up found on real hardware**: 8 subdivisions fixed the corner-
to-corner snapping, but each of the 8 *intermediate* points then became
its own visible dwell spot ("8 hotspots per edge") - the same underlying
issue at a finer scale: with a fixed per-edge time budget, fewer/coarser
points means each one gets a longer dwell, which is what makes it look
like a static dot rather than continuous motion. The fix is more
(closer-together) points, which - for the same overall Hz - shrinks each
point's dwell time automatically. Made `$190` live-settable specifically
so the right density for a given galvo's mechanical response can be
found without a reflash each time; confirmed on real hardware with
`$190=32` producing a 128-point shape (4 edges x 32 subdivisions).

**Broader fix in the same real-hardware testing session**: `G0` rapids
and any other non-marking "jump" move (`submit_plain_move()`) had the
identical problem - an instantaneous single-point voltage step with no
speed concept at all, "however fast the DAC can step." Fixed by routing
plain moves through `submit_run()` (the same interpolation/constant-
velocity pacing marking runs already use) at `$182`'s max velocity,
instead of a one-point jump. Verified on real hardware: a `G0 X3 Y3`
diagonal move now genuinely interpolates (213 points at ~99us/point =
~21.1ms total, matching hand-calculated travel time for that distance
at the default 200mm/s). If `$182`'s default speed still produces
visible hotspots on a given galvo, lower it - as the user who found this
put it, "I'd rather have a slow laser than one full of hotspots."

**`M68` no longer discards its recording, and a real watchdog-starvation
bug was found/fixed while testing continuous looping**: `M68` used to
call the same `fill_reset()` M65 uses after a one-shot fill, meaning a
second `M68` call always failed with "no recorded shape" - wrong for a
command whose whole purpose is being called repeatedly. Fixed by not
resetting the recording on `M68` (only a fresh `M64` or Ctrl-X clears
it). Testing that fix then surfaced a second, more serious bug on real
hardware: `dac_task`'s paced-stream loop is a pure `esp_rom_delay_us()`
busy-wait with no scheduler yield at all - fine for short bursts, but a
~20000-point/~3s burst was long enough to starve the idle task and trip
its watchdog (`task_wdt` panic, `IDLE0` on CPU0). Fixed by inserting a
real `vTaskDelay(1)` yield every 256 points in that loop, trading a
small amount of pacing jitter (documented as acceptable for this
preview/feed-pacing path, not marking-quality-critical timing) for
never starving the watchdog. `tools/square_loop.py` was updated to loop
indefinitely by default (previously stopped after one burst) and to
wait generously longer than each burst's logged duration before
re-issuing `M68`, since two ~160KB point buffers coexisting (one still
draining, one newly allocated) can itself exceed this board's ~300KB
heap - found the same way, via a real "out of memory" failure on
hardware when re-issuing too early.

### What's still not covered (honest gaps after this batch)

- **Arcs don't go through the wobble/skywrite/pacing/motion-planner
  pipeline** - only straight `G1` marking runs (and hatch-fill lines, via
  the same `mark_stroke()` path) do.
- **No look-ahead/character-counting serial streaming protocol** - still
  the simple one-line-wait-for-"ok" protocol (see "Serial bandwidth"
  below, unchanged this session).
- **Hatch fill**: single contour only, no self-intersection handling, no
  per-layer power/frequency/repeat distinct from the rest of the job
  (real BJJCZ-style "layers" with independent pen parameters aren't
  implemented - `S`/`F`/wobble/skywrite settings are global, not
  per-shape).
- **F-theta correction is a single global k1, not a calibration grid** -
  won't correct asymmetric or higher-order lens distortion.

## Real look-ahead motion planner (`src/motion_planner.{h,c}`)

Closes what was the largest remaining gap versus a real GRBL/EZCAD-class
controller: plain `G1` marking moves (not repeat-count marks, arcs, or
hatch fill - see below) now go through an actual trapezoidal-velocity,
junction-deviation-aware planner instead of constant-velocity feed
pacing. New settings: `$180` max path acceleration (mm/s²), `$181`
junction deviation (mm, GRBL-style cornering tolerance), `$182` max
velocity (mm/s, hard cap regardless of `F`).

**Verified on real hardware**: two connected `G1` lines (a straight run
then a 90° corner) were buffered with no execution/log output at all
until a subsequent `G0` triggered a flush, which then executed both as
"paced" time-sampled runs (414 points each at ~498us/point ≈ 206ms per
2mm segment at `F600` - matching the expected accel-to-10mm/s-then-cruise
time by hand calculation). Repeat-count (`R3`) marks executed immediately
(three separate "paced run" logs, bypassing the planner as designed), and
a `G2` arc used the pre-existing fast unpaced "streamed" burst path (also
bypassing the planner) - confirming the scope boundaries hold in practice,
not just in code.

**Design, stated honestly (see `motion_planner.h`'s header comment for
the full version):**
- **Batch look-ahead, not a continuously sliding window like real GRBL.**
  Buffers up to 16 segments; on overflow or an explicit flush, runs one
  reverse pass (junction-deviation cornering speed + decel feasibility)
  then emits all buffered segments with their own trapezoid (entry/
  cruise/exit speed). A *soft* flush (buffer just filled, more marking
  still coming) carries the batch's real exit speed forward as the next
  batch's entry anchor - no unnecessary stop there. A *hard* flush
  (anything that isn't a continuation of marking motion: `G0`, arcs,
  fill, dwell, program end, mode changes, repeat-count marks) always
  decelerates fully to a stop, which is the physically correct behavior
  in those cases. The cost of the batch approach vs. a sliding window:
  a segment near the tail of a soft-flushed batch can't see into the next
  batch's shape yet, so cornering exactly at that 16-segment boundary can
  be slightly more conservative than optimal - never unsafe, just
  occasionally not maximally fast, and only relevant for very long runs
  of connected marking `G1` lines.
- **Junction-deviation formula independently derived** (see
  `motion_planner.c`'s comment), not a byte-for-byte port of GRBL's
  `planner.c` - verified by hand for both edge cases (straight-through =
  no cornering limit; full 180° reversal = forced stop) before trusting
  it, and confirmed behaving sensibly on hardware (see above).
- **Trapezoid shape is an approximation**, not an exact jerk-limited
  S-curve: uses the standard `v²=v0²+2ad` kinematics, and a segment too
  short to reach its computed peak speed degrades to a single scaled
  accel-or-decel ramp rather than a true optimal-control profile.
- **Position sampling, not variable per-point delays**: each finalized
  segment is time-sampled at a fixed interval (position computed from the
  accel/cruise/decel kinematics, not linear interpolation), so a single
  `point_delay_us` on the existing paced-stream path (`dac_task.c`, added
  earlier this session for constant-feed pacing) is all that's needed to
  realize the non-constant velocity - no dac_task changes were required
  for this feature.
- **Skywriting only at true sequence boundaries**, not per-block: lead-in
  on the very first segment since the last hard flush, lead-out on the
  last segment of a hard flush - not on every segment, since consecutive
  planned segments already maintain continuous velocity through their
  shared junction and don't need a laser-off gap between them (a
  meaningful behavior change from the earlier per-stroke skywriting
  described above, which still applies to repeat-count marks, hatch fill,
  and any other caller of `mark_stroke()` directly).
- **A known, unresolved edge case**: a buffered batch only executes when
  something calls `motion_planner_flush()` - if a job's very last line is
  a plain `G1` mark with no `M2`/`M30`/other flush-triggering line after
  it, and the sender simply stops sending, that last batch could sit
  unexecuted indefinitely. Every normal job ending (`M2`/`M30`, a `G0`
  return-to-origin, a mode change) already triggers a flush, so this is
  unlikely to bite in practice, but it's a real gap: fixing it properly
  would need `grbl_task`'s blocking `fgetc()` read loop restructured
  around a timeout instead, which wasn't attempted this session.

## ESP32-S3 "DSP": is it usable for the geometry math?

Correction on the premise first: the S3 doesn't have a separate DSP
core. It has SIMD/vector *instruction extensions* on its two Xtensa LX7
cores (exposed via the `esp-dsp` component - FFT, FIR/biquad filters,
dot products, mainly aimed at audio/ML workloads), not a distinct DSP
chip. Given the actual profiling done this session, using it for arc
interpolation (`atan2f`/`acosf`/`sinf`/`cosf` in `grbl_task.c`'s
`apply_arc()`) wouldn't move the needle: those are called once or twice
per *arc command*, not per point, and the real bottlenecks turned out
to be `double`-vs-`float` FPU mismatch and SPI protocol overhead (both
fixed - see Performance below), not trig cost. `esp-dsp` would be worth
revisiting if bitmap/raster image preprocessing (halftoning, FFT-based
dithering) becomes a feature, since that's the bulk-array workload it's
actually built for - not for the current point-at-a-time G-code path.

## Serial bandwidth: is it enough to drive the galvo correctly?

**No - not for genuinely one-point-per-line G-code, and this is now
measured, not assumed.** A synthetic Python/pyserial sender doing the
simplest possible synchronous protocol (send a line, block until "ok",
send the next) achieved **~17.4 lines/sec** against the real board
(200/200 lines acknowledged correctly - the firmware side is working,
it's just not fast in this usage pattern). That is dramatically below
the DAC's ~0.96 MUPS - proving the concern in the question is correct:
**for a naive per-pixel raster stream (one `G1 X.. Y.. S..` line per
pixel, waiting for each `ok`), the serial round-trip is the bottleneck by
several orders of magnitude, not the DAC.**

Important caveats on that number:
- It's almost certainly dominated by **Python/pyserial overhead**
  (small-chunk polling reads, interpreter/GIL cost), not USB-CDC or this
  firmware's own processing - the boot-time diagnostics already show
  this firmware processes a queued command in well under a millisecond.
  A real sender written with efficient buffered I/O (e.g. rayforge, or
  any compiled GRBL sender) would very likely do much better than 17/sec,
  but *how much* better hasn't been measured - a repeat test with a
  tighter native (non-Python) test harness, or instrumenting a real
  sender, would give a truer number.
- What's confirmed to be working *within this firmware's control*:
  `grbl_task`'s "ok" is issued as soon as a command is accepted into the
  pipeline (enqueued to `dac_task`, or - for `S` - confirmed applied on
  the ATMEGA), not after the DAC has physically executed it, matching
  real GRBL's planner-buffer behavior. The bottleneck is the per-line
  synchronous wire protocol itself (this firmware implements the
  simplest GRBL protocol - one line, wait for "ok" - not GRBL's
  character-counting streaming protocol that keeps multiple lines in
  flight), not anything slow on the DAC/firmware side.
- **Already correctly avoided for arcs and raster-lookahead paths that
  matter most**: an entire `G2`/`G3` arc is interpolated and streamed as
  one batch (`ad3552r_board_stream_xy()`), paying the serial round-trip
  exactly once per arc command, not once per interpolated point. The
  17/sec ceiling only applies to G-code that is genuinely one point per
  line - a naive per-pixel bitmap raster stream sent that way, which is
  a real and likely scenario for dithered engraving.

**Recommendation for actual raster/dithered engraving jobs**: don't send
one `G1` line per pixel over the simple synchronous protocol. Either (a)
verify/implement GRBL's character-counting streaming protocol so a
well-behaved sender can keep several lines in flight without waiting for
each "ok" (this firmware doesn't currently support that - it's a real
gap for high point-rate raster jobs), or (b) add a custom batch/binary
command (mirroring how `ad3552r_board_stream_xy()` solved this on the
DAC side) that sends a whole raster row's worth of X/S pairs in one
message. Neither is implemented this session; (a) is the more
GRBL-compatible fix and the natural next step.

## Performance (DAC write throughput)

**Current: ~0.96 MUPS sustained, verified correct via register readback**
(steady-state buffered streaming, quad SPI @ 60MHz) - up from an initial
~0.038 MUPS, a ~25x improvement, entirely from real measurement-driven
fixes rather than guesswork. Two speed tests run at every boot
(`dac_task.c`), both on real hardware:

1. **Per-transaction** (`ad3552r_board_write_volts()`, one SPI burst per
   single-channel write): ~209k updates/sec (~0.21 MUPS), single-lane SPI.
2. **Buffered streaming** (`ad3552r_board_stream_xy()`, quad SPI @ 60MHz,
   using the AD3552R's `STREAM_MODE` register to auto-loop its internal
   register address across one continuous burst covering many X+Y
   points): **~962k X+Y point-pairs/sec (~0.96 MUPS)**, steady-state
   (warmup call excluded from timing - see below).

### What actually moved the number, in the order found

1. **Root-caused via instrumentation, not guesswork.** Added stage
   timestamps around the SPI transaction and the buffer-build loop
   (`no_os_spi_esp32.c`'s transfer profiling, `ad3552r_board.c`'s build
   timing) rather than assuming where time went.
2. **Critical bug: `double` on a single-precision-only FPU.**
   `volts_to_code()` used `double` math. ESP32-S3's hardware FPU is
   single-precision only; `double` silently falls back to slow software
   emulation. This was ~11.8us/call (10000 calls for a 5000-point
   buffer = ~118ms) - by far the single biggest cost in the whole
   pipeline. Switching to `float` cut buffer-build time from an
   estimated ~118ms to ~20ms outright, and *also* nearly 3x'd the
   per-transaction test (which uses the same function).
3. **Hoisted the per-channel scale/offset lookup out of the per-point
   loop** (it's a per-channel constant, not per-point) and replaced
   division-per-point with a precomputed reciprocal. Cut buffer-build
   from ~4.14us/point to ~0.63us/point (5000 points: 20.7ms -> 3.15ms).
4. **Enabled genuine quad SPI for the streaming burst.** This was the
   biggest single lever, and the one initially missing entirely: quad
   mode is a *hardware* mode on the AD3552R (its `QSPI` pin, not a
   register bit - datasheet Table 14), and the code had built all the
   ESP-IDF-side plumbing (`SPI_TRANS_MODE_QIO`, `active_lines`) back in
   an earlier session but never actually drove that physical pin, so it
   was never engaged. Toggling it on for just the streaming burst (see
   `set_qspi_pin()` in `ad3552r_board.c`) and raising the clock to 60MHz
   (both confirmed clean via register readback, not assumed safe) took
   the raw SPI bulk-transfer time for a 5000-point (30001-byte) burst
   from 12.0ms (single-lane @20MHz) down to 1.5ms.
5. **Measured the true steady-state number**, not a number contaminated
   by one-time diagnostic logging. The stage-profiling `ESP_LOGI` calls
   added in step 1 are themselves several hundred microseconds each of
   USB-CDC output, and they fire *inside* the very code path being timed
   on the first call. The boot-time speed test now does one unmeasured
   "warmup" streaming call (letting the one-time diagnostic logging
   fire and settle) before timing a second, otherwise-identical call for
   the real number - this is what surfaced ~0.96 MUPS instead of the
   ~0.68-0.70 MUPS an unadjusted measurement would have shown.

### Why not exactly the datasheet's 33 MUPS

That figure is for the AD3552R's quad-SPI **+ DDR** (double data rate,
data on both clock edges) hardware streaming mode. This firmware uses
quad SPI but not DDR: DDR is not supported by ESP-IDF's generic
`spi_master` driver for arbitrary GPIO-routed half-duplex transfers (it's
normally a flash/PSRAM-specific hardware timing mode). Adding it would
require bypassing to raw SPI-peripheral register-level HAL code - a
substantially larger and riskier undertaking than what's been done here,
and not attempted this session. Quad-without-DDR alone already closed
almost the entire gap to 1 MUPS, though.

`ad3552r_board_stream_xy()` is wired into `grbl_task.c`'s G2/G3 arc
interpolation (a whole arc's segments are batched into one streamed burst
instead of one queued move per segment) - a real user of the fast path,
not just a synthetic benchmark; confirmed working end-to-end (`G2 X10 Y0
I5 J0` streams 55 points and lands exactly on `MPos:10.000,0.000`).

## Known placeholders / needs real-world calibration or wiring info

- **Engrave GPIO pin** (`src/engrave_gpio.c`, `BOARD_PIN_ENGRAVE = 4`) and
  **PRR PWM pin** (`src/prr_pwm.c`, `BOARD_PIN_PRR = 3`): unconnected
  placeholders, not yet confirmed against the actual board wiring to the
  laser driver.
- **S → power % mapping** (`src/grbl_task.c`, linear 0-1000 → 0-100%) and
  **Q → PRR Hz mapping** (`PRR_MAX_HZ` = 50kHz): guesses, need the real
  laser driver's specs.
- **No velocity/trajectory planning**: `G1` moves jump the DAC output
  immediately to the target voltage; `F` (feedrate) is parsed but not
  acted on. Arc segments (`G2`/`G3`) are similarly enqueued/streamed
  back-to-back with no feedrate-based timing between them.
- **CRC disabled** on the AD3552R SPI link (`ad3552r_board.c`): CRC-framed
  transfers need genuine simultaneous tx+rx (full duplex), which conflicts
  with this board's shared-direction level-shifter (one DIR line for
  D0-D3). A second full-duplex SPI device was tried and rejected (it
  measurably broke the working half-duplex device just by existing on the
  bus). Low risk for galvo control at these distances/speeds, but revisit
  if wire-integrity checking becomes a requirement.
- **Quad SPI is now used and verified** for buffered streaming writes
  (see Performance above) - `set_qspi_pin()` in `ad3552r_board.c` drives
  the AD3552R's hardware `QSPI` pin, toggled on only for the streaming
  burst and back off after. `ad3552r_board_set_quad()` (a separate,
  older function for a persistent quad-mode switch rather than a
  per-burst one) is still unused/unverified and does NOT drive that pin
  - low priority to fix given the streaming path already covers the
  main use case.
- **DAC output voltage never measured with a meter/scope** - only
  confirmed via register readback (now proven necessary - see the bug
  above, which register readback caught and a "no SPI error" check would
  not have). Recommend verifying actual analog output with a meter before
  connecting to a real galvo.
- **ATMEGA link now verified against real hardware** (see above) - the
  true end-to-end pixel-to-pixel latency of the POWER round-trip still
  hasn't been measured, though (the 300ms figure remains a ceiling, not
  a measurement).

## How to test

```
cd WaveMaster
pio run -t upload -t monitor
```

Then, from another terminal or a GRBL sender (e.g. a serial terminal, or
`screen /dev/cu.usbmodemXXXX 115200`), send G-code lines.

## Architecture reference

- `src/laser_ctrl.{h,c}` - shared command queue (moves, engrave, PRR, and
  batched stream commands) + mm→volts conversion (via `calib.h`).
- `src/calib.{h,c}` - persistent per-axis calibration (NVS-backed),
  including the marking-quality parameters (delays, radial-correction k1,
  wobble, skywrite margin, hatch spacing/angle - see above).
- `src/dac_task.{h,c}` - core-0 task: AD3552R writes (including streaming
  and feed-paced point-at-a-time writes), Engrave GPIO (with laser-on/off
  delay), jump/mark/polygon delay, PRR PWM, boot-time speed tests.
- `src/grbl_task.{h,c}` - core-1 task: serial I/O + G-code parsing
  (motion, arcs, laser-mode semantics, ATMEGA-bound M-codes, the
  wobble/skywrite/feed-pacing stroke pipeline, hatch-fill region
  recording, preview mode).
- `src/laser_fill.{h,c}` - pure 2D geometry: even-odd scanline hatch-fill
  generation for a closed polygon (no DAC/volts/queue knowledge - used by
  `grbl_task.c`'s `M64`/`M65` handling).
- `src/motion_planner.{h,c}` - pure kinematics: batch look-ahead planning
  (trapezoidal velocity profiles, junction-deviation cornering) for plain
  `G1` marking runs (no DAC/volts/queue knowledge either - calls back
  into `grbl_task.c`'s `motion_block_execute_cb()` per finalized segment).
- `src/atmega_link.{h,c}` - binary protocol client for the ATmega328P
  board (guide laser, power, arming).
- `src/ad3552r_board.{h,c}` - board bring-up (pins, init, quad-switch,
  volts→code conversion, single-point and streamed-burst writes).
- `src/engrave_gpio.{h,c}`, `src/prr_pwm.{h,c}` - the two extra output
  signals requested (engrave on/off, pulse-repetition-rate PWM).
- `src/spi_probe.{h,c}` - low-level raw-SPI diagnostic tool, not wired
  into `main.c` by default; useful for future hardware bring-up issues.
- `components/ad3552r/` - the ported no-OS AD3552R driver + ESP-IDF HAL
  shim (SPI/GPIO/CRC8), see `docs/superpowers/specs/` and
  `docs/superpowers/plans/` for the original design/plan documents.
- `../arduino-laser-control/` - the ATmega328P firmware `atmega_link.c`
  talks to; read its `src/main.cpp` for the authoritative wire protocol.
