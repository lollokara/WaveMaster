# rftest - Rayforge-style test suite for WaveMaster

Drives the controller the way Rayforge does (GRBL 1.1 over the native USB port, character
counting, Rayforge's G-code dialects), logs every byte, and runs shapes to test the laser:
some with the red guide laser only, some with the real laser at chosen powers.

Python 3.10+, only dependency is `pyserial`. Nothing here touches the firmware sources;
`tools/jog.py` stays as the interactive jog tool, and this suite supersedes
`tools/stream_test.py`.

## Setup

```bash
pip install pyserial
python3 -m serial.tools.list_ports -v     # find the port
```

The port is the ESP32-S3's **native USB** (USB-Serial-JTAG), e.g. `/dev/ttyACM0` (Linux),
`/dev/cu.usbmodem*` (macOS), `COM5` (Windows). Not the CH340 on the ATmega board. The baud rate
is irrelevant (default 115200). Close Rayforge and any serial monitor first.

## Quick start

```bash
python3 tools/rftest/selftest.py                          # offline self-test, no hardware
python3 tools/rftest/run_tests.py --mock smoke            # offline end-to-end run
python3 tools/rftest/run_tests.py /dev/ttyACM0 smoke      # first contact, laser stays off
python3 tools/rftest/run_tests.py /dev/ttyACM0 all-guide  # all guide-laser plans
```

## Plans

| Plan | What it does |
|:---|:---|
| `smoke` | Connect, `$I`, `$$`, `$S`, `$RB`, one tiny guide square, then a cancel test: a long guide job is started, 0x18 is sent after 1 s, the device must be Idle within 2 s, accept `M5`/`M9` afterwards, and the guide must be off. |
| `guide-shapes` | square, circle (64-segment polyline), circle (G2 arcs), star, spiral, target (crosshair + rings), corner test (V corners 15..170 degrees). |
| `guide-frame` | Frame of the job box (`--size`), looped `--frame-loops` times. |
| `guide-raster` | Gradient and Bayer-dither rasters with overscan, bidirectional, `grbl_raster` dialect. Exercises the stream without firing. |
| `guide-stress` | 2000-segment circle and a ~3000-point spiral: lines/s, bytes/s, underruns. |
| `all-guide` | The four guide plans in a row. |
| `fire-power-ladder` | Hatched patches at `--powers` (default 100,200,300,500,750,1000, each capped by `--max-power`, duplicates dropped). Patch order: left to right, then the next row up (+Y). The order is the label. |
| `fire-speed-ladder` | The same square (outline + hatch) at each `--feeds` F, fixed `--power`. Same patch order. |
| `fire-shapes` | The guide-shapes set at one power. |
| `fire-raster` | Dither, gradient and checkerboard rasters. |
| `fire-delay-grid` | Rows of short marks (lengths grow left to right) for tuning `$210`/`$211`/`$212`. |

Guide plans wrap every job in `M66` ... `M67` (guide on, marking laser forced off by the firmware).
The runner sends `M66` as a command, checks `$S` shows `guide=1`, streams the job, waits for Idle, then sends `M67`.
The jobs still contain `M4 S<power>` lines, which is the point: the stream is the real one, and the firmware must not fire.

Common options (see `--help`): `--dialect grbl|grbl_raster` (default `grbl`, `grbl_raster` for raster plans),
`--feed 30000` (mm/min), `--power 100`, `--max-power 300`, `--work-area auto|WxH`, `--size`, `--origin`,
`--poll-ms 250` (status polling during jobs; `0` = exactly like Rayforge, which does not poll), `--repeat N`,
`--out DIR` (default `tools/rftest/runs/<timestamp>_<plan>/`), `--dry-run` (only write the `.gcode` files),
`--no-arcs`, `--modal-feed`, `--verbose` (echo all traffic), `--stall-timeout`, `--idle-timeout`.

### Work area and pattern placement

- `--work-area` defaults to `auto`: after the handshake the runner reads `$130`/`$131`/`$144` from `$$` and uses them
  (`--dry-run`, which has no device, assumes 100x100 with the origin in a corner). The mock reports what
  `--mock-work-area WxH` / `--mock-centered` say (default 100x100).
- `$144=1` (centred origin, coordinates `-W/2..W/2`) is supported: patterns are placed around (0,0) and the coordinate
  clamp uses the centred range. With an explicit `--work-area` and a centred device, add `--centered`.
- Default `--size` is `min(50, 0.8 * min(W, H))` (rounded down to 0.1 mm); default origin puts the pattern box
  centred on the work-area centre (the lens centre). So `fire-shapes --fire` fits any work area, e.g. 51.7x51.7 gives
  a 41.3 mm pattern at 5.2,5.2. `--origin x,y` is the lower-left corner of the box in machine coordinates.
- The box is validated once the work area is known; if it does not fit you get an error with a `--size` that fits.
- An explicit `--work-area` must equal the device's; on a mismatch fire plans stop with a SAFETY STOP and the message
  says to omit `--work-area` to adopt the device's values.

## Safety procedure for fire plans

1. Read [docs/WIRING.md](../../docs/WIRING.md). Wire a real interlock on DB25 pin 23. Closed enclosure, eye protection, test material in place.
2. Run `all-guide` and `smoke` first. They must pass.
3. Do a dry run and look at the files: `run_tests.py PORT fire-shapes --dry-run`.
4. Start low: `run_tests.py PORT fire-power-ladder --fire --powers 20,40,60,80 --max-power 100`.
5. The runner then:
   - refuses without `--fire`;
   - prints the plan (jobs, the S values actually sent after capping, F values, bounding box) and requires you to type `FIRE` (skippable only with `--yes`);
   - adopts `$130`/`$131`/`$144` from the device (`--work-area auto`) or, for an explicit `--work-area`, checks them and aborts on a mismatch;
   - runs a guide-laser frame of the job's bounding box (`M66` ... `M67`, S0) and asks `frame OK? [y/N]`;
   - sends `M10`, waits for Idle and for `armed=1` in `$S`;
   - runs the jobs, stops at the first failed job, always sends `M5` and (unless `--keep-armed`) `M11` at the end.
6. Ctrl-C or any exception during a plan sends the cancel sequence (0x18, 0.2 s, `M5`, `M9`) and then `M11`.
7. No `S` above `--max-power` is ever generated or sent (the encoder caps it, and every line is re-checked before it goes out). All coordinates are clamped to the work area; a job that needed clamping fails.

## Output of a run

```
tools/rftest/runs/<timestamp>_<plan>/
  NN_<job>.gcode            exactly the lines streamed (comments stripped, M66/M67 are sent separately)
  NN_<job>.rayforge.gcode   the generated text with comments, as Rayforge would write it
  session.log               every TX/RX with monotonic timestamps; realtime bytes shown as <0x18>, ?, !, ~
  events.jsonl              tx_line, rx_line, ack, error, status, msg, realtime, note
  firmware_msgs.log         all [MSG:...] lines
  statuses.csv              t,state,x,y,feed,spindle,bf_blocks,bf_rx
  report.md / report.json
```

Reading `report.md`:

- **lines/s, B/s**: stream throughput. Compare plans: many tiny segments (`guide-stress`) show the host/USB/parse limit.
- **max in-flight**: the character-counting window use. It should reach most of the RX window (1024) on long jobs; if it stays small, the device acks quickly.
- **ack->Idle**: time from the last `ok` to the first `Idle` status. This is how long the motion queue still had work after the host finished sending.
- **underruns / barriers / chunks**: deltas of `$S` over the job. Underruns must be 0. Barriers are power and PRR changes (grayscale raster causes many unless `$223=1`).
- **ATmega link**: `link=1` after the job, else the job fails.
- **transitions** (report.json): the status state changes seen while polling, e.g. Idle, Run, Idle.
- **cancel** (smoke): banner latency after 0x18, time to the first Idle, replies to `M5`/`M9`, guide state afterwards. The firmware target is about 60 ms (STATUS.md, not yet measured on hardware).
- **Checks / reasons**: why a job failed (line errors, abort, no Idle, underruns, firmware clamped targets, ATmega link down, guide not on after `M66`).

## What to look for when tuning

See [docs/RAYFORGE.md](../../docs/RAYFORGE.md) section 8 for the procedure and
[STATUS.md](../../STATUS.md) for the bring-up checklist.

- `fire-delay-grid`: dots at the start of marks mean `$210` (laser-on delay) is too short, gaps mean too long. Tails at the end mean `$211` is too long. A hook at the start after a jump means the galvo has not settled: raise `$212`. Run it at the speeds you actually use.
- `guide-shapes` corner test and `fire-shapes` corner test: burnt or hooked corners point to `$215`/`$216`.
- `fire-speed-ladder`: shows the speed range and whether corners stay sharp at high F.
- `fire-power-ladder`: the first visible patch gives the threshold power; compare against `$224`/`$225`.
- `fire-raster`: dither has no ATmega traffic; the gradient stalls ~1 ms per power change (`barriers` in `$S`); try `$223=1` (pulse density, `$229` period).
- `smoke` and the sizes of a printed square: scale `$100`/`$101`.
- Underruns during `guide-raster`/`guide-stress`: the host or the parser cannot keep the segment queue filled; check lines/s and `ack->Idle`.

## Offline mode (`--mock`)

`mock_grbl.py` is a fake device on a pty: banner, `?` status (Idle/Run with interpolated MPos, Bf), `$I` (`[OPT:V,512,1024]`),
`$$`, `$S`, `$RB`, G0-G4/M-codes (unknown gives `error:20`), simulated motion time (jump 3000 mm/s, F for marks), a real
1024-byte RX buffer, a 512-segment queue that withholds `ok` when full, `0x18`, `!`, `~`. It records protocol violations
(`OVERFLOW`, `LONG_LINE`, `OK_COUNT_MISMATCH`) which fail a run.

```bash
python3 tools/rftest/mock_grbl.py --speedup 10 --slow-ms 2 --error-on 'G2'   # prints /dev/pts/N
python3 tools/rftest/run_tests.py /dev/pts/N guide-shapes
python3 tools/rftest/run_tests.py --mock fire-raster --fire --yes --mock-speedup 50
```

`--mock-speedup N` runs the simulated motion N times faster, `--mock-slow-ms`, `--mock-error-on REGEX` inject slowness and errors.
`selftest.py` covers G-code golden tests and the mock runs (smoke, guide plans, cancel, fire gates, error injection, Ctrl-C).

## Files

| File | Role |
|:---|:---|
| `rfclient.py` | `RayforgeClient` (handshake, character-counting `stream`, status parsing and polling, `command`, `cancel`, `hold`/`resume`, `jog`/`move_to`) and `SessionLogger`. |
| `gcode_gen.py` | Op list, `grbl` and `grbl_raster` encoders, raster run-length linearisation, shape library. |
| `mock_grbl.py` | Fake device. |
| `run_tests.py` | CLI runner and reports. |
| `selftest.py` | Offline self-test. |

## Rayforge fidelity: assumptions

- Rayforge's driver reads the RX window from `[OPT:..,..,<rx>]`; default 127.
- Comment stripping is `;...` first, then `(...)`, then trim. Empty lines are not sent.
- The exact column order of Rayforge's `grbl_raster` `G1` is assumed as `X Y S F`, and `F` appears only when it changes. `grbl` is `X Y F`.
- `ok` is recognised only as a standalone line. A banner (`Grbl ...`) during a job is treated as a device reset and aborts the job.
- Arcs always emit both `X` and `Y` plus `I`/`J`; other coordinates that did not change are omitted.
- Zero-length travels and zero-length cuts are dropped. Blank raster rows are skipped.
- The runner polls `?` every 250 ms by default (Rayforge does not); use `--poll-ms 0` for the Rayforge-exact traffic pattern.
