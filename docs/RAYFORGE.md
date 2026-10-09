# Rayforge setup for WaveMaster

How to configure Rayforge to drive the WaveMaster galvo
controller. The wiring is in [WIRING.md](WIRING.md). Settings are described in
[README.md](../README.md).

The ready-made files are in [`rayforge/`](rayforge/). They were checked against
the Rayforge source at commit `22dc2c6d` (cloned 2026-10-08) by loading them with
Rayforge's own model classes (see [Validation status](#validation-status)).
Rayforge's file formats change between versions. If a newer Rayforge refuses a
file, use the manual settings in sections 1 to 6 below.

---

## Quick setup with the WaveMaster profile

You need three things from [`docs/rayforge/`](rayforge/):

| File | Goes to | What it is |
|:---|:---|:---|
| `wavemaster-machine.yaml` | `<config>/machines/wavemaster.yaml` | The machine: GRBL serial driver, 100 x 100 mm, Bottom Left origin, fiber laser head (max power 1000), no Z, arcs on (0.01 mm), no homing, 8 macros. |
| `wavemaster-dialect.yaml` | `<config>/dialects/wavemaster.yaml` | The G-code dialect "WaveMaster Galvo". |
| `wavemaster.rfdevice/` | (optional) zipped, imported in the wizard | The same machine and dialect as a Rayforge device profile. No port, no macros. |

`<config>` is Rayforge's config directory:

| OS | `<config>` |
|:---|:---|
| macOS | `~/Library/Application Support/rayforge` |
| Linux | `~/.config/rayforge` |
| any | the value of `RAYFORGE_CONFIG_DIR`, if set |

(`platformdirs.user_config_dir("rayforge")`, `rayforge/config.py`.)

### macOS, step by step

1. **Quit Rayforge.** It rewrites its machine file whenever a setting changes, so a copy made while it runs can be overwritten.
2. Copy the files (run this from the root of the WaveMaster repository). The target names matter: a machine's id is its file name, and a dialect is stored as `<uid>.yaml`, so the dialect file must be called `wavemaster.yaml`.
   ```bash
   CFG="$HOME/Library/Application Support/rayforge"
   mkdir -p "$CFG/machines" "$CFG/dialects"
   cp docs/rayforge/wavemaster-machine.yaml "$CFG/machines/wavemaster.yaml"
   cp docs/rayforge/wavemaster-dialect.yaml "$CFG/dialects/wavemaster.yaml"
   ```
3. **Set the serial port.** Plug in the ESP32-S3 **native USB** port, then:
   ```bash
   ls /dev/cu.usbmodem*
   ```
   Put that name into `driver_args: port:` in `"$CFG/machines/wavemaster.yaml"`. The profile ships with the placeholder `/dev/cu.usbmodem1101`. The number can change when you use another USB port. Instead of a path you can write the USB id, `303a:1001`. Use the `cu.` device, not `tty.`. You can also change the port later in Rayforge: **Machine > Machine Settings** > **General** > **Driver Settings** > **Port**.
4. **Check the controller settings** the profile assumes (send `$$` from the Console, or read them in Machine Settings > **Device**): `$130=100`, `$131=100`, `$144=0`, `$30=1000`, `$32=1`. If your work area is not 100 x 100, change `axes` and `axis_extents` in the machine file (or **Hardware** > **X Extent** / **Y Extent**) to match `$130` / `$131`. Keep `$144=0`: Rayforge's work area starts at (0,0), and the firmware clamps targets to its own work area, so a centred origin (`$144=1`, coordinates -50..50) would clip everything on the negative side.
5. **Start Rayforge** and pick **WaveMaster Galvo** in the machine selector (toolbar, tooltip "Select active machine"). If it is the only machine in the folder (Rayforge never started before), it is already active, and the first-run setup wizard does not appear because a real, non-placeholder machine exists. Rayforge connects by itself (`auto_connect: true`).
6. **First connection check:**
   - The machine selector shows the machine name with the status **Idle**. If it shows "Disconnected" or an error, the port is wrong or another program (a serial monitor, `tools/*.py`) holds it. Only one program can have the port open.
   - The work surface is **100 x 100 mm**, with (0,0) at the bottom left. Machine Settings > **Hardware** shows X Extent 100, Y Extent 100, Coordinate Origin (0,0) = Bottom Left, and "Has Z-Axis" off.
   - In the bottom panel open **Console** and send `$I`. Expect `[VER:1.1h.WaveMaster:]`, `[OPT:V,512,1024]` and `[MSG:machine:WaveMaster Galvo]`.
   - Send `$$` and check `$30=1000`, `$32=1`, `$130=100`, `$131=100`, `$144=0`, `$12=0.01`.
   - Machine Settings > **G-code** > **Dialect**: the list shows **WaveMaster Galvo** with its check mark set. That is what `dialect_uid: wavemaster` selects.
   - **Machine > Macros** lists the 8 macros (below).
7. Before the first real job, click **Frame**: the guide laser traces the job's box (see "Framing and preview" below). The bring-up steps for the real laser are in [STATUS.md](../STATUS.md).

### Macros

Run them from **Machine > Macros**. Their output (for `$S`, `$RB`) appears in the **Console**.

| Macro | Sends | Use |
|:---|:---|:---|
| Preview ON (M66) | `M66` | Guide laser on, marking laser forced off, motion runs normally. |
| Preview OFF (M67) | `M67` | Preview and guide laser off. |
| Guide laser ON (M62) / OFF (M63) | `M62` / `M63` | Guide laser only. |
| Arm (M10) / Disarm (M11) | `M10` / `M11` | Arm manually. The firmware auto-arms at the first mark when `$226=1`. |
| Status ($S) | `$S` | Stream and ATmega link statistics. |
| DAC readback ($RB) | `$RB` | Reads the two DAC output registers, only while idle. |

Macros live in the machine file only. Rayforge's device profile format has no place for them, so the profile import (below) does not bring them along.

### Alternative: import the device profile in the wizard

Use this if you do not want to touch the config folder. You lose the macros and the port is entered in the wizard.

```bash
cd docs/rayforge/wavemaster.rfdevice && zip -r ../WaveMaster_Galvo.rfdevice.zip . && cd -
```

Then **Settings > Machines > Add Machine** > **Choose a Machine** > **Import from File...**, pick `WaveMaster_Galvo.rfdevice.zip` and follow the wizard (the connection step asks for the port). The wizard installs the dialect as "WaveMaster Galvo (device dialect)". Rayforge lists `.rfdevice.zip` and LightBurn `.lbdev` files. There is no UI import for a bare machine YAML or a bare dialect YAML.

### What the profile sets, and why

| Setting | Value | Why |
|:---|:---|:---|
| Driver | `GrblSerialDriver` ("GRBL (Serial)") | Character-counting GRBL streaming. |
| `rx_buffer_size_override` | 1024 | Matches `[OPT:V,512,1024]`. `0` would auto-detect the same number from `$I`. 1024 is also the largest value Rayforge accepts. |
| `poll_status_while_running`, `deadlock_detection` | off | Rayforge defaults. |
| Work area | 100 x 100 mm, origin `bottom_left` | `$144=0`: (0,0)..(W,H), Y up, X right. |
| Z axis | none | The firmware has no Z. Rayforge then never emits a Z word. |
| `home_on_start`, `single_axis_homing_enabled` | off | `$H` is accepted and does nothing. |
| `supports_arcs`, `arc_tolerance` | on, 0.01 mm | The firmware linearises `G2` / `G3` at `$12` (default 0.01). |
| `max_cut_speed` | 60000 mm/min | Estimate and UI limit. The firmware limit is `$110` (default 300000). Raise it for faster marking. |
| `max_travel_speed` | 180000 mm/min | Only used for estimates and as the default frame speed. It equals `$201` = 3000 mm/s. Jumps always run at `$201`, not at a G0 feed. |
| `acceleration` | 2000000 mm/s2 | Matches `$120`. Marks have no acceleration ramp. |
| Laser head | fiber, `max_power` 1000, spot 0.05 mm | `$30=1000`. The spot size sets the raster line pitch. Measure yours. |
| Frame | power 0 %, speed 60000 mm/min, 20 repeats | Safe by construction: the frame never marks. |
| Hooks | none | See below. |

**Job start and end.** This Rayforge version has no "job start" hook. The hooks (`hookmacros`) only exist per layer and per workpiece (`LAYER_START`, `LAYER_END`, `WORKPIECE_START`, `WORKPIECE_END`). Old job start / end hooks are migrated into the dialect's `preamble` and `postscript`, and that is where the WaveMaster profile puts them (`rayforge/machine/models/machine.py`, `_migrate_legacy_hooks_to_dialect`).

The dialect "WaveMaster Galvo" is Rayforge's `GRBL Raster` dialect (`grbl_raster`: `M4 S0` once, `S` on every move, modal `F`) with two changes:

- `can_g0_with_speed: false`. Jumps always run at `$201`, whatever F says, and an F on a G0 line still changes the modal feed rate of the next mark. So Rayforge must not send one, and it does not offer a travel speed in the step settings.
- Preamble `G21`, `G90` (Rayforge adds `G54` after it). Postscript `G4 P0`, `M5`, `M67`. `G4 P0` is a barrier: the firmware waits until its motion queue is empty (`wait_idle()` in `src/grbl.c`) before it does anything else. Current firmware already waits for the queue before `M10`/`M11`, `M62`/`M63` and `M66`/`M67` (they act on the ATmega at once, so they must not overtake queued motion); `G4 P0` keeps the profile safe with older firmware, where `M67` could switch the guide laser off in the middle of a frame. There is no `G0 X0 Y0` return to origin.

Example output (from Rayforge's own encoder, an 80 percent vector job with an arc):

```gcode
G21 ;Set units to mm
G90 ;Absolute positioning
G54
T0
G0 X10 Y10
M4 S0
G1 X50 F30000 S800
G1 Y50 S800
G0 X20 Y20 S0
G2 X30 Y30 I5 J5 S800
G4 P0 ;Wait until all queued motion is done
M5 ;Marking laser off
M67 ;Preview off (also switches the guide laser off)
```

Two things to know:

- **Do not start a real job while preview is on.** `M66` forces the marking laser off, so the job would run without marking. The postscript ends every job with `M67`, and a soft reset (Cancel Job, Ctrl-X) also clears preview. If you switched preview on and changed your mind, run "Preview OFF (M67)".
- Rayforge rewrites `machines/wavemaster.yaml` and `dialects/wavemaster.yaml` itself when you change something in the UI. Comments are lost, and the previous version is kept as `.bak` next to the file.

### Manual dialect (if you cannot use the file)

Rayforge has no dialect import. In **Machine Settings > G-code > Dialect** use **Create from Template**, pick **Grbl (Compat)** and change exactly these fields, then save and click the check mark on the new row:

| Field | Value |
|:---|:---|
| Label | `WaveMaster Galvo` |
| Continuous laser mode | on |
| Modal feedrate | on |
| Laser On | `M4 S0` |
| Travel Move | `G0{x_cmd}{y_cmd}{z_cmd}{extra_cmd}{f_command}{s_command}` |
| Linear Move | `G1{x_cmd}{y_cmd}{z_cmd}{extra_cmd}{f_command}{s_command}` |
| Arc (CW) | `G2{x_cmd}{y_cmd}{z_cmd}{extra_cmd} I{i} J{j}{f_command}{s_command}` |
| Arc (CCW) | `G3{x_cmd}{y_cmd}{z_cmd}{extra_cmd} I{i} J{j}{f_command}{s_command}` |
| Postscript | `G4 P0`, `M5`, `M67` (one per line) |

Start from **Grbl (Compat)**, not from GRBL Raster: the dialect editor has no switch for `can_g0_with_speed`, and only Grbl (Compat) has it off. Everything else stays as it is.

### Framing and preview

Rayforge's **Frame** (toolbar button, or **Machine > Frame**) generates a normal minimal job (`rayforge/machine/driver/driver.py`, `build_frame_ops`): preamble, the outline with constant power (`M3`) at the head's Frame Power and Frame Speed, then the postscript. Frame Power is 0 %, so it sends `M3 S0` and the marking laser cannot fire. Frame Speed, Frame Power, Repeat Count and Pause at Corners are in Machine Settings > **Heads** > laser > **Framing**.

To see the outline with the guide laser, **just click Frame**. With `$231=1` (the default) the firmware turns the guide laser on while it moves under `M3 S0` - exactly what Rayforge's Frame sends; real jobs always use `M4` - and turns it off after the last frame move, at the frame's closing `M5` (also on `M4`, any `S` above 0, `M2`/`M30` or a soft reset). Keep Frame Power at **0 %**. With `$231=0`, or to trace a whole job instead of its box, use the macros: **Preview ON (M66)**, run Frame or the job, then **Preview OFF (M67)** (the postscript's `M67` also switches it off at the end of a job).

Why there is no hook that sends `M66` / `M67` around framing: the frame job is built from the same layer and workpiece markers as a real job, and a `WORKPIECE_START` hook would run for every real job as well. `M66` in a real job would stop all marking. The macro is the safe way.

---

## 1. Connection

- **Driver:** **GRBL (Serial)** (`GrblSerialDriver`, Machine Settings > **General** > **Driver Settings**) with character counting, on the ESP32-S3's **native USB port** (USB-Serial-JTAG). The baud rate does not matter, but the field must not be empty.
- **Port:** on macOS `/dev/cu.usbmodem*`.
- **RX buffer:** **RX Buffer Size Override** `0` auto-detects it from `$I`, which reports `[OPT:V,512,1024]`. The 1024 is the RX buffer. If you enter it by hand, use **1024** (the field's maximum).
- **Poll device status during jobs** and **Deadlock detection** stay off (Rayforge's defaults). Do not turn polling on, it only adds traffic.

## 2. Dialect

- **WaveMaster Galvo** (`wavemaster`, from `docs/rayforge/`) is the recommended dialect. It is **GRBL Raster** with the changes described in the quick setup.
- **Grbl (Compat)** (`grbl`) works for vector jobs. Its postscript returns to `G0 X0 Y0` and it sends `M4 S...` / `M5` around every segment.
- **GRBL Raster** (`grbl_raster`) has the lowest overhead for raster and engraving. It sends `M4 S0` once, then an `S` value on every move, and `F` only when it changes. It advertises F on G0 (`can_g0_with_speed`), which the firmware does not use for jumps.
- Choose the dialect in Machine Settings > **G-code** > **Dialect**. When a machine uses a built-in dialect, Rayforge copies it into a per-machine custom dialect ("... (for MachineName)") on the next load, so edits never touch the built-in.
- Arcs can stay enabled (Machine Settings > **Advanced** > **Support Arcs**). The firmware linearises `G2` / `G3` to the chord tolerance `$12`. Set **Arc Tolerance** to the same value (0.01).

## 3. Machine

| Rayforge setting (Machine Settings) | WaveMaster value | Notes |
|:---|:---|:---|
| **Hardware** > X Extent / Y Extent | `$130` / `$131` (mm) | Default 100 x 100. |
| **Hardware** > Coordinate Origin (0,0) | **Bottom Left** with `$144=0` | `0` = corner origin: (0,0)..(W,H), work area centred on the lens. `1` = lens centre is (0,0), which Rayforge's work area (it starts at 0,0) cannot express: keep `0`. |
| **Hardware** > Has Z-Axis | off | No Z axis. |
| **Heads** > laser > Max Power | `$30` | S value for 100 % power. Default 1000. |
| **General** > Max Cut Speed | up to `$110` (mm/min) | Feeds above `$110` are clamped by the firmware. |
| **General** > Max Travel Speed | `$201` x 60 (mm/min) | Estimates only. Jumps run at `$201`. |
| **Advanced** > Home On Start, Allow Single Axis Homing | off | `$H` is accepted and does nothing. |

## 4. Speed semantics

- **Marking speed:** `F` (mm/min) is the constant marking speed. Without an `F`, the firmware uses `$203` (mm/s).
- **Travel (`G0`):** uses the jump speed `$201` (**mm/s**, not mm/min), with `$120` as the jump acceleration (mm/s²; `0` = no ramp).
- Typical galvo marking speeds are 500-3000 mm/s, which is **30000-180000 mm/min**.

## 5. Raster and engraving

- **Use dithering.** With `S` at 0 or maximum, the laser is only gated on and off. That makes no ATmega traffic.
- **Grayscale** works, but each power change between two lasing runs stalls for about 1 ms, because power is latched over the ATmega link. For grayscale without that stall, set **`$223=1`** (pulse-density mode). The firmware then keeps power fixed at `$225` and gates the density of pulses instead. The gating period is `$229` ticks.

## 6. Framing and preview

- **`M66`** turns preview on: guide laser on, marking laser forced off, the real trajectory is run.
- **`M67`** turns preview off.
- `M62` / `M63` turn the guide laser on and off directly.
- Rayforge's **Frame** shows the outline with the guide laser by itself (`$231=1`, default: guide on while moving under `M3 S0`, off at the frame's closing `M5`). Keep the head's **Frame Power** at 0 %. Use M66/M67 to trace a whole job instead.

## 7. Cancel

Rayforge's **Cancel Job** soft-resets the controller (**Ctrl-X**, `0x18`) and then sends the dialect's laser-off commands (`M5`, `M9`). The soft reset also clears preview. The firmware cuts the gate immediately and stops the stream. The target is within about 60 ms. This is not yet measured on hardware (see [STATUS.md](../STATUS.md)).

## 8. Tuning the galvo and laser delays

| Setting | Meaning | Default |
|:---|:---|:---|
| `$200` | Tick period, us (one DAC point per tick). The actual period is quantised by the SPI clock; read it back from `$S` as `tick_us`. | 10 |
| `$210` | Laser-on delay, us | 100 |
| `$211` | Laser-off delay, us | 120 |
| `$212` | Jump delay, us (settle after `G0`) | 300 |
| `$213` | Extra jump delay per mm of jump, us/mm | 0 |
| `$214` | Mark delay, us (hold at the end of a polyline) | 100 |
| `$215` | Polygon delay at a 180 deg corner, us | 0 |
| `$216` | Polygon delay angle threshold, deg | 30 |

Tuning procedure:

1. Draw a grid of short lines at your working speed. Include several line lengths. `tools/stream_test.py grid` generates this pattern.
2. **Dots at the start of each line:** the laser-on delay `$210` is too short. Increase it. Gaps at the start mean it is too long.
3. **Tails at the end of each line:** the laser-off delay `$211` is too long. Decrease it. If the end is cut short, it is too short.
4. **A hook at the start of marks after a jump:** the galvo has not settled. Increase the jump delay `$212`.
5. **Burnt corners:** raise `$215` and check `$216`.
6. Repeat with the speeds you actually use. The delays are in microseconds, so their effect in mm scales with speed.

Change `$200` only if you need to. The delay settings above are the normal tuning knobs.

## 9. Calibration

The guided way is the calibration wizard, `python3 tools/calibrate.py PORT` (add `--fire` for the real-laser steps). It walks through orientation, scale, distortion, offset, delays, power and speed, computes the `$` values from your measurements, writes them and keeps a restorable backup. The procedure, the measuring technique and troubleshooting are in **[tools/CALIBRATION.md](../tools/CALIBRATION.md)**. The manual summary:


- **Scale, `$100` / `$101` (V/mm):** draw a 50 mm square, measure it, then adjust the scale until it measures 50 mm.
- **Offset, `$140` / `$141` (V):** added to the X / Y output voltage.
- **Invert, `$3`:** bit 0 inverts X, bit 1 inverts Y.
- **Swap X/Y, `$143`:** `1` swaps the axes.
- **Radial correction, `$142` (k1):** applies `r' = r(1 + k1 r²)` for F-theta lens distortion.

## 10. Diagnostics

Send these from the Rayforge **Console** (bottom panel).

| Command | Output |
|:---|:---|
| `$S` | Stream statistics: `[MSG:galvo chunks=... underruns=... barriers=... ticks=... tick_us=...]`, and the ATmega link state: `[MSG:atmega link=... armed=... ready=... ...]`. |
| `$RB` | DAC output register readback, `[MSG:RB X=0x.... Y=0x....]`. Only valid while idle. |
| `$RD` | DAC configuration register dump (range `19`, offsets/gains `1B`-`1E`, interface and stream config), `[MSG:RD 00=.. ...]`. Only while idle. Compare with a dump taken right after boot; `$S` reports `[MSG:dac config_repairs=N]` when the firmware had to restore any of them. |
| `$PW=<0-255>` | Power-word wiring test: latches a raw word on DB25 pins 1-8 (D0-D7) through the ATmega, with no motion and no gate. Walk the bits (`$PW=1`, `2`, `4` … `128`) and check that only the matching pin is high (~5 V). The next job re-applies its own power. |
| `$GT=<ms>` | Gate wiring test: drives EMISSION MODULATION (GPIO4 → DB25 pin 19) active for up to 5000 ms so you can check the level at the connector with a meter or scope. Refused while the laser is armed (no MO → no emission). |
| `$LT=<ms>,<S>` | Static laser test: sets the power for `S`, then opens the gate for up to 2000 ms with the galvo standing still (burns one spot). Refused unless armed **and** ready (`M10`, wait ~2 s). Ctrl-X stops it. |
| `$$` | All settings. |
| `$I` | Version and the RX buffer size. |
| `$#` | Work offsets and G92. |
| `$G` | Parser state. |

Firmware messages arrive in the host stream as `[MSG:...]` lines.

M-codes `M64`, `M65` and `M68` are accepted and ignored. Do not rely on them.

## Validation status

Done with a throwaway Python venv, `python -I`, and a clone of Rayforge at commit `22dc2c6d` (not installed system-wide), using `RAYFORGE_CONFIG_DIR` pointing at an empty temporary folder. Rayforge's own classes loaded both files from the folders given above:

- `DialectManager` loaded `dialects/wavemaster.yaml` and `to_dict(from_dict(file))` reproduced the file exactly.
- `MachineManager` loaded `machines/wavemaster.yaml` (the real load path, `Machine.from_dict`) and `Machine.to_dict()` reproduced the file exactly. The dialect resolved to `wavemaster` without a "(for ...)" copy, the 8 macros expanded through Rayforge's macro formatter, and the driver arguments are all known setup variables of `GrblSerialDriver`.
- Rayforge's G-code encoder (`raygeo`) produced the output shown above for a vector job, an arc, and the frame (power 0 and 5 %). No Z word appears.
- `wavemaster.rfdevice/` loaded as a `DeviceProfile`, installed from a zip made with the command above (`install_from_zip`), and `create_machine()` produced the same driver, 100 x 100, Bottom Left, speeds, head and dialect.

Not validated: the GTK user interface itself (menu and button names are read from the source), a real connection to the controller, and anything on the laser. The first-connection check above is the first real test.
