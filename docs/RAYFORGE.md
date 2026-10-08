# Rayforge setup for WaveMaster

How to configure Rayforge to drive the WaveMaster galvo
controller. The wiring is in [WIRING.md](WIRING.md). Settings are described in
[README.md](../README.md).

---

## 1. Connection

- **Driver:** `GRBL (Serial)` with character counting, on the ESP32-S3's **native USB port** (USB-Serial-JTAG). The baud rate does not matter.
- **RX buffer:** auto-detected from `$I`, which reports `[OPT:V,512,1024]`. The 1024 is the RX buffer. If you enter it by hand, use **1024**.
- Rayforge does not poll the controller during a job by default. This is fine.

## 2. Dialect

- **`GRBL (Compat)` (`grbl`)** works for vector jobs.
- **`grbl_raster`** has the lowest overhead for raster and engraving. It sends `M4 S0` once, then an `S` value on every move.
- Arcs can stay enabled. The firmware linearises `G2` / `G3` to the chord tolerance `$12`.

## 3. Machine

| Rayforge setting | WaveMaster value | Notes |
|:---|:---|:---|
| Work area width / height | `$130` / `$131` (mm) | Default 100 x 100. |
| Origin | `$144` | `0` = corner origin: (0,0)..(W,H), work area centred on the lens. `1` = lens centre is (0,0). |
| Max power (laser head) | `$30` | S value for 100 % power. Default 1000. |
| Max speed | `$110` (mm/min) | Feeds above this are clamped. |
| Homing | off | `$H` is accepted and does nothing. |

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
- To frame with Rayforge's frame feature, send `M66` from the console or a macro first, frame, then send `M67`.

## 7. Cancel

Rayforge's stop sends **Ctrl-X** (`0x18`). The firmware cuts the gate immediately and stops the stream. The target is within about 60 ms. This is not yet measured on hardware (see [STATUS.md](../STATUS.md)).

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

Send these from the Rayforge console.

| Command | Output |
|:---|:---|
| `$S` | Stream statistics: `[MSG:galvo chunks=... underruns=... barriers=... ticks=... tick_us=...]`, and the ATmega link state: `[MSG:atmega link=... armed=... ready=... ...]`. |
| `$RB` | DAC output register readback, `[MSG:RB X=0x.... Y=0x....]`. Only valid while idle. |
| `$$` | All settings. |
| `$I` | Version and the RX buffer size. |
| `$#` | Work offsets and G92. |
| `$G` | Parser state. |

Firmware messages arrive in the host stream as `[MSG:...]` lines.

M-codes `M64`, `M65` and `M68` are accepted and ignored. Do not rely on them.
