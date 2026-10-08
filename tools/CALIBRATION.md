# Calibrating WaveMaster

`tools/calibrate.py` is an interactive wizard. It draws test patterns, tells you what to
look at and measure, asks for the numbers, computes the new `$` settings, shows
`old -> new`, writes them after you confirm, and can re-draw the pattern to check that the
error went away. This document is the procedure behind it.

```bash
pip install pyserial
python3 tools/calibrate.py /dev/ttyACM0                   # guide laser only: safe, no emission
python3 tools/calibrate.py /dev/ttyACM0 --fire --max-power 300   # also the real-laser steps
python3 tools/calibrate.py /dev/ttyACM0 --step scale --step offset   # selected steps only
python3 tools/calibrate.py /dev/ttyACM0 --restore tools/calibrate/runs/<ts>/settings_before.json
python3 tools/calibrate.py --mock                          # rehearse offline (tools/rftest/mock_grbl.py)
python3 tools/calibrate_selftest.py                        # offline self-test of the wizard and its math
```

The port is the ESP32-S3 native USB (same as `tools/rftest`). Close Rayforge first.

At every prompt: a number (`12.34` or `12,34`; two numbers may be separated by a space),
`s` skips the current step, `q` quits (you get the summary and an offer to restore the
backup), `again` draws the pattern again.

## What you need

| Item | For |
|:---|:---|
| Digital calipers (150 mm, 0.01 mm) | every measurement; a steel ruler is enough for the guide-laser steps |
| Test cards: anodized aluminium (black) or black-painted metal, flat, about 70 x 70 mm, at least 4-6 pieces | real-laser marks (scale, distortion, offset, delays, power, speed) |
| White paper, tape, a sharp pencil | guide-laser marks when you do not fire |
| A loupe or microscope camera (10x) | the delay steps; also gives better scale readings |
| Laser safety glasses rated for your laser wavelength, closed enclosure, working interlock | any `--fire` step |
| The wizard output folder | `tools/calibrate/runs/<timestamp>/` |

Put the test card (or the paper) at the real **working height**: the surface must be at the
focal plane, flat, and fixed. Scale and distortion are only valid at that height. Redo them
if you change the lens or the distance.

## Safety

* Only steps `delays` and `power` need `--fire`. For `scale`, `distortion`, `offset` and the
  jump-speed part of `speed`, `--fire` lets you mark with the real laser; without it the same
  patterns are traced with the red guide laser (M66 preview, S0).
* The same rules as `tools/rftest/run_tests.py`: `--fire` is required, `FIRE` must be typed
  once per session (`--yes` skips it and the frame question), every pattern is framed with the
  guide laser first, `M10` arms and `$S` must show `armed=1`, every `S` is capped at
  `--max-power` (default 300 of 1000), Ctrl-C or any error sends `0x18`, `M5`, `M9`, `M11`.
* The laser is disarmed (`M11`) after every pattern so you can open the enclosure to look at the
  card. `--keep-armed` keeps it armed between patterns and at the end. Do not use it with the
  enclosure open.
* Start with low `S`. The wizard asks for the mark power once (default `S100`). Use a power that
  just visibly marks the card, not one that burns through the anodizing.
* `$S` must show `atmega link=1`. With `link=0` the wizard warns and skips the fire steps.

## Order of the steps, and why

1. **connect** reads `$I`, `$$` and `$S`, and saves the backup. Nothing is changed.
2. **field** sets the lens field (`$130`/`$131`), the origin mode (`$144`) and the laser PRR
   (`$220`/`$221`). Everything after uses the field size to keep patterns inside the work area.
3. **orientation** must come first. Scale, distortion and offset are computed per physical axis
   (X to your right, Y away from you); they only make sense once commanded +X really moves the
   beam right and +Y away. It sets `$143` (swap) and `$3` (invert).
4. **scale** next: a small square so that lens distortion is negligible, sets `$100`/`$101`.
5. **distortion** needs the right scale, and fits the radial term `$142` (k1) plus the residual
   centre scales. It uses a large square.
6. **offset** last of the geometry: it only moves the whole picture, and it is measured at the
   centre where distortion does not matter. A large offset error (more than about 1 mm) slightly
   biases the distortion fit, so after a big offset correction run `distortion` once more (its
   first pass should then already be within tolerance).
7. **delays** after the geometry is right, because the marks are judged by eye on corners and
   short lines: `$210` laser-on, `$211` laser-off, `$212` jump, `$213` jump per mm, `$214` mark,
   `$215` polygon.
8. **power** maps `S` onto the useful part of the laser power range (`$224`/`$225`).
9. **speed** finds the highest marking speed with clean corners (`$110`) and the highest clean
   jump speed (`$201`).
10. **summary** lists all changes and the final `$$`.

## Measuring: general rules

* Measure to the **centres of the marked lines**, not to the outer edges. A 0.1 mm line adds
  0.1 mm to an outside-to-outside measurement. Use the same reference (all centres, or all outer
  edges, or all inner edges) on both ends of a length.
* Use a loupe. Place the caliper jaws on the line centres; for guide-laser pencil marks, use the
  centre of the dot.
* Square sides are measured between **corner marks** in a straight line, top and bottom (or left
  and right) separately; give the two numbers and the wizard averages them.
* Keep the card flat and in the same place when you put it back after measuring.
* The wizard asks the measurement in mm. A typo that is more than 4x off is rejected.
* Convergence matters more than a single perfect measurement: the wizard re-draws after each
  correction, so a 0.05 mm reading error does not accumulate.

## Step by step

### orientation (guide laser)

Look from above, standing at the front of the machine, as you will when you use it. A red arrow
is traced for a few seconds around the field centre, first one that stands for **+X**, then one
for **+Y**. Where is the arrow **head** (the pointed end)? Answer `right`, `left`, `up` (away
from you) or `down` (toward you). `again` repeats the trace.

The wizard works out which DAC channel moves which physical axis and with which sign, and then
sets `$143` and `$3` so that +X is right and +Y is away (the firmware inverts first, then
swaps). It re-draws both arrows and asks again to confirm. If you report both arrows along the
same axis, it says so and asks again: with a working galvo pair that cannot happen, so check that
both axes move.

### scale (`$100`, `$101`)

A square of side S (default 20 mm) centred in the field, with a small cross at the centre. Measure
the **X side** (top and bottom) and the **Y side** (left and right). The new value is
`old * S / measured`; for a swapped machine the wizard applies the X measurement to `$101` and
the Y measurement to `$100`. It iterates until both errors are below 0.1 % or you stop. A 20 mm
square measured with 0.01 mm calipers gives about 0.05 % resolution.

Typical: `$100`/`$101` between 0.05 and 0.2 V/mm, the two within a few percent of each other.

### distortion (`$142`, and the scales)

A large square of side 2R (default R = 0.4 x the smaller field side, so 80 mm in a 100 mm field)
with a cross of total length 2R along both axes. Measure:

* cross **X** length and cross **Y** length (end to end of the arms);
* square **top and bottom** side lengths (two numbers) and **left and right** side lengths;
* how the square edges look against a straight ruler: bowing inward (concave, corners stick out:
  pincushion), bowing outward (convex: barrel), or straight. This is a sign check only. If your
  answer contradicts the sign implied by the numbers, the wizard warns you before you apply
  anything.

Model: the relative scale error is `e(r) = a + b r^2`. The cross arm ends sit at r = R, the square
corners at r = R sqrt 2. The firmware applies `r' = r (1 + k1 r^2)`, so the fix is
`k1_new = k1_old - b` and the centre errors `a` go into the scales. Each round applies 80 % of
the computed correction (damping 0.8) and re-draws, for at most 4 rounds or until every error is
below 0.15 %.

Typical: `|k1|` between 1e-7 and 1e-5 for a 100 mm F-theta lens (for R = 40 mm, `k1 R^2` is
a few tenths of a percent up to a few percent). The wizard warns when `|k1 R^2| > 0.1` and clamps
k1 to the firmware range -1..1.

### offset (`$140`, `$141`)

A crosshair at the work-area centre (or at the lens centre when `$144=1`). Decide what it
**should** hit: a reference mark on the fixture, a pinhole, the lens centreline, or just the
centre of the card if you only want it central. Measure where the crosshair lands relative to
that reference: `dx` is positive when it is to the **right** of the reference, `dy` is positive
when it is **further away from you**. Enter the two numbers; the wizard moves the beam by
`(-dx, -dy)` through the offsets of the channels that drive each axis (the sign depends on `$3`,
`$143` and the scale signs; it is covered by the self-test for every combination).

Typical: offsets below 0.3 V (about 3 mm). The offsets are limited to +-10 V.

### delays (`$210` ... `$216`), real laser

Use your real marking speed; the delays are in microseconds so their effect in mm scales with
speed. Each pattern goes to a fresh spot on the card (the card size is `--card WxH`, default
60 x 60; the wizard says when it is full). Look with a loupe.

| Look at | You see | Meaning | Change |
|:---|:---|:---|:---|
| START of short marks | small dot or blob | laser on too early | increase `$210` |
| START of short marks | the mark begins late, first part missing | laser on too late | decrease `$210` |
| END of short marks | blob or tail | laser off too late | decrease `$211` |
| END of short marks | end cut short | laser off too early | increase `$211` |
| START of marks after a jump | wavy or hooked start | galvo not settled | increase `$212` |
| same, only after LONG jumps | rows of long jumps hook, short ones do not | settle time grows with distance | increase `$213` (us per mm) |
| END of marks before a jump | smeared or hooked toward the next jump | mark ends before the galvo stops | increase `$214`; heavy dot at the end: decrease |
| Corners of the V shapes | rounded or cut corners | galvo cannot follow the corner | increase `$215`; burned corner spots: decrease |

Steps are 20 us (at least one tick); whenever your answer changes direction the step is halved
(bisection), so it converges in a handful of marks. The finest useful step is the tick period
(`$200`/`$S tick_us`, normally 10 us), because the firmware rounds a delay to whole ticks.
Defaults: `$210=100`, `$211=120`, `$212=300`, `$213=0`, `$214=100`, `$215=0`.

### power (`$224`, `$225`), real laser

Nine hatched patches with rising `S` (up to `--max-power`). Patch 1 is bottom-left (nearest you),
numbers rise left to right, then the next row away. Enter the **lowest patch that visibly marks** and the
patch where the result **stops getting stronger** (0 = never within the tested range). The
wizard converts those `S` values to power percent with the current mapping and sets `$224` (power at
`S` = 0+) to the marking threshold and `$225` (power at `S` = `$30`) to saturation. Rayforge's
max power stays `$30`; its 1 % ... 100 % range then spans "just marks" to "saturated". Use
`--max-power` high enough to see saturation, or accept that only `$224` is set. If the lowest patch
already marks, repeat with a smaller `--max-power`.

### speed (`$110`, `$201`)

Marking speeds: six patches (same square, outline plus hatch) from 200 to 2200 mm/s, never above
`$110`, at one fixed power. Say where corners start to round or lines break up; `$110` becomes the
last good patch's feed in mm/min (`$111` is an alias). Jump speed: marks separated by jumps of
2-24 mm are drawn with `$201` of 1000, 2000, 3000, 5000, 8000, 12000 mm/s; the first dirty
result stops the search and the last clean value is proposed. `$201` is restored to its old value
unless you accept the new one. Without `--fire` the jump test traces with the guide laser, which
only shows gross ringing.

## The `$$` display precision, and `last_values.json`

Firmware older than commit `0737950` printed settings with 4 decimals (`%.4f`), so `$$`
showed a k1 of `-0.0000102` as `0.0000`. Current firmware prints about 7 significant digits.
The wizard still keeps the exact values it wrote in `tools/calibrate/last_values.json` and
uses them while the `$$` readback agrees within 5e-5, so it works with either firmware. The
backup `settings_before.json` stores both the readback (`settings`) and these exact values
(`exact`); `--restore` uses `exact` when present.

## Files

`tools/calibrate/runs/<timestamp>/` (git-ignored, or `--out DIR`):

| File | Content |
|:---|:---|
| `settings_before.json` | `$$` before the first change (plus exact values and device info): the backup |
| `settings_after.json` | `$$` after the last change |
| `calibration_report.md` | changes with old/new/why, every step's measurements, warnings, restore command |
| `session.log`, `events.jsonl`, `firmware_msgs.log`, `statuses.csv` | every byte exchanged (same logger as `tools/rftest`) |

## Restoring a backup

```bash
python3 tools/calibrate.py /dev/ttyACM0 --restore tools/calibrate/runs/<timestamp>/settings_before.json
```

It shows `current -> backup` for each setting and asks before writing. `--yes` writes without
asking. A raw text dump of `$$` (lines like `$100=0.1000`) works as well; its values only have 4
decimals. Answering `q` during a run also offers the restore.

## Troubleshooting

| Symptom | Cause / fix |
|:---|:---|
| "no GRBL status/banner" | Wrong port (use the native USB, not the ATmega CH340), Rayforge or a monitor still has the port open. |
| `link=0` warning | ATmega not answering: UART0 (GPIO 44 TX / 43 RX), level shifter, 250000 baud, companion firmware. Guide steps still work. |
| Guide pattern is not visible | Guide laser off or misaligned: `M62` then `M63` from a console; check `$S guide=1` while tracing. Wait until the paper is at focal height. |
| "M66 sent but $S does not report guide=1" | The ATmega did not switch the guide laser. See above. |
| Orientation says "same axis" | Both arrows moved along one line: one galvo channel is dead or both DAC channels drive one mirror. Test with `$RB` and `tools/jog.py`. |
| Scale will not converge below 0.1 % | Card not flat or not at the focal plane; marks too wide (lower power, better focus); lens distortion (run `distortion`). |
| Distortion rounds keep alternating sign | Measurement noise: use the loupe and measure centres; or the square is bigger than the usable field. |
| k1 warning `|k1 R^2| > 0.1` | Wrong lengths entered (X and Y swapped?), or the scale step was skipped; check `$143` and re-run `scale`. |
| Offset reaches +-10 V | The galvo mount is mechanically far off centre; fix the mounting. |
| Delay never converges, "reached its limit" | The problem is not the delay: the marking speed is too high for the galvos (see `speed`), or the scale is wrong. |
| Marks double or ghost after the speed step | Lower `$110`; check `$212`. |
| `$142` reads `0.0000` after the run | Normal, see "The `$$` display limit". The exact value is in `last_values.json` and in the report. |
| Wizard exits with "SAFETY STOP" | Not armed, no link, wrong frame, FIRE not typed. The cancel sequence ran; nothing is firing. |

## Assumptions about the physical procedure

* Positive X is to the operator's right, positive Y is away, viewed from above, facing the machine.
* Both galvo channels are linear enough that the single fit `e(r) = a + b r^2` describes the lens;
  anything beyond that (keystone, non-orthogonal axes) is not corrected (the firmware has no such
  parameter).
* The reference mark for the offset step is also where the optical axis of the lens is. If they
  differ, decide which one you want the machine centre to be.
* The guide laser is coaxial with the marking laser. Paper marks give about 0.1-0.2 mm accuracy,
  real-laser marks on a card about 0.02-0.05 mm.
* `$100`/`$101` are signed volts per mm; negative scales are supported and handled in every formula.
