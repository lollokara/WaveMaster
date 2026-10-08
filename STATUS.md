# WaveMaster status

Last updated: 2026-10-08, branch `claude/admiring-mendel-iqe4lx`.

## What changed and why

The previous firmware moved the galvo point to point. Each `G1` jumped the DAC
to the endpoint, and the host sent one serial line at a time, at roughly 17
lines per second. A galvo commanded straight from A to B arrives almost at
once and then sits there. With the laser on, corners dwelled and lines were
faint, and the per-line serial round trip capped the job speed.

The redesign:

- **Constant-speed trajectory.** `src/motion.c` samples each path by arc length at a fixed tick (`$200`, default 10 us). Corners are passed at speed.
- **Hardware-paced streaming.** `src/dac_task.c` clocks chunks out as one continuous quad-SPI transfer. The SPI clock is the point rate. Chunks are 2048 ticks, and the pool is 8 deep.
- **Sample-locked gate.** The gate (GPIO4) is set from the SPI transaction callback and toggled mid-chunk by a GPTimer alarm. Both are timed from the same instant as the DAC samples.
- **Flow control.** The host's character counting stalls on a full segment queue, so there is no per-line round trip.
- **Safety.** The ATmega disarms within 1 s if the ESP32 goes silent, and abort (Ctrl-X) cuts the gate immediately.

## Verified

- **Firmware builds.** `pio run` in the repo root succeeds. The only warning in the log is an esptool flash-size notice, not a compiler warning in `src/`.
- **Companion builds.** `cd ArduinoCompanion && pio run` succeeds.
- **Host motion tests.** `make -C test/host`: 58808 checks, 0 failures.
- **Host G-code tests.** `make -C test/host/gcode`: passes.

## Verified on hardware (previous generation)

These results were measured on earlier code. They motivate the design, but the
new stream engine has not been measured yet.

- AD3552R streaming mode at SPI-clock pacing: 10 to 104 us per point. Measured P200 9.90 us, P100 19.97 us, P50 40.96 us, P20 104.35 us. One X+Y point is 12 quad-mode SPI clocks.
- Register readback is the ground truth for the DAC output. SPI success alone did not catch a single-point write that was a no-op.
- UART0 TX and RX are swapped relative to the IOMUX defaults. The ATmega link is TX on GPIO44 and RX on GPIO43.
- The level shifter must be powered. The CH340 on the Arduino board sits on the same TX and RX lines, so do not leave a serial monitor connected.

## Not verified on hardware

- Gate timing against the DAC output (`START_LATENCY_COUNTS` in `src/dac_task.c` is 0 and is unmeasured).
- Constant speed along a line, and the laser-on and laser-off edge positions.
- Abort latency. The target is under about 60 ms with the gate off immediately.
- Underrun behaviour during a Rayforge job.
- Heartbeat disarm on the ATmega.
- Power latch timing and all eight power bits.
- AVR watchdog behaviour with the bootloader in use.
- Analog output voltage. Only register readback has been checked, not a meter or scope.

## Bring-up checklist

Work through this in order. Do the first steps with the laser disconnected.

1. **Companion link.** Flash the companion (`cd ArduinoCompanion && pio run -t upload`). Boot the ESP32 and send `$S`. Check that `atmega link=1` appears.
2. **DAC self-test.** The boot log must show `self-test OK` (the boot test writes +2.5 V to CH0 and -2.5 V to CH1, then reads them back). It must not show `MISMATCH`. Then send `$RB` and compare the codes with the expected output.
3. **Chunk boundary.** On a scope, watch SCLK and CS across a chunk boundary. CS must stay low. Expect a gap of a few microseconds in SCLK per chunk, about every 20 ms.
4. **Gate against DAC.** Scope the gate (GPIO4) against the DAC output. The offset should be constant, with no drift within a 20 ms chunk. If it drifts or is offset, tune `START_LATENCY_COUNTS` in `src/dac_task.c`, or `$210` / `$211`.
5. **Constant speed.** Draw a long line at a known `F`. Check the ramp slope on the scope against the speed.
6. **Abort latency.** Send Ctrl-X during a long line. The gate must go off immediately. The stream should stop within about 60 ms.
7. **Underruns.** During a Rayforge job, `$S` must show `underruns=0`.
8. **Heartbeat.** Arm the laser (`M10`), then unplug the ESP32. The ATmega should disarm within about 1 s.
9. **Watchdog and bootloader.** The companion enables a 250 ms AVR watchdog. A classic "old bootloader" can boot-loop after a watchdog reset. If that happens, use Optiboot, or flash over ISP.
10. **Power latch.** Check the latch timing on a scope: 2 us setup, 3 us strobe. Step through all eight power bits and confirm each one reaches the laser.
11. **Barriers.** Power and PRR changes are barriers. CS is held low while the stream is open. Watch CS during the long idle at each barrier. If the DAC misbehaves there, close the streaming session at each barrier instead.

Tuning and test patterns: `tools/stream_test.py` generates grids, circles, ladders and squares. It runs in preview (`M66`) unless `--fire` is given.

## Open items

- `START_LATENCY_COUNTS` in `src/dac_task.c` is 0 and needs measuring (item 4).
- Grayscale raster stalls about 1 ms on each power change, because power is latched over the ATmega link. `$223=1` avoids this.
- DB25 pin 23 (E-stop) is held high by the firmware. Wire a real interlock before production use.
- `$230=1` (active-low gate) inverts the meaning of the 10 kOhm pull-down on pin 19. See [docs/WIRING.md](docs/WIRING.md).
- Pin 21 (LASERST3) has no firmware input.

The previous STATUS.md is in git history.
