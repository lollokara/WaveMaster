#!/usr/bin/env python3
"""
Records a shape (square, triangle, or circle) via M64 and loops it
on-device with the guide laser via M68 - see STATUS.md's "On-device
shape preview loop" section for why (the simple one-line-per-point
serial protocol tops out around 5 moves/sec, nowhere near fast enough
to watch a shape live).

Marking laser stays forced off the whole session (M66 preview mode).

Usage:
    python3 tools/square_loop.py /dev/cu.usbmodem13201
    python3 tools/square_loop.py /dev/cu.usbmodem13201 --shape circle --size 4 --hz 50
    python3 tools/square_loop.py /dev/cu.usbmodem13201 --shape triangle --size 3 --points 3

If the galvo shows overshoot/ringing at corners or visible dwell spots
along an edge, that's a real physical slew-rate effect (see STATUS.md) -
adjust it live without reflashing:
    $190=<n>   edge subdivisions (higher = smoother, but shorter dwell
               per point for the same --hz)
    $182=<mm/s> max velocity - also used for the G0 travel move to the
               shape's start point
Send those as their own lines to the same serial port (e.g. with a
terminal, or a one-off pyserial script) before re-running this tool.
"""
import argparse
import math
import re
import sys
import time

import serial


def square_points(size):
    half = size / 2.0
    return [(-half, -half), (half, -half), (half, half), (-half, half)]


def triangle_points(size):
    # Equilateral triangle, centered on the origin, "size" = circumradius.
    return [
        (size * math.cos(math.radians(90 + 120 * i)),
         size * math.sin(math.radians(90 + 120 * i)))
        for i in range(3)
    ]


def circle_points(size, n):
    radius = size / 2.0
    return [
        (radius * math.cos(2 * math.pi * i / n), radius * math.sin(2 * math.pi * i / n))
        for i in range(n)
    ]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port', help='serial port, e.g. /dev/cu.usbmodem13201')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--shape', choices=['square', 'triangle', 'circle'],
                         default='square')
    parser.add_argument('--size', type=float, default=2.0,
                         help='square side length / triangle circumradius / circle '
                              'diameter, in mm (default 2.0)')
    parser.add_argument('--points', type=int, default=24,
                         help='vertex count approximating a circle (ignored for '
                              'square/triangle, default 24)')
    parser.add_argument('--hz', type=float, default=100.0,
                         help='M68 repeat rate in Hz (default 100)')
    parser.add_argument('--subdivisions', type=int, default=None,
                         help='edge interpolation points ($190) - sets it live '
                              'before recording/looping; omit to leave whatever '
                              'value is already persisted on the board')
    parser.add_argument('--seconds', type=float, default=0.0,
                         help='stop after roughly this long (0 = keep looping '
                              'indefinitely, re-issuing M68 as each burst finishes, '
                              'until Ctrl-C)')
    args = parser.parse_args()

    if args.shape == 'square':
        pts = square_points(args.size)
    elif args.shape == 'triangle':
        pts = triangle_points(args.size)
    else:
        pts = circle_points(args.size, args.points)

    s = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(2)
    s.reset_input_buffer()

    def send(cmd, wait=0.1):
        s.write((cmd + '\n').encode())
        time.sleep(wait)
        return s.read(2000).decode(errors='replace')

    print('Preview mode on (guide laser only, marking laser forced off)...')
    send('G21')
    send('G90')
    send('M66')
    send('M62')  # guide laser on for the whole run

    if args.subdivisions is not None:
        print('Setting edge subdivisions ($190) to %d...' % args.subdivisions)
        send('$190=%d' % args.subdivisions)

    print('Recording %s (%d points, size %.2fmm) via M64...'
          % (args.shape, len(pts), args.size))
    send('M64')
    for (x, y) in pts:
        send('G0 X%.4f Y%.4f' % (x, y))

    print('Looping %s%s - Ctrl-C to stop.'
          % (args.shape, '' if args.seconds <= 0 else ' for ~%.0fs' % args.seconds))
    t_start = time.time()
    try:
        while True:
            resp = send('M68 P%.1f' % args.hz, 0.3)
            print(resp.strip())

            if args.seconds > 0 and (time.time() - t_start) >= args.seconds:
                break

            # M68 is bounded to one burst per call (see STATUS.md's "known
            # limitation") - parse the logged duration and wait for it to
            # actually finish before re-issuing. The log line fires as
            # soon as the burst is *queued*, not once dac_task has
            # actually drained it (~160KB for a 20000-point buffer) - re-
            # issuing too early leaves the previous burst's buffer still
            # allocated when the next one is built, and two of them
            # together can exceed this board's ~300KB heap ("out of
            # memory" - found the hard way). Wait a bit *longer* than the
            # logged duration, not shorter, to be sure it's freed first.
            # Generous margin: the logged duration doesn't account for
            # dac_task's periodic scheduler-yield pauses during a paced
            # burst (added to avoid starving the idle-task watchdog on
            # long bursts - see dac_task.c), which can push the real
            # duration well past the estimate.
            m = re.search(r'~(\d+)s\)', resp)
            burst_s = float(m.group(1)) if m else 2.0
            time.sleep(burst_s * 1.5 + 1.5)

            if args.seconds > 0 and (time.time() - t_start) >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print('\nStopping: guide laser off, preview mode off.')
        send('M63')
        send('M67')
        s.close()


if __name__ == '__main__':
    main()
