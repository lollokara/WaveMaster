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

    # A single M68 loops indefinitely on-device: dac_task replays the
    # outline buffer until another command is queued, so there is nothing
    # to re-issue and no gap between revolutions. This used to be a
    # re-issue loop that guessed from the logged burst duration when the
    # point buffer had been freed; guessing wrong either dropped bursts
    # with "out of memory" or left the galvo parked between them.
    print('Looping %s%s - Ctrl-C to stop.'
          % (args.shape, '' if args.seconds <= 0 else ' for ~%.0fs' % args.seconds))
    print(send('M68 P%.1f' % args.hz, 0.5).strip())

    t_start = time.time()
    try:
        while args.seconds <= 0 or (time.time() - t_start) < args.seconds:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        # Any command ends the on-device loop; these are also what restores
        # the laser state, so no separate stop command is needed.
        print('\nStopping: guide laser off, preview mode off.')
        send('M63')
        send('M67')
        s.close()


if __name__ == '__main__':
    main()
