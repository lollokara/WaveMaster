#!/usr/bin/env python3
"""
Bring-up / tuning streamer for WaveMaster.

Streams a generated test pattern with GRBL character counting (the same
protocol Rayforge uses), then prints throughput and the firmware's own
stream statistics ($S: underruns, barriers, ATmega link state).

Patterns:
  grid     rows of short marks separated by jumps - tunes laser on/off
           delays ($210/$211) and jump delay ($212): dots at the start of
           each mark = on-delay too short, gaps = too long; tails at the
           end = off-delay too long.
  circles  concentric circles from G2 arcs - checks constant speed and
           the arc tolerance ($12).
  ladder   the same square at increasing F - shows the speed range and
           whether corners stay sharp.
  square   a single square of --size mm, for scale calibration ($100/$101).

Safety: by default the run is wrapped in M66/M67 (preview: guide laser on,
marking laser forced off), so nothing fires. Pass --fire to actually mark.

Usage:
    python3 tools/stream_test.py /dev/ttyACM0 grid
    python3 tools/stream_test.py /dev/ttyACM0 circles --fire --power 300
"""
import argparse
import math
import re
import sys
import time

import serial

RX_WINDOW_DEFAULT = 127


def pattern_grid(a):
    lines = []
    for r in range(a.rows):
        y = a.y0 + r * a.pitch
        for c in range(a.cols):
            x = a.x0 + c * a.pitch
            lines.append('G0 X%.3f Y%.3f' % (x, y))
            lines.append('M4 S%d' % a.power)
            lines.append('G1 X%.3f F%d' % (x + a.pitch * 0.6, a.feed))
            lines.append('M5')
    return lines


def pattern_circles(a):
    lines = []
    cx, cy = a.x0 + a.size / 2, a.y0 + a.size / 2
    n = max(1, int(a.size / 2 / a.pitch))
    for k in range(1, n + 1):
        r = k * a.pitch
        lines.append('G0 X%.3f Y%.3f' % (cx + r, cy))
        lines.append('M4 S%d' % a.power)
        lines.append('G2 X%.3f Y%.3f I%.3f J0 F%d' % (cx + r, cy, -r, a.feed))
        lines.append('M5')
    return lines


def square(x, y, s, feed, power):
    return ['G0 X%.3f Y%.3f' % (x, y), 'M4 S%d' % power,
            'G1 X%.3f F%d' % (x + s, feed), 'G1 Y%.3f' % (y + s),
            'G1 X%.3f' % x, 'G1 Y%.3f' % y, 'M5']


def pattern_ladder(a):
    lines = []
    s = a.size / 6
    for i, f in enumerate([6000, 15000, 30000, 60000, 120000, 180000]):
        lines += square(a.x0 + (i % 3) * 2 * s, a.y0 + (i // 3) * 2 * s, s, f, a.power)
    return lines


def pattern_square(a):
    return square(a.x0, a.y0, a.size, a.feed, a.power)


PATTERNS = {'grid': pattern_grid, 'circles': pattern_circles,
            'ladder': pattern_ladder, 'square': pattern_square}


class Grbl:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.05)
        self.buf = b''
        self.rx_window = RX_WINDOW_DEFAULT

    def readline(self, timeout=2.0):
        end = time.time() + timeout
        while time.time() < end:
            if b'\n' in self.buf:
                line, self.buf = self.buf.split(b'\n', 1)
                return line.decode(errors='replace').strip()
            self.buf += self.s.read(256)
        return None

    def command(self, line, timeout=5.0):
        """Send one line and wait for ok/error; returns the reply lines."""
        self.s.write((line + '\n').encode())
        out = []
        end = time.time() + timeout
        while time.time() < end:
            l = self.readline(end - time.time())
            if l is None:
                break
            if l == 'ok' or l.startswith('error:'):
                out.append(l)
                return out
            out.append(l)
        raise RuntimeError('no reply to %r (got %r)' % (line, out))

    def handshake(self):
        self.s.write(b'?')
        for _ in range(20):
            l = self.readline(0.5)
            if l and (l.startswith('<') or l.startswith('Grbl')):
                break
        self.s.reset_input_buffer()
        self.buf = b''
        for l in self.command('$I'):
            m = re.match(r'\[OPT:[^,]*,\d+,(\d+)\]', l)
            if m:
                self.rx_window = int(m.group(1))
            if l.startswith('[') or l.startswith('ok'):
                print('  ' + l)

    def stream(self, lines):
        """GRBL character-counting streaming. Returns (lines, seconds, errors)."""
        pending = []      # lengths of lines sent but not acknowledged
        errors = []
        i = 0
        t0 = time.time()
        while i < len(lines) or pending:
            while i < len(lines) and sum(pending) + len(lines[i]) + 1 <= self.rx_window:
                self.s.write((lines[i] + '\n').encode())
                pending.append(len(lines[i]) + 1)
                i += 1
            l = self.readline(10.0)
            if l is None:
                raise RuntimeError('stream stalled with %d lines in flight' % len(pending))
            if l == 'ok' or l.startswith('error:'):
                pending.pop(0)
                if l != 'ok':
                    errors.append((i - len(pending), l))
            elif l.startswith('[MSG:'):
                print('  ' + l)
        return len(lines), time.time() - t0, errors

    def wait_idle(self, timeout=60.0):
        end = time.time() + timeout
        while time.time() < end:
            self.s.write(b'?')
            l = self.readline(0.5)
            if l and l.startswith('<Idle'):
                return l
            time.sleep(0.05)
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port')
    ap.add_argument('pattern', choices=sorted(PATTERNS))
    ap.add_argument('--fire', action='store_true', help='really mark (default: M66 preview)')
    ap.add_argument('--power', type=int, default=200, help='S value (default 200 of $30)')
    ap.add_argument('--feed', type=int, default=60000, help='marking speed, mm/min')
    ap.add_argument('--x0', type=float, default=10.0)
    ap.add_argument('--y0', type=float, default=10.0)
    ap.add_argument('--size', type=float, default=40.0, help='pattern size, mm')
    ap.add_argument('--pitch', type=float, default=2.0, help='grid / ring pitch, mm')
    ap.add_argument('--rows', type=int, default=10)
    ap.add_argument('--cols', type=int, default=10)
    a = ap.parse_args()

    g = Grbl(a.port)
    print('Connecting...')
    g.handshake()
    print('RX window: %d bytes' % g.rx_window)

    body = PATTERNS[a.pattern](a)
    pre = ['G21', 'G90'] + ([] if a.fire else ['M66'])
    post = ['M5', 'G0 X%.3f Y%.3f' % (a.x0, a.y0)] + ([] if a.fire else ['M67'])
    if a.fire:
        print('*** FIRING THE LASER - make sure the enclosure is closed ***')
        time.sleep(2)

    n, secs, errors = g.stream(pre + body + post)
    print('Streamed %d lines in %.2f s = %.0f lines/s' % (n, secs, n / secs if secs else 0))
    for idx, e in errors:
        print('  line %d: %s' % (idx, e))
    print('Final: %s' % g.wait_idle())
    for l in g.command('$S'):
        print('  ' + l)
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
