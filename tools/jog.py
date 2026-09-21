#!/usr/bin/env python3
"""
Interactive keyboard jog tool for WaveMaster.

Arrow keys move the galvo by one step (default 0.5mm) in absolute
coordinates; space toggles the guide laser (M62/M63); 'q' or Ctrl-C
quits. Position (from the firmware's own "?" status report) is printed
after every move.

Safety: the whole session runs in preview mode (M66) - the marking
laser is forced off regardless of M3/M4 intent, so jogging around can
never fire the real laser by accident. Guide laser and preview mode are
both turned off again on exit.

Usage:
    python3 tools/jog.py /dev/cu.usbmodem13201
    python3 tools/jog.py /dev/cu.usbmodem13201 --step 0.1
"""
import argparse
import sys
import termios
import time
import tty

import serial


def read_key(fd):
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        ch2 = sys.stdin.read(1)
        ch3 = sys.stdin.read(1)
        return ch + ch2 + ch3
    return ch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port', help='serial port, e.g. /dev/cu.usbmodem13201')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--step', type=float, default=0.5, help='jog step size in mm')
    args = parser.parse_args()

    s = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(2)
    s.reset_input_buffer()

    def send(cmd):
        s.write((cmd + '\n').encode())
        time.sleep(0.03)
        return s.read(2000).decode(errors='replace')

    def query_pos():
        s.write(b'?')
        time.sleep(0.08)
        return s.read(500).decode(errors='replace').strip()

    send('G21')
    send('G90')
    send('M66')  # preview mode: guide laser available, marking laser forced off

    x = 0.0
    y = 0.0
    guide_on = False

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        sys.stdout.write(
            '\r\nArrows: jog %.2fmm  |  space: toggle guide laser  |  q: quit\r\n' % args.step
        )
        sys.stdout.flush()

        while True:
            key = read_key(fd)

            if key in ('q', '\x03'):
                break
            elif key == ' ':
                guide_on = not guide_on
                send('M62' if guide_on else 'M63')
            elif key == '\x1b[A':   # up
                y += args.step
            elif key == '\x1b[B':   # down
                y -= args.step
            elif key == '\x1b[C':   # right
                x += args.step
            elif key == '\x1b[D':   # left
                x -= args.step
            else:
                continue

            if key in ('\x1b[A', '\x1b[B', '\x1b[C', '\x1b[D'):
                send('G0 X%.3f Y%.3f' % (x, y))

            pos = query_pos()
            sys.stdout.write('\r%s   guide=%s   ' % (pos, 'ON ' if guide_on else 'off'))
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        sys.stdout.write('\r\nExiting: guide laser off, preview mode off.\r\n')
        send('M63')
        send('M67')
        s.close()


if __name__ == '__main__':
    main()
