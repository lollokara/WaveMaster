#!/usr/bin/env python3
"""
mock_grbl - a fake WaveMaster device on a pseudo-terminal, for offline tests.

Models what the firmware does on the wire (src/grbl.c):
  * banner "Grbl 1.1h ['$' for help]", status "<Idle|MPos:..|FS:..|Bf:..>",
    $I / $$ / $S / $RB / $G / $# / $J=, G/M code acceptance (unknown -> error:20)
  * a 1024-byte RX buffer: bytes are free again only when their line is
    acknowledged; overflowing it records an OVERFLOW violation and answers
    error:14 (like the firmware's line overflow)
  * a segment queue (512) and simulated execution time: G0 at the jump speed
    (3000 mm/s), G1/G2/G3 at F; status MPos is interpolated; "Run" while any
    queued segment is still executing; a full queue withholds "ok"
  * realtime: ? ! ~ 0x18 (abort + banner) 0x85 (jog cancel)
  * protocol violations are collected in .violations

Run standalone (prints the pty path, Ctrl-C to quit):
    python3 tools/rftest/mock_grbl.py [--speedup 10] [--slow-ms 5] [--error-on 'G2']
or import and call MockGrbl(...).start() (it runs in daemon threads).
"""
from __future__ import annotations

import argparse
import math
import os
import re
import select
import threading
import time
import tty
from collections import deque
from dataclasses import dataclass
from typing import Optional

from gcode_gen import arc_points

BANNER = "\r\nGrbl 1.1h ['$' for help]\r\n"
LINE_MAX_LEN = 256
JUMP_DELAY = 300e-6           # s ($212)
TICK_US = 10.0

SETTINGS_DEFAULT: dict[int, float] = {
    0: 10, 1: 25, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 12: 0.01, 13: 0, 30: 1000, 31: 0, 32: 1,
    100: 0.1, 101: 0.1, 110: 300000, 111: 300000, 120: 2000000, 121: 2000000, 130: 100,
    131: 100, 140: 0, 141: 0, 142: 0, 143: 0, 144: 0, 150: 0, 151: 0.5, 200: 10, 201: 3000,
    203: 500, 210: 100, 211: 120, 212: 300, 213: 0, 214: 100, 215: 0, 216: 30, 220: 30000,
    221: 50, 223: 0, 224: 0, 225: 100, 226: 1, 227: 4000, 229: 10, 230: 0,
}
RO_SETTINGS = {0, 1, 2, 4, 5, 6, 10, 11, 13, 20, 21, 22, 23, 24, 25, 26, 27, 31, 32, 102, 112, 122, 132}
G_OK = {0, 1, 2, 3, 4, 10, 17, 18, 19, 20, 21, 28, 30, 40, 43, 49, 53, 54, 55, 56, 57, 58, 59, 61,
        64, 80, 90, 91, 92, 94}
M_OK = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 30, 62, 63, 64, 65, 66, 67, 68, 69}
WORD_RE = re.compile(r"([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")


@dataclass
class Seg:
    t0: float
    t1: float
    x0: float
    y0: float
    x1: float
    y1: float


class MockGrbl:
    def __init__(self, slow_ms: float = 0.0, error_on: Optional[str] = None, speedup: float = 1.0,
                 rx_size: int = 1024, queue_len: int = 512, underrun_gap: float = 0.15,
                 work_area: tuple[float, float] = (100.0, 100.0), verbose: bool = False):
        self.slow = slow_ms / 1000.0
        self.error_re = re.compile(error_on) if error_on else None
        self.speedup = max(speedup, 1e-6)
        self.rx_size, self.queue_len, self.underrun_gap = rx_size, queue_len, underrun_gap
        self.work = work_area
        self.verbose = verbose
        self.violations: list[str] = []
        self.settings = dict(SETTINGS_DEFAULT)
        self.settings[130], self.settings[131] = work_area
        self.path = ""
        self._master = self._slave = -1
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._wlock = threading.Lock()
        self._mlock = threading.RLock()
        self._acklock = threading.Lock()
        self._lines: "deque[tuple]" = deque()
        self._lines_cv = threading.Condition()
        self._gen = 0
        self._held = False
        self._held_total = 0.0
        self._held_clock = 0.0
        self._status_count = 0
        self.lines_received = 0
        self.replies_sent = 0
        self.max_occupancy = 0
        self._rx_total = 0
        self._acked = 0
        self.chunks_ticks = 0.0
        self.underruns = 0
        self.barriers = 0
        self.clamped = 0
        self.armed = False
        self._reset_machine(initial=True)

    # ----------------------------------------------------------- lifecycle
    def start(self) -> str:
        self._master, self._slave = os.openpty()
        tty.setraw(self._slave)
        tty.setraw(self._master)
        self.path = os.ttyname(self._slave)
        self._write(BANNER)
        for fn in (self._reader_loop, self._proc_loop):
            th = threading.Thread(target=fn, daemon=True, name=fn.__name__)
            th.start()
            self._threads.append(th)
        return self.path

    def stop(self) -> None:
        self._stop.set()
        with self._lines_cv:
            self._lines_cv.notify_all()
        for th in self._threads:
            th.join(timeout=1.0)
        for fd in (self._master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass

    def _violation(self, text: str) -> None:
        self.violations.append(f"{time.monotonic():.3f} {text}")
        if self.verbose:
            print("VIOLATION", text, flush=True)

    # --------------------------------------------------------------- state
    def _reset_machine(self, initial: bool = False) -> None:
        with self._mlock:
            self.segs: deque[Seg] = deque()
            if initial:
                self.x = self.y = 0.0
            self.last_end = 0.0
            self.motion, self.relative, self.inches = 0, False, False
            self.laser, self.preview = 0, False
            self.s, self.feed = 0.0, 0.0
            self.guide = False
            self.jog_until = 0.0
            self._prev_mark = False
            self._wco_dirty = True

    def _clock(self) -> float:
        return self._held_clock if self._held else time.monotonic() - self._held_total

    def _position(self, now: float) -> tuple[float, float]:
        while self.segs and self.segs[0].t1 <= now:
            s = self.segs.popleft()
            self.x, self.y = s.x1, s.y1
        if self.segs:
            s = self.segs[0]
            f = 0.0 if s.t1 <= s.t0 else min(max((now - s.t0) / (s.t1 - s.t0), 0.0), 1.0)
            return s.x0 + (s.x1 - s.x0) * f, s.y0 + (s.y1 - s.y0) * f
        return self.x, self.y

    def _end_point(self) -> tuple[float, float]:
        return (self.segs[-1].x1, self.segs[-1].y1) if self.segs else (self.x, self.y)

    # ------------------------------------------------------------ writing
    def _write(self, text: str) -> None:
        data = text.encode()
        with self._wlock:
            while data and not self._stop.is_set():
                try:
                    n = os.write(self._master, data)
                except BlockingIOError:
                    time.sleep(0.001)
                    continue
                except OSError:
                    return
                data = data[n:]

    def _reply(self, err: int, gen: int) -> None:
        with self._lines_cv:
            if gen != self._gen:
                return
            self.replies_sent += 1
        self._write("ok\r\n" if err == 0 else f"error:{err}\r\n")

    def _free(self, n: int, gen: int) -> None:
        with self._acklock:
            if gen == self._gen:
                self._acked += n

    # ------------------------------------------------------- reader thread
    def _reader_loop(self) -> None:
        buf = bytearray()
        last_cr = overflow = False
        while not self._stop.is_set():
            r, _, _ = select.select([self._master], [], [], 0.05)
            if not r:
                continue
            try:
                data = os.read(self._master, 4096)
            except OSError:
                time.sleep(0.01)
                continue
            for c in data:
                if c == 0x3F:
                    self._write(self._status_text())
                elif c == 0x21:
                    self._hold(True)
                elif c == 0x7E:
                    self._hold(False)
                elif c == 0x18:
                    self._do_reset()
                    buf.clear()
                    last_cr = overflow = False
                elif c == 0x85:
                    self._abort_motion()
                else:
                    with self._acklock:
                        self._rx_total += 1
                        occ = self._rx_total - self._acked
                    self.max_occupancy = max(self.max_occupancy, occ)
                    if occ > self.rx_size and not overflow:
                        overflow = True
                        self._violation(f"OVERFLOW rx occupancy {occ} > {self.rx_size}")
                    if c == 0x0A and last_cr:           # second half of CRLF
                        last_cr = False
                        self._free(1, self._gen)
                        continue
                    last_cr = c == 0x0D
                    if c in (0x0A, 0x0D):
                        too_long = len(buf) >= LINE_MAX_LEN - 1
                        if too_long:
                            self._violation(f"LONG_LINE {len(buf)} chars")
                        with self._lines_cv:
                            self.lines_received += 1
                            self._lines.append((self._gen, bytes(buf).decode("latin-1"),
                                                len(buf) + 1, overflow or too_long))
                            self._lines_cv.notify()
                        buf.clear()
                        overflow = False
                    else:
                        buf.append(c)

    def _do_reset(self) -> None:
        with self._lines_cv, self._acklock:
            self._gen += 1
            self._lines.clear()
            self._rx_total = self._acked = 0
            self.lines_received = self.replies_sent = 0
            self._lines_cv.notify_all()
        self._abort_motion()
        with self._mlock:
            if self.preview:
                self.guide = False
            self.motion, self.relative, self.inches = 0, False, False
            self.laser, self.preview, self.s = 0, False, 0.0
            self._held = False
            self._prev_mark = False
        self._write(BANNER)

    def _abort_motion(self) -> None:
        with self._mlock:
            x, y = self._position(self._clock())
            self.segs.clear()
            self.x, self.y = x, y
            self.last_end = 0.0
            self.jog_until = 0.0

    def _hold(self, on: bool) -> None:
        with self._mlock:
            if on and not self._held:
                self._held_clock = self._clock()
                self._held = True
            elif not on and self._held:
                self._held_total = time.monotonic() - self._held_clock
                self._held = False

    def _status_text(self) -> str:
        with self._mlock:
            now = self._clock()
            x, y = self._position(now)
            busy = bool(self.segs)
            if self._held:
                state = "Hold:0"
            elif self.jog_until > now and busy:
                state = "Jog"
            else:
                state = "Run" if busy else "Idle"
            blocks = self.queue_len - len(self.segs)
            feed = int(round(self.feed if self.feed > 0 else self.settings[203] * 60))
            wco = ""
            self._status_count += 1
            if self._wco_dirty or self._status_count % 10 == 0:
                self._wco_dirty = False
                wco = "|WCO:0.000,0.000,0.000"
            s = int(round(self.s))
        rxfree = max(0, min(self.rx_size, self.rx_size - (self._rx_total - self._acked)))
        return f"<{state}|MPos:{x:.3f},{y:.3f},0.000|FS:{feed},{s}|Bf:{blocks},{rxfree}{wco}>\r\n"

    # ---------------------------------------------------- processor thread
    def _proc_loop(self) -> None:
        while not self._stop.is_set():
            with self._lines_cv:
                while not self._lines and not self._stop.is_set():
                    self._lines_cv.wait(0.05)
                if self._stop.is_set():
                    return
                gen, text, n, bad = self._lines.popleft()
            if gen != self._gen:
                continue
            if self.slow:
                time.sleep(self.slow)
            err = self._process(text, bad, gen)
            if gen != self._gen:
                continue
            self._free(n, gen)            # an ok means the line's bytes are free again
            self._reply(err, gen)

    def _process(self, text: str, bad: bool, gen: int) -> int:
        if bad:
            return 14
        if self.error_re and self.error_re.search(text):
            return 33
        line = text.strip().upper()
        if line.startswith("$"):
            return self._dollar(line, gen)
        return self._gcode(line, gen, jog=False)

    # ------------------------------------------------------------ $ cmds
    def _dollar(self, line: str, gen: int) -> int:
        line = line.rstrip()
        if line == "$":
            self._write("[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $SLP $C $X $H ~ ! ? ctrl-x]\r\n")
        elif line == "$$":
            self._write("".join(f"${k}={v:g}\r\n" for k, v in sorted(self.settings.items())))
        elif line == "$I":
            self._write(f"[VER:1.1h.WaveMaster:]\r\n[OPT:V,{self.queue_len},{self.rx_size}]\r\n"
                        "[MSG:machine:WaveMaster Galvo]\r\n")
        elif line == "$G":
            self._write(f"[GC:G{self.motion} G54 G17 G21 G90 G94 M{self.laser} M9 T0 "
                        f"F{self.feed:g} S{self.s:g}]\r\n")
        elif line == "$#":
            self._write("".join(f"[G{54 + i}:0.000,0.000,0.000]\r\n" for i in range(6)))
        elif line in ("$X", "$H", "$C", "$SLP"):
            pass
        elif line == "$N":
            self._write("$N0=\r\n$N1=\r\n")
        elif line == "$RB":
            with self._mlock:
                x, y = self._position(self._clock())
                busy = bool(self.segs)
            if busy:
                self._write("[MSG:RB unavailable (busy or SPI error)]\r\n")
            else:
                cx = int(0x8000 + (x - self.work[0] / 2) * self.settings[100] / 10 * 0x7FFF) & 0xFFFF
                cy = int(0x8000 + (y - self.work[1] / 2) * self.settings[101] / 10 * 0x7FFF) & 0xFFFF
                self._write(f"[MSG:RB X=0x{cx:04X} Y=0x{cy:04X}]\r\n")
        elif line == "$S":
            self._write(self._stats_text())
        elif line.startswith("$J="):
            return self._gcode(line[3:], gen, jog=True)
        elif "=" in line:
            m = re.match(r"\$(\d+)\s*=\s*([-+]?[\d.]+)$", line)
            if not m:
                return 3
            n, v = int(m.group(1)), float(m.group(2))
            if n in self.settings and n not in RO_SETTINGS:
                self.settings[n] = v
            elif n not in RO_SETTINGS:
                return 3
        else:
            return 3
        return 0

    def _stats_text(self) -> str:
        with self._mlock:
            ticks = int(self.chunks_ticks / (TICK_US * 1e-6))
            power = 0 if self.preview or self.laser not in (3, 4) else int(self.s * 255 / 1000)
            return (f"[MSG:galvo chunks={ticks // 2048} underruns={self.underruns} "
                    f"barriers={self.barriers} ticks={ticks} tick_us={TICK_US:.2f}]\r\n"
                    f"[MSG:atmega link=1 armed={int(self.armed)} ready={int(self.armed)} "
                    f"guide={int(self.guide)} power={power} stat=0x00 vdet=5000mV err=0]\r\n"
                    f"[MSG:host rx_dropped=0 clamped={self.clamped}]\r\n"
                    f"[MSG:io gate=0 kill=0 gate_active_low=0 sync_hz={self.sync_hz()}]\r\n")

    def sync_hz(self) -> int:
        """What the firmware's laser_io reports as SYNC/PRR frequency ($220 is applied at once when idle).
        `sync_hz_stuck` (tests) simulates a firmware that does not apply it."""
        v = getattr(self, "sync_hz_stuck", None)
        return int(self.settings.get(220, 0) if v is None else v)

    # ------------------------------------------------------------ G-code
    def _gcode(self, line: str, gen: int, jog: bool) -> int:
        line = re.sub(r"\([^)]*\)", "", line.split(";", 1)[0]).strip()
        words = WORD_RE.findall(line)
        if WORD_RE.sub("", line).strip():
            return 2
        w: dict[str, float] = {}
        g_codes: list[float] = []
        m_codes: list[int] = []
        for letter, val in words:
            v = float(val)
            if letter == "G":
                g_codes.append(v)
            elif letter == "M":
                m_codes.append(int(round(v)))
            else:
                w[letter] = v
        motion: Optional[int] = None
        dwell = False
        units = dist = None
        for g in g_codes:
            gi = int(round(g * 10))
            if gi in (0, 10, 20, 30):
                motion = gi // 10
            elif gi == 40:
                dwell = True
            elif gi == 200:
                units = True
            elif gi == 210:
                units = False
            elif gi == 900:
                dist = False
            elif gi == 910:
                dist = True
            elif gi // 10 not in G_OK and gi not in (921, 922, 923):
                return 20
        if any(m not in M_OK for m in m_codes):
            return 20
        with self._mlock:
            if units is not None:
                self.inches = units
            if dist is not None:
                self.relative = dist
            k = 25.4 if self.inches else 1.0
            if w.get("F", 0) > 0:
                self.feed = w["F"] * k
            if "S" in w:
                new_s = max(w["S"], 0.0)
                if self.laser in (3, 4) and not self.preview and new_s != self.s:
                    self.barriers += 1
                self.s = new_s
            for m in m_codes:
                self._mcode(m)
            if jog:
                if "X" not in w and "Y" not in w:
                    return 33
                motion = 0
            if dwell:
                if w.get("P", 0) > 0:
                    p = self._end_point()
                    self._submit(p, p, w["P"], False, gen)
                return 0
            if motion is not None and not jog:
                self.motion = motion
            mo = self.motion if motion is None else motion
            if "X" in w or "Y" in w or (mo in (2, 3) and ("I" in w or "J" in w or "R" in w)):
                return self._move(mo, w, k, jog, gen)
        return 0

    def _mcode(self, m: int) -> None:
        if m in (3, 4, 5):
            self.laser = m
        elif m == 10:
            self.armed = True
        elif m == 11:
            self.armed = False
        elif m == 62:
            self.guide = True
        elif m == 63:
            self.guide = False
        elif m == 66:
            self.preview = self.guide = True
        elif m == 67:
            self.preview = self.guide = False

    def _clamp(self, x: float, y: float) -> tuple[float, float]:
        w, h = self.settings[130], self.settings[131]
        if self.settings.get(144):                    # centred origin: -W/2..W/2
            cx = min(max(x, -w / 2), w / 2)
            cy = min(max(y, -h / 2), h / 2)
        else:
            cx = min(max(x, 0.0), w)
            cy = min(max(y, 0.0), h)
        if (cx, cy) != (x, y):
            self.clamped += 1
        return cx, cy

    def _move(self, mo: int, w: dict[str, float], k: float, jog: bool, gen: int) -> int:
        sx, sy = self._end_point()
        if self.relative:
            tx, ty = sx + w.get("X", 0.0) * k, sy + w.get("Y", 0.0) * k
        else:
            tx, ty = (w["X"] * k if "X" in w else sx), (w["Y"] * k if "Y" in w else sy)
        tx, ty = self._clamp(tx, ty)
        d = math.hypot(tx - sx, ty - sy)
        if mo == 0:
            if d > 1e-5:
                dur = d / self.settings[201] + JUMP_DELAY
                if self._submit((sx, sy), (tx, ty), dur, False, gen) and jog:
                    self.jog_until = self._clock() + dur / self.speedup
            return 0
        speed = self.feed / 60.0 if self.feed > 0 else self.settings[203]
        speed = max(min(speed, self.settings[110] / 60.0), 1e-3)
        if mo == 1:
            if d > 1e-5:
                self._submit((sx, sy), (tx, ty), d / speed, True, gen)
            return 0
        if "R" in w and "I" not in w and "J" not in w:
            return 33
        i, j = w.get("I", 0.0) * k, w.get("J", 0.0) * k
        if math.hypot(i, j) < 1e-9:
            return 33
        prev = (sx, sy)
        for px, py in arc_points(sx, sy, tx, ty, i, j, mo == 2, self.settings[12]):
            px, py = self._clamp(px, py)
            dd = math.hypot(px - prev[0], py - prev[1])
            if dd > 1e-9 and not self._submit(prev, (px, py), dd / speed, True, gen):
                return 0
            prev = (px, py)
        return 0

    def _submit(self, p0, p1, dur_virtual: float, mark: bool, gen: int) -> bool:
        """Queue one segment; blocks (withholding ok) while the queue is full."""
        while True:
            with self._mlock:
                if gen != self._gen:
                    return False
                now = self._clock()
                self._position(now)
                if len(self.segs) < self.queue_len:
                    t0 = max(now, self.last_end) if self.segs else now
                    if (mark and self._prev_mark and not self.segs and not self._held
                            and now - self.last_end > self.underrun_gap):
                        self.underruns += 1
                    t1 = t0 + dur_virtual / self.speedup
                    self.segs.append(Seg(t0, t1, p0[0], p0[1], p1[0], p1[1]))
                    self.last_end = t1
                    self.chunks_ticks += dur_virtual
                    self._prev_mark = mark
                    return True
                wait = 0.005 if self._held else max(0.0005, min(self.segs[0].t1 - now, 0.01))
            time.sleep(wait)

    # --------------------------------------------------------------- checks
    def check_counts(self, timeout: float = 2.0) -> bool:
        """True if every received line was answered (records a violation if not)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self._lines_cv:
                if not self._lines and self.lines_received == self.replies_sent:
                    return True
            time.sleep(0.01)
        self._violation(f"OK_COUNT_MISMATCH lines={self.lines_received} replies={self.replies_sent}")
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--slow-ms", type=float, default=0.0, help="extra delay per line before its ok")
    ap.add_argument("--error-on", help="regex: lines matching it are answered with error:33")
    ap.add_argument("--speedup", type=float, default=1.0, help="run simulated motion N times faster")
    ap.add_argument("--rx-size", type=int, default=1024)
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    dev = MockGrbl(a.slow_ms, a.error_on, a.speedup, a.rx_size, verbose=a.verbose)
    print(dev.start(), flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        dev.stop()
        for v in dev.violations:
            print("VIOLATION", v)
    return 1 if dev.violations else 0


if __name__ == "__main__":
    raise SystemExit(main())
