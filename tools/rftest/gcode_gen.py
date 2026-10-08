"""
G-code generation that mimics Rayforge's encoders, plus a shape library.

Two dialects, as Rayforge emits them:

  "grbl"         (Compat) G0 travel without F/S; the laser is switched on lazily
                 at the first cut with "M4 S<p>"; every G1 carries F; a power
                 change while the laser is active re-emits "M4 S<p>"; "M5" ends
                 each cut before the next travel.
  "grbl_raster"  "M4 S0" once at the first laser-on; every G0/G1 carries S
                 (G0 gets " S0"); F only when it changed (modal); M5 only in
                 the postamble.

The internal representation is a list of ops (MoveTo, LineTo, ArcTo, SetPower,
SetSpeed, ScanLine).  All shapes are produced in machine millimetres inside a
Box that the caller places in the work area.

Pure stdlib, no global state.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence, Union

Point = tuple[float, float]

PREAMBLE = ["G21 ;Set units to mm", "G90 ;Absolute positioning", "G54"]
POSTAMBLE = ["M5 ;Ensure laser is off", "G0 X0 Y0 ;Return to origin"]
DIALECTS = ("grbl", "grbl_raster")


def fmt_num(v: float) -> str:
    """Three decimals, trailing zeros stripped ("10", "2.5", "-0.125")."""
    s = f"{v:.3f}".rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


# ------------------------------------------------------------------ ops ----

@dataclass(frozen=True)
class MoveTo:
    x: float
    y: float


@dataclass(frozen=True)
class LineTo:
    x: float
    y: float


@dataclass(frozen=True)
class ArcTo:
    """Arc from the current point to (x, y); i/j = centre offset from the START."""
    x: float
    y: float
    i: float
    j: float
    cw: bool = True


@dataclass(frozen=True)
class SetPower:
    power: float          # S units (0..$30)


@dataclass(frozen=True)
class SetSpeed:
    feed: float           # mm/min


@dataclass(frozen=True)
class ScanLine:
    """One raster row: 8-bit pixel powers starting at x0, pixel_w mm wide each."""
    y: float
    x0: float
    powers: tuple[int, ...]
    pixel_w: float
    overscan: float = 0.0
    direction: int = 1            # +1 left->right, -1 right->left
    max_power: float = 1000.0     # S for pixel value 255


Op = Union[MoveTo, LineTo, ArcTo, SetPower, SetSpeed, ScanLine]


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    w: float
    h: float

    @property
    def x1(self) -> float:
        return self.x + self.w

    @property
    def y1(self) -> float:
        return self.y + self.h

    @property
    def cx(self) -> float:
        return self.x + self.w / 2

    @property
    def cy(self) -> float:
        return self.y + self.h / 2

    def union(self, o: "Box") -> "Box":
        x0, y0 = min(self.x, o.x), min(self.y, o.y)
        return Box(x0, y0, max(self.x1, o.x1) - x0, max(self.y1, o.y1) - y0)

    def inside(self, w: float, h: float, eps: float = 1e-6) -> bool:
        return self.x >= -eps and self.y >= -eps and self.x1 <= w + eps and self.y1 <= h + eps


# ---------------------------------------------------------- geometry -------

def arc_points(x0: float, y0: float, x1: float, y1: float, i: float, j: float,
               cw: bool, tol: float = 0.01) -> list[Point]:
    """Chord-approximate an arc; returns the points after the start point."""
    cx, cy = x0 + i, y0 + j
    r = math.hypot(x0 - cx, y0 - cy)
    a0 = math.atan2(y0 - cy, x0 - cx)
    a1 = math.atan2(y1 - cy, x1 - cx)
    da = a1 - a0
    if cw:
        while da >= -1e-9:
            da -= 2 * math.pi
    else:
        while da <= 1e-9:
            da += 2 * math.pi
    if r < 1e-9:
        return [(x1, y1)]
    step = 2 * math.acos(max(-1.0, min(1.0, 1 - tol / r))) if tol < r else math.pi / 2
    n = max(2, int(math.ceil(abs(da) / max(step, 1e-3))))
    pts = [(cx + r * math.cos(a0 + da * k / n), cy + r * math.sin(a0 + da * k / n))
           for k in range(1, n)]
    pts.append((x1, y1))
    return pts


def bbox(ops: Sequence[Op], tol: float = 0.01) -> Optional[Box]:
    """Bounding box of everything the ops touch (arcs sampled, overscan included)."""
    xs: list[float] = []
    ys: list[float] = []
    cur: Optional[Point] = None
    for op in ops:
        if isinstance(op, (MoveTo, LineTo)):
            xs.append(op.x), ys.append(op.y)
            cur = (op.x, op.y)
        elif isinstance(op, ArcTo) and cur is not None:
            for px, py in arc_points(cur[0], cur[1], op.x, op.y, op.i, op.j, op.cw, tol):
                xs.append(px), ys.append(py)
            cur = (op.x, op.y)
        elif isinstance(op, ScanLine):
            n = len(op.powers)
            xs += [op.x0 - op.overscan, op.x0 + n * op.pixel_w + op.overscan]
            ys += [op.y, op.y]
    if not xs:
        return None
    return Box(min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))


def path_length(ops: Sequence[Op]) -> tuple[float, float]:
    """(cut_mm, travel_mm) estimate, useful for run-time estimates."""
    cut = travel = 0.0
    cur: Optional[Point] = None
    for op in expand_all(ops):
        if isinstance(op, MoveTo):
            if cur:
                travel += math.dist(cur, (op.x, op.y))
            cur = (op.x, op.y)
        elif isinstance(op, LineTo):
            if cur:
                cut += math.dist(cur, (op.x, op.y))
            cur = (op.x, op.y)
        elif isinstance(op, ArcTo) and cur:
            pts = [cur] + arc_points(cur[0], cur[1], op.x, op.y, op.i, op.j, op.cw)
            cut += sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
            cur = (op.x, op.y)
    return cut, travel


# ------------------------------------------------------- scanline ----------

def expand_scanline(op: ScanLine) -> list[Op]:
    """Linearise a pixel row into one LineTo per RUN of constant power.

    Overscan lead-in/out at S0 on both sides; merged with adjacent S0 runs.
    A row without any lit pixel produces nothing.
    """
    n = len(op.powers)
    if n == 0 or not any(op.powers):
        return []
    d = 1 if op.direction >= 0 else -1
    pix = list(op.powers) if d > 0 else list(reversed(op.powers))
    start_edge = op.x0 if d > 0 else op.x0 + n * op.pixel_w
    end_edge = start_edge + d * n * op.pixel_w

    runs: list[list[float]] = []          # [S, end_x]

    def add(s: float, x: float) -> None:
        if runs and runs[-1][0] == s:
            runs[-1][1] = x
        else:
            runs.append([s, x])

    if op.overscan > 0:
        add(0, start_edge)
    k = 0
    while k < n:
        e = k
        while e + 1 < n and pix[e + 1] == pix[k]:
            e += 1
        s = round(pix[k] / 255.0 * op.max_power)
        add(s, start_edge + d * (e + 1) * op.pixel_w)
        k = e + 1
    if op.overscan > 0:
        add(0, end_edge + d * op.overscan)

    out: list[Op] = [MoveTo(start_edge - d * op.overscan, op.y)]
    for s, x in runs:
        out.append(SetPower(s))
        out.append(LineTo(x, op.y))
    return out


def expand_all(ops: Sequence[Op]) -> list[Op]:
    out: list[Op] = []
    for op in ops:
        out.extend(expand_scanline(op) if isinstance(op, ScanLine) else [op])
    return out


# ------------------------------------------------------------ encoder ------

@dataclass
class EncoderOptions:
    dialect: str = "grbl"
    arcs: bool = True
    modal_feed: bool = False                 # "grbl": emit F only when it changed
    max_s: float = 1000.0                    # hard cap applied to every S
    work_area: Optional[tuple[float, float]] = (100.0, 100.0)   # clamp; None = off
    arc_tol: float = 0.01
    preamble: bool = True
    postamble: bool = True


@dataclass
class EncodeStats:
    clamped_coords: int = 0
    clamped_power: int = 0
    lines: int = 0


@dataclass
class Gcode:
    text: str
    stats: EncodeStats

    @property
    def lines(self) -> list[str]:
        return self.text.splitlines()


class GcodeEncoder:
    """Stateful op -> G-code encoder (one instance per job)."""

    def __init__(self, opts: EncoderOptions):
        if opts.dialect not in DIALECTS:
            raise ValueError(f"unknown dialect {opts.dialect!r}")
        self.o = opts
        self.raster = opts.dialect == "grbl_raster"
        self.out: list[str] = []
        self.stats = EncodeStats()
        self.pos: Optional[Point] = None          # logical position (clamped)
        self._lx: Optional[str] = None            # last emitted X / Y text
        self._ly: Optional[str] = None
        self.feed = 0.0
        self._emitted_feed: Optional[str] = None
        self.power = 0.0                          # current S (already capped)
        self._emitted_s: Optional[int] = None     # S of the last "M4 S.."
        self.laser_active = False
        self._raster_on = False

    # ---- helpers
    def _clamp(self, x: float, y: float) -> Point:
        wa = self.o.work_area
        if wa is None:
            return x, y
        cx, cy = min(max(x, 0.0), wa[0]), min(max(y, 0.0), wa[1])
        if (cx, cy) != (x, y):
            self.stats.clamped_coords += 1
        return cx, cy

    def _cap(self, p: float) -> int:
        s = int(round(max(p, 0.0)))
        cap = int(self.o.max_s)
        if s > cap:
            self.stats.clamped_power += 1
            s = cap
        return s

    def _coords(self, x: float, y: float) -> str:
        parts = []
        xs, ys = fmt_num(x), fmt_num(y)
        if xs != self._lx:
            parts.append("X" + xs)
        if ys != self._ly:
            parts.append("Y" + ys)
        self._lx, self._ly = xs, ys
        return " ".join(parts)

    def _feed_word(self) -> str:
        f = fmt_num(self.feed)
        if self.raster or self.o.modal_feed:
            if f == self._emitted_feed:
                return ""
        self._emitted_feed = f
        return "F" + f if self.feed > 0 else ""

    @staticmethod
    def _join(*words: str) -> str:
        return " ".join(w for w in words if w)

    # ---- ops
    def op(self, op: Op) -> None:
        if isinstance(op, SetSpeed):
            self.feed = op.feed
        elif isinstance(op, SetPower):
            self._set_power(op.power)
        elif isinstance(op, MoveTo):
            self._move_to(op.x, op.y)
        elif isinstance(op, LineTo):
            self._line_to(op.x, op.y)
        elif isinstance(op, ArcTo):
            self._arc_to(op)
        elif isinstance(op, ScanLine):
            for sub in expand_scanline(op):
                self.op(sub)
        else:                                      # pragma: no cover
            raise TypeError(op)

    def _set_power(self, p: float) -> None:
        self.power = self._cap(p)
        if self.laser_active and not self.raster and self.power != self._emitted_s:
            self.out.append(f"M4 S{self.power}")
            self._emitted_s = self.power

    def _laser_on(self) -> None:
        if self.laser_active:
            return
        self.laser_active = True
        if self.raster:
            if not self._raster_on:
                self._raster_on = True
                self.out.append("M4 S0")
        else:
            self.out.append(f"M4 S{self.power}")
            self._emitted_s = self.power

    def _laser_off(self) -> None:
        if self.laser_active and not self.raster:
            self.out.append("M5")
        self.laser_active = False
        self._emitted_s = None

    def _move_to(self, x: float, y: float) -> None:
        self._laser_off()
        x, y = self._clamp(x, y)
        if self.pos is not None and (fmt_num(x), fmt_num(y)) == (fmt_num(self.pos[0]), fmt_num(self.pos[1])):
            return                                  # zero-length travel dropped
        self.out.append(self._join("G0", self._coords(x, y), "S0" if self.raster else ""))
        self.pos = (x, y)

    def _line_to(self, x: float, y: float) -> None:
        x, y = self._clamp(x, y)
        if self.pos is not None and (fmt_num(x), fmt_num(y)) == (fmt_num(self.pos[0]), fmt_num(self.pos[1])):
            return
        self._laser_on()
        s = f"S{self.power}" if self.raster else ""
        self.out.append(self._join("G1", self._coords(x, y), s, self._feed_word()))
        self.pos = (x, y)

    def _arc_to(self, a: ArcTo) -> None:
        if self.pos is None:
            raise ValueError("arc without a known start point")
        if not self.o.arcs:
            x0, y0 = self.pos
            for px, py in arc_points(x0, y0, a.x, a.y, a.i, a.j, a.cw, self.o.arc_tol):
                self._line_to(px, py)
            return
        x, y = self._clamp(a.x, a.y)
        self._laser_on()
        xs, ys = fmt_num(x), fmt_num(y)
        self._lx, self._ly = xs, ys
        s = f"S{self.power}" if self.raster else ""
        self.out.append(self._join("G2" if a.cw else "G3", "X" + xs, "Y" + ys,
                                   "I" + fmt_num(a.i), "J" + fmt_num(a.j), s, self._feed_word()))
        self.pos = (x, y)

    # ---- whole job
    def encode(self, ops: Sequence[Op]) -> Gcode:
        if self.o.preamble:
            self.out.extend(PREAMBLE)
            self._lx = self._ly = None
            self.pos = None
        for op in ops:
            self.op(op)
        if self.o.postamble:
            self.out.extend(POSTAMBLE)
        self.stats.lines = len(self.out)
        return Gcode("\n".join(self.out) + "\n", self.stats)


def encode(ops: Sequence[Op], dialect: str = "grbl", **kw) -> Gcode:
    """Convenience: encode(ops, "grbl", max_s=300, work_area=(100, 100))."""
    return GcodeEncoder(EncoderOptions(dialect=dialect, **kw)).encode(ops)


# ------------------------------------------------------ shape helpers ------

def polyline(points: Sequence[Point], close: bool = False) -> list[Op]:
    if not points:
        return []
    ops: list[Op] = [MoveTo(*points[0])]
    ops += [LineTo(*p) for p in points[1:]]
    if close:
        ops.append(LineTo(*points[0]))
    return ops


def _head(feed: float, power: float) -> list[Op]:
    return [SetSpeed(feed), SetPower(power)]


def circle_arcs(cx: float, cy: float, r: float, cw: bool = True) -> list[Op]:
    """Full circle as two half arcs (G2/G3), starting at the +X point."""
    return [MoveTo(cx + r, cy), ArcTo(cx - r, cy, -r, 0.0, cw), ArcTo(cx + r, cy, r, 0.0, cw)]


def circle_points(cx: float, cy: float, r: float, n: int) -> list[Point]:
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n))
            for k in range(n)]


def regular_polygon(cx: float, cy: float, r: float, sides: int, rot: float = -math.pi / 2) -> list[Point]:
    return [(cx + r * math.cos(rot + 2 * math.pi * k / sides),
             cy + r * math.sin(rot + 2 * math.pi * k / sides)) for k in range(sides)]


def frame_ops(b: Box, feed: float, power: float = 0.0) -> list[Op]:
    return _head(feed, power) + polyline([(b.x, b.y), (b.x1, b.y), (b.x1, b.y1), (b.x, b.y1)], close=True)


# ---------------------------------------------------------- shapes ---------
# Every shape: (box, feed, power, **options) -> list[Op], inside `box`.

def square(box: Box, feed: float, power: float) -> list[Op]:
    s = min(box.w, box.h)
    x, y = box.cx - s / 2, box.cy - s / 2
    return _head(feed, power) + polyline([(x, y), (x + s, y), (x + s, y + s), (x, y + s)], close=True)


def rectangle(box: Box, feed: float, power: float, aspect: float = 0.6) -> list[Op]:
    h = box.h * aspect
    y = box.cy - h / 2
    return _head(feed, power) + polyline([(box.x, y), (box.x1, y), (box.x1, y + h), (box.x, y + h)], close=True)


def circle_poly(box: Box, feed: float, power: float, segments: int = 64) -> list[Op]:
    r = min(box.w, box.h) / 2
    return _head(feed, power) + polyline(circle_points(box.cx, box.cy, r, segments), close=True)


def circle_arc(box: Box, feed: float, power: float) -> list[Op]:
    return _head(feed, power) + circle_arcs(box.cx, box.cy, min(box.w, box.h) / 2)


def star(box: Box, feed: float, power: float, points: int = 5) -> list[Op]:
    r = min(box.w, box.h) / 2
    verts = regular_polygon(box.cx, box.cy, r, points)
    order = [(k * 2) % points for k in range(points)] if points % 2 else list(range(points))
    return _head(feed, power) + polyline([verts[k] for k in order], close=True)


def spiral(box: Box, feed: float, power: float, turns: float = 6, pts_per_turn: int = 72) -> list[Op]:
    r_max = min(box.w, box.h) / 2
    n = int(turns * pts_per_turn)
    pts = []
    for k in range(n + 1):
        t = k / n
        a = 2 * math.pi * turns * t
        r = r_max * (0.02 + 0.98 * t)
        pts.append((box.cx + r * math.cos(a), box.cy + r * math.sin(a)))
    return _head(feed, power) + polyline(pts)


def polygon_ladder(box: Box, feed: float, power: float, cols: int = 3) -> list[Op]:
    sides = list(range(3, 9))
    rows = math.ceil(len(sides) / cols)
    cw, ch = box.w / cols, box.h / rows
    r = min(cw, ch) / 2 * 0.85
    ops = _head(feed, power)
    for k, n in enumerate(sides):
        cx, cy = box.x + (k % cols + 0.5) * cw, box.y + (k // cols + 0.5) * ch
        ops += polyline(regular_polygon(cx, cy, r, n), close=True)
    return ops


def target(box: Box, feed: float, power: float, rings: int = 5) -> list[Op]:
    r = min(box.w, box.h) / 2
    ops = _head(feed, power)
    ops += polyline([(box.cx - r, box.cy), (box.cx + r, box.cy)])
    ops += polyline([(box.cx, box.cy - r), (box.cx, box.cy + r)])
    for k in range(1, rings + 1):
        ops += circle_arcs(box.cx, box.cy, r * k / rings)
    return ops


def line_grid(box: Box, feed: float, power: float, rows: int = 10, cols: int = 6,
              length_fracs: Sequence[float] = (0.15, 0.3, 0.45, 0.6, 0.75, 0.9)) -> list[Op]:
    """Rows of short marks (laser on/off delay tuning); length varies by column."""
    px, py = box.w / cols, box.h / max(rows - 1, 1)
    ops = _head(feed, power)
    for r in range(rows):
        for c in range(cols):
            x = box.x + c * px
            y = box.y + r * py
            ops += [MoveTo(x, y), LineTo(x + px * length_fracs[c % len(length_fracs)], y)]
    return ops


def corner_test(box: Box, feed: float, power: float,
                angles: Sequence[float] = (15, 30, 45, 60, 90, 120, 150, 170), cols: int = 4) -> list[Op]:
    """Sharp 'V' corners with the given apex angles (degrees)."""
    rows = math.ceil(len(angles) / cols)
    cw, ch = box.w / cols, box.h / rows
    leg = 0.45 * min(cw, ch)
    ops = _head(feed, power)
    for k, ang in enumerate(angles):
        cx, cy = box.x + (k % cols + 0.5) * cw, box.y + (k // cols + 0.5) * ch
        half = math.radians(ang) / 2
        dx, dy = leg * math.sin(half), leg * math.cos(half)
        ops += polyline([(cx - dx, cy + dy / 2), (cx, cy - dy / 2), (cx + dx, cy + dy / 2)])
    return ops


def hatch_lines(x0: float, y0: float, w: float, h: float, spacing: float,
                bidirectional: bool = True) -> list[Op]:
    ops: list[Op] = []
    n = int(h / spacing + 1e-9) + 1
    for k in range(n):
        y = y0 + k * spacing
        a, b = (x0, x0 + w)
        if bidirectional and k % 2:
            a, b = b, a
        ops += [MoveTo(a, y), LineTo(b, y)]
    return ops


def hatch_square(box: Box, feed: float, power: float, spacing: float = 0.25,
                 bidirectional: bool = True) -> list[Op]:
    s = min(box.w, box.h)
    return _head(feed, power) + hatch_lines(box.cx - s / 2, box.cy - s / 2, s, s, spacing, bidirectional)


def _patch_grid(box: Box, n: int, cols: int, gap: float) -> list[Box]:
    cols = max(1, min(cols, n))
    rows = math.ceil(n / cols)
    s = min((box.w - gap * (cols - 1)) / cols, (box.h - gap * (rows - 1)) / rows)
    gw = cols * s + (cols - 1) * gap
    gh = rows * s + (rows - 1) * gap
    ox, oy = box.cx - gw / 2, box.cy - gh / 2
    return [Box(ox + (k % cols) * (s + gap), oy + (k // cols) * (s + gap), s, s) for k in range(n)]


def power_ladder(box: Box, feed: float, powers: Sequence[float], spacing: float = 0.25,
                 cols: int = 3, gap: float = 2.0) -> list[Op]:
    """Hatched patches, one per S value; order = left->right, then next row up (+Y)."""
    ops: list[Op] = [SetSpeed(feed)]
    for p, b in zip(powers, _patch_grid(box, len(powers), cols, gap)):
        ops.append(SetPower(p))
        ops += hatch_lines(b.x, b.y, b.w, b.h, spacing)
    return ops


def speed_ladder(box: Box, feeds: Sequence[float], power: float, cols: int = 3, gap: float = 2.0) -> list[Op]:
    """The same square (outline + hatch) at each F; order as in power_ladder."""
    ops: list[Op] = [SetPower(power)]
    for f, b in zip(feeds, _patch_grid(box, len(feeds), cols, gap)):
        ops.append(SetSpeed(f))
        ops += polyline([(b.x, b.y), (b.x1, b.y), (b.x1, b.y1), (b.x, b.y1)], close=True)
        ops += hatch_lines(b.x, b.y, b.w, b.h, b.w / 4)
    return ops


# --------------------------------------------------------- raster ----------

BAYER4 = ((0, 8, 2, 10), (12, 4, 14, 6), (3, 11, 1, 9), (15, 7, 13, 5))


def raster_rows(box: Box, feed: float, max_power: float, rows: int,
                row_pixels: Callable[[int], Sequence[int]], pixel_w: float,
                overscan: float = 2.0, bidirectional: bool = True) -> list[Op]:
    pitch = box.h / max(rows - 1, 1) if rows > 1 else box.h
    ops: list[Op] = [SetSpeed(feed)]
    for r in range(rows):
        d = -1 if (bidirectional and r % 2) else 1
        ops.append(ScanLine(box.y + r * pitch, box.x, tuple(int(v) for v in row_pixels(r)),
                            pixel_w, overscan, d, max_power))
    return ops


def raster_gradient(box: Box, feed: float, max_power: float, cols: int = 64, rows: int = 50,
                    overscan: float = 2.0, bidirectional: bool = True) -> list[Op]:
    """Horizontal ramp 0..255 across `cols` steps."""
    ramp = [round(c * 255 / max(cols - 1, 1)) for c in range(cols)]
    return raster_rows(box, feed, max_power, rows, lambda r: ramp, box.w / cols, overscan, bidirectional)


def raster_dither(box: Box, feed: float, max_power: float, cols: int = 100, rows: int = 50,
                  overscan: float = 2.0, bidirectional: bool = True) -> list[Op]:
    """Ordered Bayer 4x4 dither of the horizontal gradient -> 0/255 pixels."""
    def row(r: int) -> list[int]:
        out = []
        for c in range(cols):
            v = c / max(cols - 1, 1) * 255
            t = (BAYER4[r % 4][c % 4] + 0.5) / 16 * 255
            out.append(255 if v > t else 0)
        return out
    return raster_rows(box, feed, max_power, rows, row, box.w / cols, overscan, bidirectional)


def checkerboard(box: Box, feed: float, max_power: float, cells: int = 8, rows: int = 40,
                 overscan: float = 2.0, bidirectional: bool = True) -> list[Op]:
    def row(r: int) -> list[int]:
        cell_row = min(int(r / rows * cells), cells - 1)
        return [255 if (c + cell_row) % 2 == 0 else 0 for c in range(cells)]
    return raster_rows(box, feed, max_power, rows, row, box.w / cells, overscan, bidirectional)


SHAPES: dict[str, Callable[..., list[Op]]] = {
    "square": square, "rectangle": rectangle, "circle_poly": circle_poly,
    "circle_arc": circle_arc, "star": star, "spiral": spiral,
    "polygon_ladder": polygon_ladder, "target": target, "line_grid": line_grid,
    "corner_test": corner_test, "hatch_square": hatch_square,
}
