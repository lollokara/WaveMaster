"""
Pure math for tools/calibrate.py (no I/O, no serial, stdlib only).

Everything the calibration wizard computes lives here so that it can be unit
tested offline (tools/calibrate_selftest.py).  Conventions:

  * "$N" settings are keyed by their integer number (100, 101, 143, ...).
  * "physical" X is to the operator's RIGHT, physical Y is AWAY from the
    operator (up when looking at the machine from above, facing it).
  * A "direction" is (axis, sign) with axis in {"x", "y"} and sign in {+1, -1}:
    right = ("x", +1), left = ("x", -1), up/away = ("y", +1), down/toward = ("y", -1).

galvo_transform() is a line-by-line Python reimplementation of the machine-mm
-> volts path in src/galvo_out.c (galvo_out_reload + galvo_out_tick):

    x = (X - org_x) * sgn_x        org = (W/2, H/2), or (0, 0) when $144 = 1
    y = (Y - org_y) * sgn_y        sgn = -1 when the $3 bit (bit0 X, bit1 Y) is set
    if $143: x, y = y, x           swap AFTER invert
    f = 1 + k1 (x^2 + y^2)         $142, only when k1 != 0
    V_ch0 = x f * $100 + $140      CH0 = X DAC
    V_ch1 = y f * $101 + $141      CH1 = Y DAC
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Mapping, Optional, Sequence


class CalibError(Exception):
    """A calibration input that cannot be turned into a result."""


# --------------------------------------------------------------- settings --

NAMES: dict[int, str] = {
    3: "axis invert mask (bit0 X, bit1 Y)", 12: "arc tolerance, mm", 30: "S value for 100% power",
    100: "X scale, V/mm", 101: "Y scale, V/mm", 110: "max marking speed, mm/min",
    111: "max marking speed (alias), mm/min", 120: "jump acceleration, mm/s^2",
    130: "work area width, mm", 131: "work area height, mm", 140: "X offset, V", 141: "Y offset, V",
    142: "radial correction k1", 143: "swap X/Y", 144: "origin 0 corner / 1 lens centre",
    150: "wobble diameter, mm", 151: "wobble pitch, mm", 200: "tick period, us", 201: "jump speed, mm/s",
    203: "default marking speed, mm/s", 210: "laser-on delay, us", 211: "laser-off delay, us",
    212: "jump delay, us", 213: "jump delay per mm, us/mm", 214: "mark delay, us",
    215: "polygon delay, us", 216: "polygon angle threshold, deg", 220: "PRR, Hz", 221: "PRR duty, %",
    223: "power mode (0 analog / 1 pulse density)", 224: "power at S=0+, %", 225: "power at S=$30, %",
    226: "auto-arm", 227: "arm timeout, ms", 229: "pulse-density period, ticks", 230: "gate active low",
}

# (min, max) per writable setting - copied from the s_defs table in src/calib.c.
RANGES: dict[int, tuple[float, float]] = {
    3: (0, 3), 12: (0.0005, 1), 30: (1, 100000), 100: (-100, 100), 101: (-100, 100),
    110: (1, 3000000), 120: (0, 1e9), 130: (1, 1000), 131: (1, 1000), 140: (-10, 10), 141: (-10, 10),
    142: (-1, 1), 143: (0, 1), 144: (0, 1), 150: (0, 10), 151: (0.001, 100), 200: (2, 200),
    201: (1, 100000), 203: (0.1, 100000), 210: (0, 100000), 211: (0, 100000), 212: (0, 1e6),
    213: (0, 100000), 214: (0, 1e6), 215: (0, 100000), 216: (0, 180), 220: (0, 2000000), 221: (1, 99),
    223: (0, 1), 224: (0, 100), 225: (0, 100), 226: (0, 1), 227: (0, 60000), 229: (2, 255), 230: (0, 1),
}
WRITABLE = frozenset(RANGES)

# What the `$$` dump can show: calib.c prints "%.4f" (integers without decimals).
DUMP_RESOLUTION = 5e-5


def check_range(n: int, v: float) -> None:
    if n not in RANGES:
        raise CalibError(f"${n} is not a writable calibration setting")
    lo, hi = RANGES[n]
    if not (lo <= v <= hi):
        raise CalibError(f"${n}={v:g} is outside the firmware range [{lo:g}, {hi:g}]")


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def fmt_val(v: float) -> str:
    """Plain decimal for "$N=v" (the firmware parser has no exponent syntax)."""
    s = f"{v:.8f}".rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


def same_value(a: float, b: float) -> bool:
    """Equal as far as a `$$` readback (4 decimals) can tell."""
    return abs(a - b) <= DUMP_RESOLUTION + 1e-6 * abs(b)


# ---------------------------------------------------------------- parsing --

_DUMP_RE = re.compile(r"^\$(\d+)\s*=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)")


def parse_dump(lines: Sequence[str]) -> dict[int, float]:
    """Parse `$$` output ("$100=0.1000") into {100: 0.1}."""
    out: dict[int, float] = {}
    for ln in lines:
        m = _DUMP_RE.match(ln.strip())
        if m:
            out[int(m.group(1))] = float(m.group(2))
    return out


def parse_floats(text: str, n_min: int = 1, n_max: int = 1) -> list[float]:
    """"12.34", "12,34", "20.1 20.2", "20,1; 20,2" -> floats.

    Separators between numbers are whitespace, ';' or '/'.  A comma is a decimal
    comma ("12,34" == 12.34).  Raises CalibError when the count is outside
    [n_min, n_max] or a token is not a number."""
    toks = [t for t in re.split(r"[\s;/]+", text.strip()) if t]
    if not (n_min <= len(toks) <= n_max):
        want = f"{n_min}" if n_min == n_max else f"{n_min} to {n_max}"
        raise CalibError(f"expected {want} number(s), got {len(toks)}")
    out = []
    for t in toks:
        t2 = t.replace(",", ".")
        if not re.fullmatch(r"[-+]?(\d+\.?\d*|\.\d+)", t2):
            raise CalibError(f"{t!r} is not a number")
        out.append(float(t2))
    return out


Direction = tuple[str, int]
DIRS: dict[str, Direction] = {"right": ("x", 1), "left": ("x", -1), "up": ("y", 1), "down": ("y", -1)}
DIR_ALIASES: dict[str, str] = {
    "r": "right", "l": "left", "u": "up", "d": "down", "away": "up", "back": "up", "far": "up",
    "forward": "up", "toward": "down", "towards": "down", "near": "down", "front": "down",
}


def parse_direction(text: str) -> Direction:
    w = text.strip().lower()
    w = DIR_ALIASES.get(w, w)
    if w not in DIRS:
        raise CalibError("answer right, left, up or down (up = away from you)")
    return DIRS[w]


def dir_name(d: Direction) -> str:
    return {v: k for k, v in DIRS.items()}[d]


# ------------------------------------------------------ transform (galvo_out.c)

def galvo_transform(s: Mapping[int, float], x_mm: float, y_mm: float, clamp_volts: bool = True) -> tuple[float, float]:
    """Machine mm -> (V on DAC channel 0, V on DAC channel 1).  See module docstring."""
    w, h = s.get(130, 100.0), s.get(131, 100.0)
    ox, oy = (0.0, 0.0) if s.get(144, 0.0) else (w * 0.5, h * 0.5)
    inv = int(s.get(3, 0))
    x = (x_mm - ox) * (-1.0 if inv & 1 else 1.0)
    y = (y_mm - oy) * (-1.0 if inv & 2 else 1.0)
    if s.get(143, 0.0):
        x, y = y, x
    k1 = s.get(142, 0.0)
    if k1:
        f = 1.0 + k1 * (x * x + y * y)
        x *= f
        y *= f
    v0 = x * s.get(100, 0.1) + s.get(140, 0.0)
    v1 = y * s.get(101, 0.1) + s.get(141, 0.0)
    if clamp_volts:
        v0, v1 = clamp(v0, -10.0, 10.0), clamp(v1, -10.0, 10.0)
    return v0, v1


def field_centre(s: Mapping[int, float]) -> tuple[float, float]:
    """Machine coordinates of the lens centre (V = offset only)."""
    if s.get(144, 0.0):
        return 0.0, 0.0
    return s.get(130, 100.0) * 0.5, s.get(131, 100.0) * 0.5


def work_limits(s: Mapping[int, float]) -> tuple[float, float, float, float]:
    """(x0, x1, y0, y1) accepted by the firmware (grbl.c work_limits)."""
    w, h = s.get(130, 100.0), s.get(131, 100.0)
    if s.get(144, 0.0):
        return -w / 2, w / 2, -h / 2, h / 2
    return 0.0, w, 0.0, h


# ---------------------------------------------------------- axis mapping ---

def channel_for_axis(axis: str, swap: float) -> int:
    """DAC channel that drives physical (== commanded, once oriented) axis 'x' or 'y'."""
    if axis == "x":
        return 1 if swap else 0
    if axis == "y":
        return 0 if swap else 1
    raise ValueError(axis)


def scale_key_for_axis(axis: str, swap: float) -> int:
    return 100 + channel_for_axis(axis, swap)


def offset_key_for_axis(axis: str, swap: float) -> int:
    return 140 + channel_for_axis(axis, swap)


def _sgn(v: float) -> int:
    return -1 if v < 0 else 1


# ------------------------------------------------------------ orientation --

def channel_response(swap: float, inv: int, seen_x: Direction, seen_y: Direction,
                     scale0: float = 1.0, scale1: float = 1.0) -> list[Direction]:
    """Physical direction the beam moves for a POSITIVE voltage step on DAC ch0 / ch1.

    The wizard observes where commanded +X and +Y go under the current settings.
    Commanded +X reaches channel cx with factor sx * sign(scale), so
        seen_x = response[cx] * sx * sign(scale_cx)
    and the same for Y.  Inverting that gives the channel's own response."""
    if seen_x[0] == seen_y[0]:
        raise CalibError("both arrows point along the same axis - that cannot happen with a galvo "
                         "pair; check what you saw (and that both channels are driven)")
    sx = -1 if int(inv) & 1 else 1
    sy = -1 if int(inv) & 2 else 1
    cx = 1 if swap else 0
    cy = 1 - cx
    scales = (scale0, scale1)
    resp: list[Optional[Direction]] = [None, None]
    resp[cx] = (seen_x[0], seen_x[1] * sx * _sgn(scales[cx]))
    resp[cy] = (seen_y[0], seen_y[1] * sy * _sgn(scales[cy]))
    return resp  # type: ignore[return-value]


def solve_orientation(swap: float, inv: int, seen_x: Direction, seen_y: Direction,
                      scale0: float = 1.0, scale1: float = 1.0) -> tuple[int, int]:
    """New ($143, $3) so that commanded +X -> right and +Y -> away.

    Uses the firmware order invert-then-swap: commanded X (inverted by bit0) feeds
    channel 0, or channel 1 when swapped; commanded Y (bit1) feeds the other one."""
    resp = channel_response(swap, inv, seen_x, seen_y, scale0, scale1)
    ch_x = 0 if resp[0][0] == "x" else 1          # channel that physically moves X
    ch_y = 1 - ch_x
    scales = (scale0, scale1)
    new_swap = 1 if ch_x == 1 else 0
    sx_new = resp[ch_x][1] * _sgn(scales[ch_x])   # makes sx_new*sign(scale)*resp == +1
    sy_new = resp[ch_y][1] * _sgn(scales[ch_y])
    new_inv = (1 if sx_new < 0 else 0) | (2 if sy_new < 0 else 0)
    return new_swap, new_inv


# ------------------------------------------------------------------ scale --

def scale_update(old: float, nominal: float, measured: float) -> float:
    """new = old * nominal / measured  (the beam travelled `measured` for `nominal` mm)."""
    if not (nominal > 0 and measured > 0):
        raise CalibError("nominal and measured lengths must be positive")
    return old * nominal / measured


def rel_error(measured: float, nominal: float) -> float:
    return measured / nominal - 1.0


# ------------------------------------------------------------- distortion --

@dataclass
class DistortionFit:
    R: float
    e_mx: float      # cross-arm error along X (r ~ R)
    e_my: float
    e_cx: float      # square top/bottom side error (r ~ R sqrt2)
    e_cy: float      # square left/right side error
    b: float         # e(r) = a + b r^2
    a_x: float
    a_y: float

    @property
    def max_err(self) -> float:
        return max(abs(self.e_mx), abs(self.e_my), abs(self.e_cx), abs(self.e_cy))


def distortion_fit(R: float, Lcx: float, Lcy: float, Lsx: float, Lsy: float) -> DistortionFit:
    """Fit e(r) = a + b r^2 from a cross (arms +-R) and a square (side 2R).

    The cross arm ends sample r = R; the square sides are spans between corners at
    r = R sqrt2, so r^2 differs by exactly R^2 between the two."""
    if R <= 0 or min(Lcx, Lcy, Lsx, Lsy) <= 0:
        raise CalibError("R and all measured lengths must be positive")
    e_mx, e_my = Lcx / (2 * R) - 1.0, Lcy / (2 * R) - 1.0
    e_cx, e_cy = Lsx / (2 * R) - 1.0, Lsy / (2 * R) - 1.0
    b = 0.5 * ((e_cx - e_mx) + (e_cy - e_my)) / (R * R)
    return DistortionFit(R, e_mx, e_my, e_cx, e_cy, b, e_mx - b * R * R, e_my - b * R * R)


@dataclass
class DistortionUpdate:
    k1: float
    scale_x: float            # scale of the channel driving physical X
    scale_y: float
    k1_clamped: bool = False
    warnings: list[str] = field(default_factory=list)


def distortion_update(k1: float, scale_x: float, scale_y: float, fit: DistortionFit,
                      damping: float = 0.8) -> DistortionUpdate:
    """Firmware applies (1 + k1 r^2); the residual is ~ (1 + k1 r^2)(1 + b r^2), so k1 -= b.
    The centre-scale errors a_x / a_y are removed from the scales."""
    k1_raw = k1 - damping * fit.b
    k1_new = clamp(k1_raw, -1.0, 1.0)
    upd = DistortionUpdate(k1_new, scale_x / (1.0 + damping * fit.a_x), scale_y / (1.0 + damping * fit.a_y),
                           k1_clamped=k1_raw != k1_new)
    if upd.k1_clamped:
        upd.warnings.append(f"k1 clamped to the firmware range [-1, 1] (wanted {k1_raw:.6g})")
    if abs(k1_new * fit.R ** 2) > 0.1:
        upd.warnings.append(f"|k1 R^2| = {abs(k1_new * fit.R ** 2):.3f} > 0.1 - that is a very large correction; "
                            "recheck the measurements and the scale step")
    return upd


def bow_consistent(b: float, R: float, answer: str) -> Optional[bool]:
    """Sanity check of the sign of b against what the user sees (None = too small to judge).

    Scale error growing with radius (b > 0): corners are pushed out more than the edge
    midpoints, so the square's edges bow INWARD (pincushion).  b < 0: edges bow OUTWARD
    (barrel).  answer: "inward", "outward" or "straight"."""
    if abs(b) * R * R < 0.001:
        return None
    if answer == "straight":
        return False
    return (answer == "inward") == (b > 0)


# ----------------------------------------------------------------- offset --

def offset_update(s: Mapping[int, float], dx: float, dy: float) -> dict[int, float]:
    """New $140/$141 from the measured landing error (dx right+, dy away+, mm).

    The beam must move physically by (-dx, -dy).  Physical X is driven by channel cx
    (1 if swapped) through commanded X, whose field factor is sx*scale_cx (invert bit0),
    so a voltage change dV on that channel moves the beam by dV / (sx * scale_cx) mm:
        dV_cx = -dx * sx * scale_cx            (signed, so negative scales work)
    and for Y with sy (bit1) and channel cy.  Offsets are added after the radial term,
    so k1 does not enter.  Requires an oriented machine (commanded +X == physical right)."""
    swap = s.get(143, 0.0)
    inv = int(s.get(3, 0))
    out: dict[int, float] = {}
    for axis, d, bit in (("x", dx, 1), ("y", dy, 2)):
        ch = channel_for_axis(axis, swap)
        sgn = -1.0 if inv & bit else 1.0
        scale = s.get(100 + ch, 0.1)
        key = 140 + ch
        dv = -d * sgn * scale
        out[key] = clamp(s.get(key, 0.0) + dv, -10.0, 10.0)
    return out


# ------------------------------------------------------------ galvo range ----

RANGE_LEVELS: tuple = (0.2, 0.4, 0.6, 0.7, 0.8, 0.9, 1.0)
RANGE_FLOOR = 0.05          # a failed axis that never passed is drawn at this level (never a zero-width square)
FULL_SCALE_V = 10.0


def volts_to_mm(s: Mapping[int, float], v0: float, v1: float) -> tuple[float, float]:
    """Inverse of galvo_transform() for k1 = 0, without the +-10 V clamp: DAC volts on channel 0 / 1 ->
    machine mm.  mm = (V - offset) / scale, then undo swap, invert and origin (the forward order is
    origin, invert, swap, scale, offset)."""
    s0, s1 = s.get(100, 0.1), s.get(101, 0.1)
    if s0 == 0 or s1 == 0:
        raise CalibError("$100 / $101 must not be zero to convert volts to mm")
    xp = (v0 - s.get(140, 0.0)) / s0
    yp = (v1 - s.get(141, 0.0)) / s1
    x, y = (yp, xp) if s.get(143, 0.0) else (xp, yp)           # undo the swap
    inv = int(s.get(3, 0))
    w, h = s.get(130, 100.0), s.get(131, 100.0)
    ox, oy = (0.0, 0.0) if s.get(144, 0.0) else (w * 0.5, h * 0.5)
    return x * (-1.0 if inv & 1 else 1.0) + ox, y * (-1.0 if inv & 2 else 1.0) + oy


def axis_volts_to_mm(s: Mapping[int, float], axis: str, v: float) -> float:
    """Commanded mm on `axis` ('x' / 'y') that puts voltage v on the DAC channel driving that axis ($143)."""
    ch = channel_for_axis(axis, s.get(143, 0.0))
    X, Y = volts_to_mm(s, v, 0.0) if ch == 0 else volts_to_mm(s, 0.0, v)
    return X if axis == "x" else Y


def parse_levels(text: str) -> tuple:
    """"0.2,0.5,1" -> (0.2, 0.5, 1.0): fractions of full scale in (0, 1], sorted, unique."""
    toks = [t for t in re.split(r"[,;\s]+", text.strip()) if t]
    try:
        vals = [float(t) for t in toks]
    except ValueError:
        vals = []
    if not vals or any(not (0.0 < v <= 1.0) for v in vals):
        raise CalibError("range levels must be fractions of full scale in (0, 1], e.g. 0.2,0.5,0.8,1.0")
    return tuple(sorted(set(round(v, 6) for v in vals)))


@dataclass
class RangeSquare:
    """A (rectangular) test pattern whose extreme corners sit at +-fx*10 V on the channel of axis X and
    +-fy*10 V on the channel of axis Y.  corner_volts are the volts the transform really produces
    (axis-x channel, axis-y channel) after the work-area clamp, so `limited` means the requested level
    could not be reached with the current $130/$131."""
    fx: float
    fy: float
    target: tuple[float, float]
    corners_mm: list
    corner_volts: list
    centre_mm: tuple[float, float]
    tick_mm: tuple
    limited: bool = False

    @property
    def actual(self) -> tuple[float, float]:
        return (max(abs(v[0]) for v in self.corner_volts), max(abs(v[1]) for v in self.corner_volts))


def range_square(s: Mapping[int, float], fx: float, fy: float, tick_frac: float = 0.85) -> RangeSquare:
    """Corners (-,-), (+,-), (+,+), (-,+) of the rectangle at +-fx*10 V (axis X) and +-fy*10 V (axis Y),
    centred on the electrical centre (V = 0 on both channels, i.e. the offset is part of the position).
    The mm come from volts_to_mm() and are clamped to the work area, as grbl.c does; the volts that result
    are recomputed with the forward transform with k1 = 0 (the wizard sets $142 = 0 for this test).
    `tick_mm` is a short mark from the middle of the +X side towards the centre: it tells the operator
    which pair of sides is X, whatever the (not yet calibrated) orientation is."""
    swap = s.get(143, 0.0)
    cx, cy = channel_for_axis("x", swap), channel_for_axis("y", swap)
    vx, vy = round(fx * FULL_SCALE_V, 9), round(fy * FULL_SCALE_V, 9)
    x0, x1, y0, y1 = work_limits(s)

    def X(v: float) -> float:
        return clamp(axis_volts_to_mm(s, "x", v), x0, x1)

    def Y(v: float) -> float:
        return clamp(axis_volts_to_mm(s, "y", v), y0, y1)

    corners = [(X(-vx), Y(-vy)), (X(vx), Y(-vy)), (X(vx), Y(vy)), (X(-vx), Y(vy))]
    forward = dict(s)
    forward[142] = 0.0
    volts = []
    for mx, my in corners:
        v = galvo_transform(forward, mx, my, clamp_volts=False)
        volts.append((v[cx], v[cy]))
    sq = RangeSquare(fx, fy, (vx, vy), corners, volts, (X(0.0), Y(0.0)),
                     ((X(vx), Y(0.0)), (X(tick_frac * vx), Y(0.0))))
    ax, ay = sq.actual
    sq.limited = ax < vx - 0.01 or ay < vy - 0.01
    return sq


class RangeLadder:
    """Pure logic of the range search.  Each round draws a rectangle with the level of the ladder for
    every axis that is still passing and the last good level for an axis that already failed.  Answers:
    'ok' (all good), 'x' / 'y' (that axis' sides are wrong), 'both'.  A failed axis stays at its last
    good level; the other one carries on up the ladder until it fails too or the ladder ends."""

    def __init__(self, levels: Sequence[float] = RANGE_LEVELS, floor: float = RANGE_FLOOR):
        self.levels = tuple(sorted(set(levels)))
        self.floor = floor
        self.i = 0
        self.good = {"x": 0.0, "y": 0.0}
        self.active = {"x": True, "y": True}
        self.history: list[dict] = []

    @property
    def done(self) -> bool:
        return self.i >= len(self.levels) or not any(self.active.values())

    def next_test(self) -> tuple[float, float]:
        f = self.levels[self.i]
        return tuple(f if self.active[a] else max(self.good[a], self.floor) for a in ("x", "y"))  # type: ignore

    def answer(self, ans: str) -> None:
        if ans not in ("ok", "x", "y", "both"):
            raise CalibError(f"unknown range answer {ans!r}")
        fx, fy = self.next_test()
        failing = {"x": ans in ("x", "both"), "y": ans in ("y", "both")}
        for a, f in (("x", fx), ("y", fy)):
            if self.active[a]:
                if failing[a]:
                    self.active[a] = False
                else:
                    self.good[a] = f
        self.history.append({"level": self.levels[self.i], "fx": fx, "fy": fy, "answer": ans})
        self.i += 1


def suggest_work_area(vx: float, vy: float, scale_x: float, scale_y: float, margin_v: float = 0.5,
                      offset_x: float = 0.0, offset_y: float = 0.0) -> tuple[float, float]:
    """Work area (W, H) in mm for usable half-ranges vx / vy (volts on the channels driving physical X / Y):
    W = 2 (vx - margin - |offset_x|) / |scale_x|.  The offset takes part because the mirror limit is on the
    absolute DAC voltage while the field is centred on the offset.  Rounded DOWN to 0.1 mm and kept inside
    the firmware range of $130 / $131."""
    if scale_x == 0 or scale_y == 0:
        raise CalibError("the scale must not be zero")

    def one(v: float, sc: float, off: float) -> float:
        half_v = v - margin_v - abs(off)
        if half_v <= 0:
            raise CalibError(f"usable range {v:g} V is not larger than margin + offset ({margin_v:g} + {abs(off):g} V)")
        return clamp(math.floor(2.0 * half_v / abs(sc) * 10.0 + 1e-6) / 10.0, RANGES[130][0], RANGES[130][1])

    return one(vx, scale_x, offset_x), one(vy, scale_y, offset_y)


# ----------------------------------------------------------------- delays ---

@dataclass
class DelayTuner:
    """Bisection-ish tuner for one delay: the user says "increase", "decrease" or "clean".

    Starts with `step`.  After three answers in the same direction the step doubles each time
    (up to `max_step`) so a far-off start is not walked in tiny steps; each time the direction
    flips the step is halved (never below `min_step`, the tick resolution) and growth stops."""
    name: str
    value: float
    step: float = 20.0
    min_step: float = 10.0
    lo: float = 0.0
    hi: float = 1e5
    max_step: float = 0.0            # growth cap while the answer keeps the same direction (0 = 3 x step)
    last_dir: int = 0
    same_run: int = 0
    flips: int = 0
    done: bool = False
    at_limit: bool = False
    history: list = field(default_factory=list)

    def update(self, direction: int) -> float:
        """direction: +1 increase, -1 decrease, 0 looks clean.  Returns the next value."""
        self.history.append((self.value, direction))
        if direction == 0:
            self.done = True
            return self.value
        if not self.max_step:
            self.max_step = 3.0 * self.step
        if self.last_dir and direction != self.last_dir:
            self.flips += 1
            self.same_run = 1
            self.step = max(self.min_step, self.step / 2.0)
        else:
            self.same_run += 1
            if self.flips == 0 and self.same_run >= 3:
                self.step = min(self.max_step, self.step * 2.0)
        self.last_dir = direction
        new = round(clamp(self.value + direction * self.step, self.lo, self.hi), 4)
        if new == self.value:
            self.done = self.at_limit = True
        self.value = new
        if self.flips >= 3 and self.step <= self.min_step:
            self.done = True
        return self.value


class RegionAllocator:
    """Hands out non-overlapping boxes on a test card, centre outwards: the first
    box on a fresh card is centred exactly on the card centre (the lens centre),
    every later one takes the free position whose centre is nearest to it.
    `limits` = (x0, y0, x1, y1) of the usable card in machine mm; boxes keep
    `gap` mm from each other."""

    STEP = 0.5          # candidate grid, mm (aligned to the centred position)

    def __init__(self, limits: tuple[float, float, float, float], gap: float = 2.0):
        self.x0, self.y0, self.x1, self.y1 = limits
        self.gap = gap
        self.reset()

    def reset(self) -> None:
        self.used: list[tuple[float, float, float, float]] = []

    def _free(self, x: float, y: float, w: float, h: float) -> bool:
        g = self.gap - 1e-9
        for ux, uy, uw, uh in self.used:
            if x < ux + uw + g and ux < x + w + g and y < uy + uh + g and uy < y + h + g:
                return False
        return True

    def _axis(self, lo: float, hi: float, size: float) -> list[float]:
        """Candidate starts along one axis: the centred one, then +-STEP outwards."""
        c0 = (lo + hi) / 2 - size / 2
        out = [c0]
        k = 1
        while True:
            added = False
            for v in (c0 - k * self.STEP, c0 + k * self.STEP):
                if lo - 1e-9 <= v and v + size <= hi + 1e-9:
                    out.append(v)
                    added = True
            if not added:
                return out
            k += 1

    def allocate(self, w: float, h: float) -> Optional[tuple[float, float, float, float]]:
        """(x, y, w, h) of the free region nearest the card centre, or None when full."""
        if w > self.x1 - self.x0 + 1e-9 or h > self.y1 - self.y0 + 1e-9:
            return None
        cx, cy = (self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2
        best = None
        for y in self._axis(self.y0, self.y1, h):
            for x in self._axis(self.x0, self.x1, w):
                d = (x + w / 2 - cx) ** 2 + (y + h / 2 - cy) ** 2
                if best is not None and d >= best[0]:
                    continue
                if self._free(x, y, w, h):
                    best = (d, x, y)
        if best is None:
            return None
        r = (best[1], best[2], w, h)
        self.used.append(r)
        return r


# ------------------------------------------------------------ power / speed -

def ladder_values(cap: float, n: int = 9) -> list[int]:
    """n distinct S values rising to `cap` (the last patch is the cap)."""
    vals: list[int] = []
    for k in range(1, n + 1):
        v = max(1, int(round(cap * k / n)))
        if v not in vals:
            vals.append(v)
    return vals


def prr_values(lo_khz: float, hi_khz: float, n: int) -> list[int]:
    """n PRR values in Hz, linearly spaced from lo_khz to hi_khz (both included), distinct and rising."""
    if not (0 < lo_khz < hi_khz):
        raise CalibError(f"PRR range must satisfy 0 < min < max (got {lo_khz:g} .. {hi_khz:g} kHz)")
    if n < 2:
        raise CalibError("at least 2 PRR steps are needed")
    vals: list[int] = []
    for k in range(n):
        hz = int(round((lo_khz + (hi_khz - lo_khz) * k / (n - 1)) * 1000.0))
        if hz not in vals:
            vals.append(hz)
    return vals


def power_percent(S: float, smax: float, pmin: float, pmax: float) -> float:
    """% power the firmware requests for S (gcode_power_byte): pmin + (pmax-pmin) * S/smax."""
    f = 1.0 if smax <= 0 else min(max(S / smax, 0.0), 1.0)
    return pmin + (pmax - pmin) * f


def power_map(s_mark: float, s_sat: Optional[float], smax: float, old_pmin: float, old_pmax: float) -> dict[int, float]:
    """New $224 / $225 so that S=0+ is just at the marking threshold and S=$30 at saturation.

    s_mark: lowest S that visibly marks; s_sat: S where the result stops getting stronger
    (None = not reached within the tested range - then only $224 is returned)."""
    if s_mark <= 0:
        raise CalibError("lowest marking S must be positive")
    p_thr = power_percent(s_mark, smax, old_pmin, old_pmax)
    out = {224: round(clamp(p_thr, 0.0, 100.0), 1)}
    if s_sat is not None:
        if s_sat <= s_mark:
            raise CalibError("the saturation S must be larger than the lowest marking S")
        p_sat = power_percent(s_sat, smax, old_pmin, old_pmax)
        out[225] = round(clamp(p_sat, 0.0, 100.0), 1)
        if out[225] - out[224] < 1.0:
            raise CalibError("marking threshold and saturation are less than 1% apart - cannot map that")
    return out


def speed_ladder_values(vmax_mm_min: float, mm_s: Sequence[float] = (200, 400, 700, 1000, 1500, 2200)) -> list[float]:
    """Feeds (mm/min) for the speed ladder, never above the firmware's $110 clamp."""
    return [round(v * 60.0, 1) for v in mm_s if v * 60.0 <= vmax_mm_min + 1e-9]


def recommend_speed(feeds: Sequence[float], first_bad: int) -> Optional[float]:
    """first_bad: 1-based patch where quality degrades, 0 = none degraded.
    Returns the feed to use as the maximum marking speed (None: even the slowest is bad)."""
    if first_bad <= 0:
        return feeds[-1]
    if first_bad == 1:
        return None
    return feeds[first_bad - 2]
