"""
WaveMaster live interactive keyboard calibration mode.

Continuous guide-laser tracing with real-time keyboard control for shape-based setup steps:
    - bounds: offset ($140/$141) and range ($130/$131) in one, a dot driven to the four corners
    - range: galvo deflection limits ($130/$131)
    - field: lens work area ($130/$131) & origin mode ($144)
    - orientation: axis swap ($143) & invert ($3)
    - scale: square size & scale calibration ($100/$101)
    - distortion: F-theta k1 curvature ($142) & scales
    - offset: crosshair centering ($140/$141)

Controls:
    Arrows                  Move crosshair / shape (offset, nudging)
    [ / ] or - / +          Resize shape / grow / shrink
    < / >                   Trace speed (multiply / divide guide-feed by 1.25)
    Tab or m                Cycle step size (fine / normal / coarse)
    1, 2, 3                 Select step size (1=fine, 2=normal, 3=coarse)
    Shift / Ctrl + Arrow    Coarse movement modifier
    a                       Cycle active axis (both / x / y)
    x / y / b               Select X axis only / Y axis only / Both axes
    r                       Reset shape to initial parameters
    p / Backspace           Bounds step: back to the previous corner
    Enter                   Accept current shape and apply computed settings
                            (bounds step: record this corner, the last Enter applies)
    s                       Skip step
    q                       Quit wizard
"""
from __future__ import annotations

import curses
import math
import select
import sys
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Optional, Sequence

import calibrate_math as cm
import gcode_gen as gg
from calibrate_math import CalibError, clamp
from gcode_gen import Box, MoveTo, SetPower, SetSpeed

if TYPE_CHECKING:
    import calibrate as cal

# ------------------------------------------------------------------ constants --

DEFAULT_GUIDE_FEED = 24000.0       # mm/min (400 mm/s) - steady persistence of vision without flicker
MIN_GUIDE_FEED = 600.0             # mm/min (10 mm/s)
MAX_FIRMWARE_FEED = 300000.0       # mm/min ($110 default ceiling: 5000 mm/s)

STEP_SIZES = ("fine", "normal", "coarse")


def normalize_key(k: Any) -> Optional[str]:
    """Normalize raw curses keycodes, escape sequences, or string characters to standard key names."""
    if k is None:
        return None
    if isinstance(k, str):
        k_low = k.lower()
        if k_low in ("\n", "\r", "enter"):
            return "enter"
        if k_low in ("\t", "tab"):
            return "tab"
        if k_low in ("\x7f", "backspace"):
            return "backspace"
        if k in ("\x1b[A", "\x1bOA"):
            return "up"
        if k in ("\x1b[B", "\x1bOB"):
            return "down"
        if k in ("\x1b[C", "\x1bOC"):
            return "right"
        if k in ("\x1b[D", "\x1bOD"):
            return "left"
        return k_low

    # Curses keycodes
    if k in (curses.KEY_UP,):
        return "up"
    if k in (curses.KEY_DOWN,):
        return "down"
    if k in (curses.KEY_LEFT,):
        return "left"
    if k in (curses.KEY_RIGHT,):
        return "right"
    if k in (curses.KEY_SR, curses.KEY_SF):  # Shift-Up / Shift-Down
        return "shift_up" if k == curses.KEY_SR else "shift_down"
    if k in (curses.KEY_SLEFT, curses.KEY_SRIGHT):
        return "shift_left" if k == curses.KEY_SLEFT else "shift_right"
    if k in (ord("\n"), ord("\r"), curses.KEY_ENTER):
        return "enter"
    if k in (ord("\t"),):
        return "tab"
    if k in (127, curses.KEY_BACKSPACE):
        return "backspace"
    if 0 <= k <= 255:
        return chr(k).lower()
    return None


# ------------------------------------------------------------- key providers --

class LiveInput:
    """Base interface for polling keys."""
    def get_key(self) -> Optional[str]:
        raise NotImplementedError


class ScriptedKeyInput(LiveInput):
    """Feeds a canned list of keys for testing and automation."""
    def __init__(self, keys: Sequence[str | int]):
        self.queue: list[str] = [normalize_key(k) for k in keys if normalize_key(k) is not None]  # type: ignore

    def get_key(self) -> Optional[str]:
        if self.queue:
            return self.queue.pop(0)
        return None

    def push(self, *keys: str | int) -> None:
        for k in keys:
            norm = normalize_key(k)
            if norm:
                self.queue.append(norm)

    @property
    def empty(self) -> bool:
        return len(self.queue) == 0


class CursesKeyInput(LiveInput):
    """Non-blocking key reader from a curses window."""
    def __init__(self, win: Any):
        self.win = win
        self.win.nodelay(True)
        self.win.keypad(True)

    def get_key(self) -> Optional[str]:
        try:
            ch = self.win.getch()
            if ch == -1:
                return None
            return normalize_key(ch)
        except Exception:
            return None


class HeadlessKeyInput(LiveInput):
    """Fallback non-blocking reader when running without curses/script."""
    def __init__(self, scripted_keys: Optional[Sequence[str | int]] = None) -> None:
        self.buf: list[str] = [normalize_key(k) for k in (scripted_keys or []) if normalize_key(k) is not None]  # type: ignore

    def get_key(self) -> Optional[str]:
        if self.buf:
            return self.buf.pop(0)
        try:
            if not sys.stdin.isatty():
                r, _, _ = select.select([sys.stdin], [], [], 0.0)
                if r:
                    line = sys.stdin.readline()
                    if not line:
                        return "enter"
                    tokens = line.strip().split()
                    for tok in tokens:
                        k = normalize_key(tok)
                        if k:
                            self.buf.append(k)
                    if self.buf:
                        return self.buf.pop(0)
        except Exception:
            pass
        return None


# ------------------------------------------------------------- display UI --

class LiveDisplay:
    def update(self, step: "LiveStepBase") -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


class HeadlessDisplay(LiveDisplay):
    """Outputs status lines to standard output or buffers them for tests."""
    def __init__(self, quiet: bool = False):
        self.quiet = quiet
        self.last_status: list[str] = []

    def update(self, step: "LiveStepBase") -> None:
        lines = step.status_lines()
        self.last_status = lines
        if not self.quiet:
            print("\n".join(lines), flush=True)


class CursesDisplay(LiveDisplay):
    """Draws a clean terminal UI using curses."""
    def __init__(self, win: Any):
        self.win = win
        try:
            curses.curs_set(0)
        except Exception:
            pass

    def update(self, step: "LiveStepBase") -> None:
        try:
            self.win.erase()
            max_y, max_x = self.win.getmaxyx()
            lines = step.status_lines()
            for row, text in enumerate(lines[:max_y - 1]):
                self.win.addstr(row, 0, text[:max_x - 1])
            self.win.refresh()
        except Exception:
            pass

    def close(self) -> None:
        try:
            curses.curs_set(1)
        except Exception:
            pass


# ------------------------------------------------------------- step bases --

class LiveStepBase:
    def __init__(self, wizard: "cal.Wizard", initial_feed: Optional[float] = None):
        self.wiz = wizard
        max_f = min(MAX_FIRMWARE_FEED, wizard.get(110))
        self.feed: float = min(max_f, max(MIN_GUIDE_FEED, initial_feed or wizard.a.guide_feed))
        self.step_mode: str = "normal"  # "fine", "normal", "coarse"
        self.axis_mode: str = "both"    # "both", "x", "y"

    @property
    def feed_ceiling(self) -> float:
        return min(MAX_FIRMWARE_FEED, self.wiz.get(110))

    def adjust_feed(self, factor: float) -> None:
        new_feed = self.feed * factor
        self.feed = clamp(new_feed, MIN_GUIDE_FEED, self.feed_ceiling)

    def cycle_step_size(self) -> None:
        idx = STEP_SIZES.index(self.step_mode)
        self.step_mode = STEP_SIZES[(idx + 1) % len(STEP_SIZES)]

    def cycle_axis(self) -> None:
        order = ("both", "x", "y")
        idx = order.index(self.axis_mode)
        self.axis_mode = order[(idx + 1) % len(order)]

    def title(self) -> str:
        raise NotImplementedError

    def make_pattern(self, feed: float) -> "cal.Pattern":
        raise NotImplementedError

    def handle_key(self, key: str) -> bool:
        """Handle a step-specific key. Return True if state was modified."""
        raise NotImplementedError

    def handle_enter(self) -> bool:
        """Enter was pressed. True when the step is complete and must be accepted; a step that collects several
        readings records this one and returns False."""
        return True

    def needs_stream(self) -> bool:
        """False when nothing changed since the last lap that was streamed (a static dot needs no repeats)."""
        return True

    def status_lines(self) -> list[str]:
        raise NotImplementedError

    def accept(self) -> dict[str, Any]:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError

    def cleanup(self) -> None:
        pass


# ------------------------------------------------------------- step: range --

class LiveRangeStep(LiveStepBase):
    """Live interactive tuning of galvo deflection range (DAC volts per axis)."""
    def __init__(self, wizard: "cal.Wizard", initial_feed: Optional[float] = None):
        super().__init__(wizard, initial_feed)
        self.swap = self.wiz.get(143)
        self.cx = cm.channel_for_axis("x", self.swap)
        self.cy = cm.channel_for_axis("y", self.swap)

        # Temporary settings
        self.orig_settings = {n: self.wiz.get(n) for n in (130, 131, 142)}
        self.temp_settings = self.wiz.range_temp_values()
        self.touched: set[int] = set()

        rows = [(n, self.wiz.get(n), self.temp_settings[n]) for n in (130, 131)
                if abs(self.wiz.get(n) - self.temp_settings[n]) > 1e-9]
        if self.orig_settings[142] != 0.0:
            rows.append((142, self.orig_settings[142], 0.0))
        self.touched.update(n for n, _, _ in rows)
        if rows:
            self.wiz.write(rows, "temporary values for range test", quiet=True, record=False)

        # State: fractions of full scale (10V)
        self.init_fx = 0.7
        self.init_fy = 0.7
        self.fx = self.init_fx
        self.fy = self.init_fy

    def cleanup(self) -> None:
        if self.touched:
            self.wiz.range_restore(self.orig_settings, self.touched, None)
            self.touched.clear()

    def title(self) -> str:
        return "RANGE: Galvo Travel Deflection Limits"

    def step_delta(self) -> float:
        if self.step_mode == "fine":
            return 0.01
        if self.step_mode == "coarse":
            return 0.10
        return 0.05

    def handle_key(self, key: str) -> bool:
        d = self.step_delta()
        if key in ("shift_up", "shift_right", "ctrl_up", "ctrl_right"):
            d = 0.10
        elif key in ("shift_down", "shift_left", "ctrl_down", "ctrl_left"):
            d = 0.10

        if key in ("+", "=", "]", "up", "right", "shift_up", "shift_right", "ctrl_up", "ctrl_right"):
            if self.axis_mode in ("both", "x"):
                self.fx = clamp(self.fx + d, 0.05, 1.0)
            if self.axis_mode in ("both", "y"):
                self.fy = clamp(self.fy + d, 0.05, 1.0)
            return True
        elif key in ("-", "_", "[", "down", "left", "shift_down", "shift_left", "ctrl_down", "ctrl_left"):
            if self.axis_mode in ("both", "x"):
                self.fx = clamp(self.fx - d, 0.05, 1.0)
            if self.axis_mode in ("both", "y"):
                self.fy = clamp(self.fy - d, 0.05, 1.0)
            return True
        elif key == "a":
            self.cycle_axis()
            return True
        elif key == "x":
            self.axis_mode = "x"
            return True
        elif key == "y":
            self.axis_mode = "y"
            return True
        elif key == "b":
            self.axis_mode = "both"
            return True
        return False

    def reset(self) -> None:
        self.fx = self.init_fx
        self.fy = self.init_fy
        self.axis_mode = "both"

    def current_volts(self) -> tuple[float, float]:
        vx = round(self.fx * cm.FULL_SCALE_V, 4)
        vy = round(self.fy * cm.FULL_SCALE_V, 4)
        return vx, vy

    def make_pattern(self, feed: float) -> "cal.Pattern":
        import calibrate as cal
        sq = cm.range_square(self.wiz.view(), self.fx, self.fy)
        return cal.pat_range(sq, feed)

    def suggest_workarea(self) -> Optional[tuple[float, float]]:
        vx, vy = self.current_volts()
        kx = cm.scale_key_for_axis("x", self.swap)
        ky = cm.scale_key_for_axis("y", self.swap)
        try:
            return cm.suggest_work_area(vx, vy, self.wiz.get(kx), self.wiz.get(ky),
                                        self.wiz.a.range_margin_v, self.wiz.get(kx + 40), self.wiz.get(ky + 40))
        except CalibError:
            return None

    def status_lines(self) -> list[str]:
        vx, vy = self.current_volts()
        sug = self.suggest_workarea()
        sug_str = f"$130 = {sug[0]:g} mm, $131 = {sug[1]:g} mm" if sug else "(cannot compute)"
        sq = cm.range_square(self.wiz.view(), self.fx, self.fy)
        w_mm = sq.corners_mm[1][0] - sq.corners_mm[0][0]
        h_mm = sq.corners_mm[2][1] - sq.corners_mm[1][1]

        lines = [
            f"=== {self.title()} ===",
            f"Active axis: [{self.axis_mode.upper()}] (press 'a' to cycle: both/x/y, or 'x'/'y'/'b')",
            f"Deflection fraction: X={self.fx:.2f} ({self.fx*100:.0f}%), Y={self.fy:.2f} ({self.fy*100:.0f}%)",
            f"Usable Range Volts:  X = +-{vx:.2f} V (Ch {self.cx}), Y = +-{vy:.2f} V (Ch {self.cy})",
            f"Shape Rectangle:     {abs(w_mm):.1f} x {abs(h_mm):.1f} mm on bed",
            f"Suggested Work Area: {sug_str} (margin {self.wiz.a.range_margin_v:g} V)",
            f"Step size: {self.step_mode} (Tab/m: fine 0.01 / normal 0.05 / coarse 0.10)   Speed: F{self.feed:.0f} mm/min (< / >)",
            "--------------------------------------------------------------------------------",
            "Controls: [ / ] or - / + resize | Arrows: adjust | a/x/y/b: axis | < / >: feed",
            "          Tab: step size | r: reset | Enter: accept | s: skip | q: quit",
        ]
        return lines

    def accept(self) -> dict[str, Any]:
        self.cleanup()
        vx, vy = self.current_volts()
        res: dict[str, Any] = {
            "f_x": self.fx,
            "f_y": self.fy,
            "range_volts_x": vx,
            "range_volts_y": vy,
            "swap": self.swap,
            "channel_x": self.cx,
            "channel_y": self.cy,
            "restored": True,
        }
        self.wiz.range_ch = [0.0, 0.0]
        self.wiz.range_ch[self.cx] = vx
        self.wiz.range_ch[self.cy] = vy
        self.wiz.range_swap = self.swap
        self.wiz.range_source = "live run"
        self.wiz.save_state()

        sug = self.suggest_workarea()
        if sug:
            W, H = sug
            res["suggested"] = {"W": W, "H": H, "margin_v": self.wiz.a.range_margin_v}
            res["applied"] = bool(self.wiz.apply({130: W, 131: H}, "work area from galvo range", confirm=False))
            self.wiz.regions = None
        return res


# ------------------------------------------------------------ step: bounds --

class LiveBoundsStep(LiveRangeStep):
    """Offset and range in one: a guide-laser dot is driven to each corner until it starts to fade there.

    The centre of the four limits is the offset ($140/$141), half their span is the range ($130/$131). The dot
    is positioned in absolute DAC volts, so the result does not depend on the current scale or offset being
    right; the arrows move it the way you look at the machine, which needs the orientation to be calibrated."""
    START_V = 7.0
    LIMIT_V = 10.0
    STEP_V = {"fine": 0.01, "normal": 0.05, "coarse": 0.25}
    CORNER_SIGNS = {"top-left": (-1, 1), "top-right": (1, 1), "bottom-right": (1, -1), "bottom-left": (-1, -1)}
    MOVES = {"left": (-1, 0), "right": (1, 0), "up": (0, 1), "down": (0, -1)}
    SPREAD_WARN_V = 0.3

    def __init__(self, wizard: "cal.Wizard", initial_feed: Optional[float] = None):
        super().__init__(wizard, initial_feed)
        inv = int(self.wiz.get(3))
        self.sx = math.copysign(1.0, (-1.0 if inv & 1 else 1.0) * self.wiz.get(100 + self.cx))
        self.sy = math.copysign(1.0, (-1.0 if inv & 2 else 1.0) * self.wiz.get(100 + self.cy))
        self.centre0 = (self.wiz.get(140 + self.cx), self.wiz.get(140 + self.cy))
        self.recorded: dict[str, tuple[float, float]] = {}
        self.idx = 0
        self.note = ""
        self.streamed: Optional[tuple[float, float]] = None
        self.v = self.corner_start(0)

    def title(self) -> str:
        return "BOUNDS: Offset and Range from the Four Corners"

    def corner_start(self, idx: int) -> list[float]:
        qx, qy = self.CORNER_SIGNS[cm.BOUNDS_CORNERS[idx]]
        vx = next((v[0] for n, v in self.recorded.items() if self.CORNER_SIGNS[n][0] == qx), None)
        vy = next((v[1] for n, v in self.recorded.items() if self.CORNER_SIGNS[n][1] == qy), None)
        if vx is None:
            vx = self.centre0[0] + qx * self.sx * self.START_V
        if vy is None:
            vy = self.centre0[1] + qy * self.sy * self.START_V
        return [clamp(vx, -self.LIMIT_V, self.LIMIT_V), clamp(vy, -self.LIMIT_V, self.LIMIT_V)]

    def result(self) -> "cm.BoundsResult":
        return cm.bounds_from_corners(self.wiz.view(), self.recorded, self.wiz.a.range_margin_v)

    def back(self) -> bool:
        if self.idx == 0:
            return False
        self.idx -= 1
        self.v = list(self.recorded.pop(cm.BOUNDS_CORNERS[self.idx]))
        self.note = ""
        return True

    def handle_key(self, key: str) -> bool:
        if key in ("p", "backspace"):
            return self.back()
        base = key.split("_", 1)[1] if key.startswith(("shift_", "ctrl_")) else key
        if base not in self.MOVES or self.idx >= len(cm.BOUNDS_CORNERS):
            return False
        d = self.STEP_V["coarse"] if base != key else self.STEP_V[self.step_mode]
        dx, dy = self.MOVES[base]
        self.v[0] = clamp(self.v[0] + self.sx * dx * d, -self.LIMIT_V, self.LIMIT_V)
        self.v[1] = clamp(self.v[1] + self.sy * dy * d, -self.LIMIT_V, self.LIMIT_V)
        return True

    def handle_enter(self) -> bool:
        if self.idx >= len(cm.BOUNDS_CORNERS):
            return True
        self.recorded[cm.BOUNDS_CORNERS[self.idx]] = (self.v[0], self.v[1])
        self.idx += 1
        self.note = ""
        if self.idx < len(cm.BOUNDS_CORNERS):
            self.v = self.corner_start(self.idx)
            return False
        try:
            self.v = list(self.result().centre_v)
        except CalibError as e:
            self.note = str(e)
            self.back()
        return False

    def reset(self) -> None:
        if self.idx >= len(cm.BOUNDS_CORNERS):
            self.back()
        self.v = self.corner_start(self.idx)

    def dot_mm(self) -> tuple[float, float]:
        v0, v1 = (self.v[0], self.v[1]) if self.cx == 0 else (self.v[1], self.v[0])
        return cm.volts_to_mm(self.wiz.view(), v0, v1)

    def make_pattern(self, feed: float) -> "cal.Pattern":
        import calibrate as cal
        pos = self.dot_mm()
        self.streamed = (self.v[0], self.v[1])
        return cal._pat("bounds_dot", cal._head(feed, 0.0) + [MoveTo(*pos)], pos, kind="dot")

    def needs_stream(self) -> bool:
        return self.streamed != (self.v[0], self.v[1])

    def status_lines(self) -> list[str]:
        n = len(cm.BOUNDS_CORNERS)
        mm = self.dot_mm()
        lines = [f"=== {self.title()} ==="]
        if self.idx < n:
            lines.append(f"Corner {self.idx + 1}/{n}: {cm.BOUNDS_CORNERS[self.idx].upper()}  - drive the dot out "
                         "until it just starts to fade, then press Enter")
        else:
            lines.append("All four corners recorded: the dot is at the computed centre. Enter applies, "
                         "p goes back to redo the last corner")
        lines.append(f"Dot: X = {self.v[0]:+.2f} V (ch {self.cx}), Y = {self.v[1]:+.2f} V (ch {self.cy})   "
                     f"= {mm[0]:.1f}, {mm[1]:.1f} mm")
        for name in cm.BOUNDS_CORNERS:
            if name in self.recorded:
                x, y = self.recorded[name]
                lines.append(f"  {name:<13} X {x:+.2f} V  Y {y:+.2f} V")
        if len(self.recorded) == n:
            res = self.result()
            ox, oy = res.offsets[140 + self.cx], res.offsets[140 + self.cy]
            lines += [f"Offset:    $140/$141 = {res.offsets[140]:+.3f} V / {res.offsets[141]:+.3f} V "
                      f"(centre X {ox:+.3f} V, Y {oy:+.3f} V)",
                      f"Range:     half travel X {res.half_v[0]:.2f} V, Y {res.half_v[1]:.2f} V  "
                      f"-> $130 = {res.work_area[0]:g} mm, $131 = {res.work_area[1]:g} mm "
                      f"(margin {self.wiz.a.range_margin_v:g} V)"]
            if max(res.spread_v) > self.SPREAD_WARN_V:
                lines.append(f"WARNING: the two readings of an edge differ by up to {max(res.spread_v):.2f} V: "
                             "judge the fade at the same brightness, or redo with p")
        if self.note:
            lines.append(f"NOTE: {self.note}")
        lines += [f"Step: {self.step_mode} {self.STEP_V[self.step_mode]:g} V (Tab/m; 1/2/3 fine {self.STEP_V['fine']:g} "
                  f"/ normal {self.STEP_V['normal']:g} / coarse {self.STEP_V['coarse']:g}; Shift/Ctrl+arrow coarse)   "
                  f"Speed: F{self.feed:.0f} (< / >)",
                  "-" * 80,
                  "Controls: Arrows: move the dot | Enter: record corner / apply | p or Backspace: previous corner",
                  "          Tab: step size | r: restart this corner | s: skip | q: quit"]
        return lines

    def accept(self) -> dict[str, Any]:
        self.cleanup()
        res: dict[str, Any] = {"corners_v": {n: list(v) for n, v in self.recorded.items()}, "restored": True}
        if len(self.recorded) < len(cm.BOUNDS_CORNERS):
            res.update({"incomplete": True, "applied": False})
            return res
        out = self.result()
        res.update({"centre_v": list(out.centre_v), "half_v": list(out.half_v), "spread_v": list(out.spread_v),
                    "offsets": {str(n): v for n, v in out.offsets.items()},
                    "suggested": {"W": out.work_area[0], "H": out.work_area[1], "margin_v": self.wiz.a.range_margin_v}})
        changes = {**out.offsets, 130: out.work_area[0], 131: out.work_area[1]}
        res["applied"] = bool(self.wiz.apply(changes, "offset and range from the four corner readings", confirm=False))
        self.wiz.range_ch = [0.0, 0.0]
        self.wiz.range_ch[self.cx] = out.half_v[0] + abs(out.centre_v[0])
        self.wiz.range_ch[self.cy] = out.half_v[1] + abs(out.centre_v[1])
        self.wiz.range_swap = self.swap
        self.wiz.range_source = "bounds step"
        self.wiz.regions = None
        self.wiz.save_state()
        return res


# ------------------------------------------------------------ step: offset --

class LiveOffsetStep(LiveStepBase):
    """Live interactive tuning of centre offset $140 / $141."""
    def __init__(self, wizard: "cal.Wizard", initial_feed: Optional[float] = None):
        super().__init__(wizard, initial_feed)
        g = self.wiz.geometry()
        self.cx, self.cy = g["centre"]
        self.arm = min(6.0, 0.1 * min(g["W"], g["H"]))
        self.limits = g["limits"]
        self.move_x = 0.0
        self.move_y = 0.0

    def title(self) -> str:
        return "OFFSET: Crosshair Centering ($140 / $141)"

    def step_delta(self) -> float:
        if self.step_mode == "fine":
            return 0.02
        if self.step_mode == "coarse":
            return 1.00
        return 0.10

    def handle_key(self, key: str) -> bool:
        d = self.step_delta()
        if key in ("shift_left", "ctrl_left", "shift_right", "ctrl_right",
                   "shift_up", "ctrl_up", "shift_down", "ctrl_down"):
            d = 1.00

        x0, x1, y0, y1 = self.limits
        new_mx, new_my = self.move_x, self.move_y

        if key in ("left", "shift_left", "ctrl_left"):
            new_mx -= d
        elif key in ("right", "shift_right", "ctrl_right"):
            new_mx += d
        elif key in ("down", "shift_down", "ctrl_down"):
            new_my -= d
        elif key in ("up", "shift_up", "ctrl_up"):
            new_my += d
        else:
            return False

        # Keep inside bounds
        px = self.cx + new_mx
        py = self.cy + new_my
        if (px - self.arm >= x0 and px + self.arm <= x1 and
                py - self.arm >= y0 and py + self.arm <= y1):
            self.move_x = new_mx
            self.move_y = new_my
            return True
        return False

    def reset(self) -> None:
        self.move_x = 0.0
        self.move_y = 0.0

    def make_pattern(self, feed: float) -> "cal.Pattern":
        import calibrate as cal
        pos = (self.cx + self.move_x, self.cy + self.move_y)
        return cal.pat_crosshair(pos, self.arm, feed, 0.0)

    def proposed_settings(self) -> dict[int, float]:
        dx = -self.move_x
        dy = -self.move_y
        return cm.offset_update(self.wiz.view(), dx, dy)

    def status_lines(self) -> list[str]:
        dx = -self.move_x
        dy = -self.move_y
        ch = self.proposed_settings()
        v140 = ch.get(140, self.wiz.get(140))
        v141 = ch.get(141, self.wiz.get(141))

        lines = [
            f"=== {self.title()} ===",
            f"Crosshair Position:  X = {self.cx + self.move_x:.3f} mm,  Y = {self.cy + self.move_y:.3f} mm",
            f"Shift from Centre:   dX = {self.move_x:+.3f} mm, dY = {self.move_y:+.3f} mm",
            f"Equivalent Landing:  dx = {dx:+.3f} mm, dy = {dy:+.3f} mm",
            f"Proposed Settings:   $140 = {v140:+.4f} V, $141 = {v141:+.4f} V",
            f"Step size: {self.step_mode} (Tab/m: fine 0.02 / normal 0.10 / coarse 1.00 mm)   Speed: F{self.feed:.0f} mm/min (< / >)",
            "--------------------------------------------------------------------------------",
            "Controls: Arrow keys: move crosshair to reference point | Shift/Ctrl: coarse",
            "          < / >: feed | Tab/m: step size | r: reset | Enter: accept | s: skip | q: quit",
        ]
        return lines

    def accept(self) -> dict[str, Any]:
        ch = self.proposed_settings()
        applied = bool(self.wiz.apply(ch, "live centre offset", confirm=False))
        return {
            "move_x": self.move_x,
            "move_y": self.move_y,
            "dx": -self.move_x,
            "dy": -self.move_y,
            "proposed": ch,
            "applied": applied,
        }


# ------------------------------------------------------------- step: scale --

class LiveScaleStep(LiveStepBase):
    """Live interactive tuning of $100 / $101 (volts per mm)."""
    def __init__(self, wizard: "cal.Wizard", initial_feed: Optional[float] = None):
        super().__init__(wizard, initial_feed)
        g = self.wiz.geometry()
        self.cx, self.cy = g["centre"]
        self.s_nom = min(20.0, 0.8 * min(g["W"], g["H"]))
        self.side_x = self.s_nom
        self.side_y = self.s_nom
        self.swap = self.wiz.get(143)
        self.kx = cm.scale_key_for_axis("x", self.swap)
        self.ky = cm.scale_key_for_axis("y", self.swap)

    def title(self) -> str:
        return "SCALE: Volts per mm ($100 / $101)"

    def step_delta(self) -> float:
        if self.step_mode == "fine":
            return 0.05
        if self.step_mode == "coarse":
            return 1.00
        return 0.20

    def handle_key(self, key: str) -> bool:
        d = self.step_delta()
        if key in ("shift_up", "shift_right", "ctrl_up", "ctrl_right"):
            d = 1.00
        elif key in ("shift_down", "shift_left", "ctrl_down", "ctrl_left"):
            d = 1.00

        if key in ("+", "=", "]", "up", "right", "shift_up", "shift_right", "ctrl_up", "ctrl_right"):
            if self.axis_mode in ("both", "x"):
                self.side_x = max(2.0, self.side_x + d)
            if self.axis_mode in ("both", "y"):
                self.side_y = max(2.0, self.side_y + d)
            return True
        elif key in ("-", "_", "[", "down", "left", "shift_down", "shift_left", "ctrl_down", "ctrl_left"):
            if self.axis_mode in ("both", "x"):
                self.side_x = max(2.0, self.side_x - d)
            if self.axis_mode in ("both", "y"):
                self.side_y = max(2.0, self.side_y - d)
            return True
        elif key == "a":
            self.cycle_axis()
            return True
        elif key == "x":
            self.axis_mode = "x"
            return True
        elif key == "y":
            self.axis_mode = "y"
            return True
        elif key == "b":
            self.axis_mode = "both"
            return True
        return False

    def reset(self) -> None:
        self.side_x = self.s_nom
        self.side_y = self.s_nom
        self.axis_mode = "both"

    def proposed_settings(self) -> dict[int, float]:
        # Operator matches physical reference of size S_nom
        # Equivalent measured size is S_nom * S_nom / side
        # new_scale = old_scale * (side / S_nom)
        sc_x = self.wiz.get(self.kx) * (self.side_x / self.s_nom)
        sc_y = self.wiz.get(self.ky) * (self.side_y / self.s_nom)
        return {self.kx: sc_x, self.ky: sc_y}

    def make_pattern(self, feed: float) -> "cal.Pattern":
        import calibrate as cal
        hx, hy = self.side_x / 2.0, self.side_y / 2.0
        c = (self.cx, self.cy)
        arm = min(hx, hy) / 2.0
        poly = [(c[0] - hx, c[1] - hy), (c[0] + hx, c[1] - hy),
                (c[0] + hx, c[1] + hy), (c[0] - hx, c[1] + hy)]
        ops = cal._head(feed, 0.0) + gg.polyline(poly, close=True) + cal._cross(c, arm, arm)
        return cal._pat("scale_square", ops, c, side_x=self.side_x, side_y=self.side_y)

    def status_lines(self) -> list[str]:
        ch = self.proposed_settings()
        ex = (self.side_x / self.s_nom - 1.0) * 100
        ey = (self.side_y / self.s_nom - 1.0) * 100

        lines = [
            f"=== {self.title()} ===",
            f"Active axis: [{self.axis_mode.upper()}] (press 'a' to cycle: both/x/y, or 'x'/'y'/'b')",
            f"Reference Target:    {self.s_nom:.2f} mm",
            f"Commanded Size:      X = {self.side_x:.2f} mm ({ex:+.2f}%), Y = {self.side_y:.2f} mm ({ey:+.2f}%)",
            f"Proposed Scales:     ${self.kx} = {ch[self.kx]:.6f} V/mm, ${self.ky} = {ch[self.ky]:.6f} V/mm",
            f"Step size: {self.step_mode} (Tab/m: fine 0.05 / normal 0.20 / coarse 1.00 mm)   Speed: F{self.feed:.0f} mm/min (< / >)",
            "--------------------------------------------------------------------------------",
            "Controls: [ / ] or - / + resize square to match reference caliper/ruler",
            "          a/x/y/b: axis | < / >: feed | Tab/m: step size | r: reset | Enter: accept",
        ]
        return lines

    def accept(self) -> dict[str, Any]:
        ch = self.proposed_settings()
        applied = bool(self.wiz.apply(ch, "live scale calibration", confirm=False))
        return {
            "side_x": self.side_x,
            "side_y": self.side_y,
            "s_nom": self.s_nom,
            "proposed": ch,
            "applied": applied,
        }


# ------------------------------------------------------- step: orientation --

class LiveOrientationStep(LiveStepBase):
    """Live interactive tuning of $143 (swap) and $3 (invert)."""
    def __init__(self, wizard: "cal.Wizard", initial_feed: Optional[float] = None):
        super().__init__(wizard, initial_feed)
        self.init_swap = int(self.wiz.get(143))
        self.init_inv = int(self.wiz.get(3))
        self.swap = self.init_swap
        self.inv = self.init_inv
        g = self.wiz.geometry()
        self.c = g["centre"]
        self.L = max(10.0, min(40.0, 0.5 * min(g["W"], g["H"])))

    def title(self) -> str:
        return "ORIENTATION: +X Right, +Y Away ($143 swap, $3 invert)"

    def handle_key(self, key: str) -> bool:
        if key in ("tab", "a"):
            self.swap = 1 - self.swap
            self._push_settings()
            return True
        elif key in ("x", "left", "right"):
            self.inv ^= 1  # flip X invert (bit 0)
            self._push_settings()
            return True
        elif key in ("y", "up", "down"):
            self.inv ^= 2  # flip Y invert (bit 1)
            self._push_settings()
            return True
        return False

    def _push_settings(self) -> None:
        self.wiz.write([(143, self.wiz.get(143), float(self.swap)),
                        (3, self.wiz.get(3), float(self.inv))],
                       "live orientation trial", quiet=True, record=False)

    def reset(self) -> None:
        self.swap = self.init_swap
        self.inv = self.init_inv
        self._push_settings()

    def cleanup(self) -> None:
        # If not accepted, caller will handle or keep
        pass

    def make_pattern(self, feed: float) -> "cal.Pattern":
        import calibrate as cal
        # Traces +X arrow followed by +Y arrow
        c = self.c
        L = self.L
        pat_x = cal.pat_arrow(c, "x", L, feed, 0.0)
        pat_y = cal.pat_arrow(c, "y", L, feed, 0.0)
        ops = pat_x.ops + pat_y.ops
        return cal._pat("orient_arrows", ops, c)

    def status_lines(self) -> list[str]:
        lines = [
            f"=== {self.title()} ===",
            f"Current settings:    $143 = {self.swap} (swap), $3 = {self.inv} (invert bits: X={self.inv&1}, Y={(self.inv>>1)&1})",
            "Goal:                +X arrow points RIGHT, +Y arrow points AWAY from you",
            f"Trace Speed:         F{self.feed:.0f} mm/min (< / >)",
            "--------------------------------------------------------------------------------",
            "Controls: Tab / a: Toggle axis swap ($143: 0 <-> 1)",
            "          x / Left / Right: Invert X axis (bit 0 of $3)",
            "          y / Up / Down:    Invert Y axis (bit 1 of $3)",
            "          r: reset | Enter: accept | s: skip | q: quit",
        ]
        return lines

    def accept(self) -> dict[str, Any]:
        applied = bool(self.wiz.apply({143: float(self.swap), 3: float(self.inv)},
                                      "orientation: +X right, +Y away", confirm=False))
        return {
            "swap": self.swap,
            "inv": self.inv,
            "applied": applied,
        }


# -------------------------------------------------------- step: distortion --

class LiveDistortionStep(LiveStepBase):
    """Live interactive tuning of F-theta radial correction $142 (k1)."""
    def __init__(self, wizard: "cal.Wizard", initial_feed: Optional[float] = None):
        super().__init__(wizard, initial_feed)
        g = self.wiz.geometry()
        self.c = g["centre"]
        self.R = round(0.4 * min(g["W"], g["H"]), 2)
        self.init_k1 = self.wiz.get(142)
        self.k1 = self.init_k1

    def title(self) -> str:
        return "DISTORTION: F-theta Radial Curvature $142 (k1)"

    def step_delta(self) -> float:
        if self.step_mode == "fine":
            return 0.000005
        if self.step_mode == "coarse":
            return 0.000100
        return 0.000020

    def handle_key(self, key: str) -> bool:
        d = self.step_delta()
        if key in ("+", "=", "]", "up", "right"):
            self.k1 = clamp(self.k1 + d, -1.0, 1.0)
            self._push_k1()
            return True
        elif key in ("-", "_", "[", "down", "left"):
            self.k1 = clamp(self.k1 - d, -1.0, 1.0)
            self._push_k1()
            return True
        return False

    def _push_k1(self) -> None:
        self.wiz.write([(142, self.wiz.get(142), self.k1)],
                       "live distortion trial", quiet=True, record=False)

    def reset(self) -> None:
        self.k1 = self.init_k1
        self._push_k1()

    def make_pattern(self, feed: float) -> "cal.Pattern":
        import calibrate as cal
        return cal.pat_distortion(self.c, self.R, feed, 0.0)

    def status_lines(self) -> list[str]:
        lines = [
            f"=== {self.title()} ===",
            f"Square Size:         2R = {2 * self.R:.1f} mm with centre cross",
            f"Current Curvature:   $142 = {self.k1:.7f} (k1)",
            "Judge against ruler: Increase/decrease k1 until square edges are perfectly straight",
            f"Step size: {self.step_mode} (Tab/m: fine / normal / coarse)   Speed: F{self.feed:.0f} mm/min (< / >)",
            "--------------------------------------------------------------------------------",
            "Controls: [ / ] or - / + adjust k1 curvature to straighten square edges",
            "          < / >: feed | Tab/m: step size | r: reset | Enter: accept | s: skip | q: quit",
        ]
        return lines

    def accept(self) -> dict[str, Any]:
        applied = bool(self.wiz.apply({142: self.k1}, "live distortion fit", confirm=False))
        return {
            "R": self.R,
            "k1": self.k1,
            "applied": applied,
        }


# ------------------------------------------------------------- step: field --

class LiveFieldStep(LiveStepBase):
    """Live interactive tuning of working field ($130/$131) and origin mode ($144)."""
    def __init__(self, wizard: "cal.Wizard", initial_feed: Optional[float] = None):
        super().__init__(wizard, initial_feed)
        g = self.wiz.geometry()
        self.init_w = g["W"]
        self.init_h = g["H"]
        self.w = self.init_w
        self.h = self.init_h
        self.init_origin = int(self.wiz.get(144))
        self.origin = self.init_origin

    def title(self) -> str:
        return "FIELD & ORIGIN: Lens Working Area ($130/$131, $144)"

    def step_delta(self) -> float:
        if self.step_mode == "fine":
            return 1.0
        if self.step_mode == "coarse":
            return 20.0
        return 5.0

    def handle_key(self, key: str) -> bool:
        d = self.step_delta()
        if key in ("+", "=", "]", "up", "right"):
            if self.axis_mode in ("both", "x"):
                self.w = clamp(self.w + d, 5.0, 1000.0)
            if self.axis_mode in ("both", "y"):
                self.h = clamp(self.h + d, 5.0, 1000.0)
            return True
        elif key in ("-", "_", "[", "down", "left"):
            if self.axis_mode in ("both", "x"):
                self.w = clamp(self.w - d, 5.0, 1000.0)
            if self.axis_mode in ("both", "y"):
                self.h = clamp(self.h - d, 5.0, 1000.0)
            return True
        elif key in ("o", "0", "1"):
            self.origin = 1 - self.origin
            return True
        elif key == "a":
            self.cycle_axis()
            return True
        elif key == "x":
            self.axis_mode = "x"
            return True
        elif key == "y":
            self.axis_mode = "y"
            return True
        elif key == "b":
            self.axis_mode = "both"
            return True
        return False

    def reset(self) -> None:
        self.w = self.init_w
        self.h = self.init_h
        self.origin = self.init_origin
        self.axis_mode = "both"

    def make_pattern(self, feed: float) -> "cal.Pattern":
        import calibrate as cal
        if self.origin == 0:
            poly = [(0.0, 0.0), (self.w, 0.0), (self.w, self.h), (0.0, self.h)]
            c = (self.w / 2.0, self.h / 2.0)
        else:
            poly = [(-self.w / 2.0, -self.h / 2.0), (self.w / 2.0, -self.h / 2.0),
                    (self.w / 2.0, self.h / 2.0), (-self.w / 2.0, self.h / 2.0)]
            c = (0.0, 0.0)
        ops = cal._head(feed, 0.0) + gg.polyline(poly, close=True)
        return cal._pat("field_box", ops, c)

    def status_lines(self) -> list[str]:
        orig_name = "0 (corner origin)" if self.origin == 0 else "1 (lens centre origin)"
        lines = [
            f"=== {self.title()} ===",
            f"Active axis: [{self.axis_mode.upper()}] (press 'a' to cycle: both/x/y, or 'x'/'y'/'b')",
            f"Field Dimensions:    W = {self.w:.1f} mm, H = {self.h:.1f} mm",
            f"Origin Mode:         {orig_name} (press 'o' to toggle 0 <-> 1)",
            f"Proposed Settings:   $130 = {self.w:.1f}, $131 = {self.h:.1f}, $144 = {self.origin}",
            f"Step size: {self.step_mode} (Tab/m: fine 1 / normal 5 / coarse 20 mm)   Speed: F{self.feed:.0f} mm/min (< / >)",
            "--------------------------------------------------------------------------------",
            "Controls: [ / ] or - / + resize field | o: toggle origin | a/x/y/b: axis",
            "          < / >: feed | Tab/m: step size | r: reset | Enter: accept | s: skip | q: quit",
        ]
        return lines

    def accept(self) -> dict[str, Any]:
        ch = {130: self.w, 131: self.h, 144: float(self.origin)}
        applied = bool(self.wiz.apply(ch, "live field / origin", confirm=False))
        self.wiz.regions = None
        return {
            "W": self.w,
            "H": self.h,
            "origin": self.origin,
            "applied": applied,
        }


# ----------------------------------------------------------- live runner --

def run_live_loop(wizard: "cal.Wizard", step: LiveStepBase,
                  key_input: Optional[LiveInput] = None,
                  display: Optional[LiveDisplay] = None) -> dict[str, Any]:
    """Execute continuous guide-laser tracing with non-blocking keyboard input loop."""
    import calibrate as cal

    # Determine input and display if not provided
    is_tty = sys.stdin.isatty() and sys.stdout.isatty()
    if key_input is None:
        key_input = HeadlessKeyInput()

    if display is None:
        display = HeadlessDisplay(quiet=not is_tty)

    wizard.preview_on()
    clean = False
    try:
        display.update(step)
        while True:
            # 1. Process all pending keys
            dirty = False
            while True:
                k = key_input.get_key()
                if k is None:
                    break
                if k == "q":
                    raise cal.QuitWizard("operator quit in live mode")
                if k == "s":
                    raise cal.SkipStep()
                if k == "r":
                    step.reset()
                    dirty = True
                elif k == "enter":
                    if step.handle_enter():
                        clean = True
                        return step.accept()
                    dirty = True
                elif k in ("tab", "m"):
                    step.cycle_step_size()
                    dirty = True
                elif k == "1":
                    step.step_mode = "fine"
                    dirty = True
                elif k == "2":
                    step.step_mode = "normal"
                    dirty = True
                elif k == "3":
                    step.step_mode = "coarse"
                    dirty = True
                elif k in ("<", ","):
                    step.adjust_feed(1.0 / 1.25)
                    dirty = True
                elif k in (">", "."):
                    step.adjust_feed(1.25)
                    dirty = True
                else:
                    if step.handle_key(k):
                        dirty = True

                # In scripted test mode, stream 1 lap per key so every change is tested in G-code
                if isinstance(key_input, ScriptedKeyInput):
                    break

            if dirty:
                display.update(step)

            # 2. Generate and stream 1 short lap
            if step.needs_stream():
                pat = step.make_pattern(step.feed)
                ops = cal._retime(pat.ops, step.feed, 0.0)
                lines = wizard.encode(ops, pat.centre, repeat=1, cap=0.0)
                est = max(0.01, wizard.est_seconds(ops, step.feed))
                wizard.stream(lines, est)
            else:
                time.sleep(0.03)

            # In mock or headless scripted test runs, if input queue is empty after
            # processing keys and streaming, exit cleanly with accept.
            if isinstance(key_input, ScriptedKeyInput) and key_input.empty:
                clean = True
                return step.accept()

    finally:
        step.cleanup()
        wizard.preview_off()
        display.close()


def run_live_step_curses(wizard: "cal.Wizard", step: LiveStepBase) -> dict[str, Any]:
    """Launch interactive curses environment when running in a real TTY."""
    result: dict[str, Any] = {}
    err: Optional[BaseException] = None

    def curses_main(stdscr: Any) -> None:
        nonlocal result, err
        try:
            key_in = CursesKeyInput(stdscr)
            disp = CursesDisplay(stdscr)
            result = run_live_loop(wizard, step, key_input=key_in, display=disp)
        except BaseException as e:
            err = e

    curses.wrapper(curses_main)
    if err is not None:
        raise err
    return result
