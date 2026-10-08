#!/usr/bin/env python3
"""
WaveMaster interactive calibration wizard.

    python3 tools/calibrate.py PORT [--step NAME]... [--fire] [--yes] [--max-power S] [--out DIR]
    python3 tools/calibrate.py --mock [...]            # offline, against tools/rftest/mock_grbl.py
    python3 tools/calibrate.py PORT --restore FILE     # write a backed-up settings file back

Steps (default: all, in this order, each skippable):
    connect      handshake, $I, $S (ATmega link / armed), current calibration values, backup of $$
    focus        (needs --fire) circles at a changing Z height -> best focal distance (recorded, not a setting)
    range        guide laser only: largest correct galvo deflection, volts per axis -> suggested $130/$131
    field        lens field size ($130/$131), origin ($144), laser PRR ($220/$221)
    orientation  guide-laser arrows -> $143 swap and $3 invert so +X = right, +Y = away
    scale        centred square -> $100/$101
    distortion   big square + cross -> $142 (k1) and the scales
    offset       crosshair -> $140/$141
    delays       (needs --fire) $210 .. $216
    power        (needs --fire) power ladder -> $224/$225
    speed        (marking ladder needs --fire) $110 and jump speed $201
    summary      all changes, final $$, next steps (always printed)

At every prompt: a number (12.34 or 12,34), "s" = skip this step, "q" = quit (summary and an
offer to restore the backup), "again" = draw the pattern again.

Answers of the range step: ok, x, y, both, again. The focus loop: Enter, n, "h <value>", done.

Real-laser steps follow tools/rftest/run_tests.py: --fire is required, FIRE must be typed once,
every pattern is framed with the guide laser first, M10 arms explicitly and $S must report
armed=1, every S is capped at --max-power, Ctrl-C / errors send 0x18, M5, M9 and M11.

Output (default tools/calibrate/runs/<timestamp>/): session.log, events.jsonl, firmware_msgs.log,
statuses.csv, settings_before.json, settings_after.json, calibration_report.md.
See tools/CALIBRATION.md for the physical procedure.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "rftest"))

import calibrate_math as cm                                   # noqa: E402
import gcode_gen as gg                                        # noqa: E402
from calibrate_math import CalibError                         # noqa: E402
from gcode_gen import Box, MoveTo, SetPower, SetSpeed         # noqa: E402
from rfclient import RayforgeClient, SessionLogger, strip_comment  # noqa: E402
from run_tests import SafetyError, assert_safe                # noqa: E402

STEP_ORDER = ("connect", "focus", "range", "field", "orientation", "scale", "distortion", "offset", "delays",
              "power", "speed", "summary")
FIRE_STEPS = ("focus", "delays", "power")
STEP_INFO = {
    "connect": "handshake, current values, backup",
    "focus": "focal distance (Z height) with circles, real laser",
    "range": "galvo maximum deflection (volts per axis), guide laser",
    "field": "lens field size, origin, laser PRR",
    "orientation": "guide laser: +X right, +Y away ($143, $3)",
    "scale": "square size ($100, $101)",
    "distortion": "F-theta correction ($142) and scales",
    "offset": "centre offset ($140, $141)",
    "delays": "laser/jump/mark/polygon delays ($210-$216), real laser",
    "power": "power ladder ($224, $225), real laser",
    "speed": "marking speed ($110) and jump speed ($201)",
    "summary": "report",
}
DEFAULTS: dict[int, float] = {
    3: 0, 12: 0.01, 30: 1000, 100: 0.1, 101: 0.1, 110: 300000, 120: 2000000, 130: 100, 131: 100, 140: 0,
    141: 0, 142: 0, 143: 0, 144: 0, 150: 0, 151: 0.5, 200: 10, 201: 3000, 203: 500, 210: 100, 211: 120,
    212: 300, 213: 0, 214: 100, 215: 0, 216: 30, 220: 30000, 221: 50, 223: 0, 224: 0, 225: 100, 226: 1,
    227: 4000, 229: 10, 230: 0,
}
GRID_SIZE = (24.0, 12.0)       # test-card regions, mm (see RegionAllocator)
JUMP_SIZE = (28.0, 12.0)
CORNER_SIZE = (28.0, 14.0)
POWER_SIZE = (27.0, 27.0)
SPEED_SIZE = (27.0, 20.0)
FOCUS_LAPS = 10
MAX_RETRY = 40
DAMPING = 0.8


class QuitWizard(Exception):
    pass


class SkipStep(Exception):
    pass


# ------------------------------------------------------------------ input --

@dataclass
class Question:
    key: str                      # stable id, e.g. "scale_x" (the selftest answers by key)
    prompt: str
    kind: str = "text"            # text | floats | yesno | menu | direction | confirm
    default: Optional[str] = None
    choices: tuple = ()
    nav: bool = True              # "s" / "q" / "again" are understood
    meta: dict = field(default_factory=dict)


class ConsoleInput:
    """Answers come from the keyboard."""

    def ask(self, q: Question) -> str:
        suffix = f" [{q.default}]" if q.default not in (None, "") else ""
        try:
            return input(f"{q.prompt}{suffix}: ")
        except EOFError:
            raise QuitWizard("end of input")


# --------------------------------------------------------------- patterns --

@dataclass
class Pattern:
    name: str
    ops: list
    centre: tuple[float, float]
    box: Box
    meta: dict = field(default_factory=dict)


def _pat(name: str, ops: list, centre: tuple[float, float], **meta: Any) -> Pattern:
    box = gg.bbox(ops)
    assert box is not None
    return Pattern(name, ops, centre, box, meta)


def _head(feed: float, power: float) -> list:
    return [SetSpeed(feed), SetPower(power)]


def pat_arrow(c: tuple[float, float], axis: str, length: float, feed: float, power: float) -> Pattern:
    d = (1.0, 0.0) if axis == "x" else (0.0, 1.0)
    n = (-d[1], d[0])
    tail = (c[0] - d[0] * length / 2, c[1] - d[1] * length / 2)
    tip = (c[0] + d[0] * length / 2, c[1] + d[1] * length / 2)
    h = 0.3 * length
    a = (tip[0] - h * d[0] + 0.5 * h * n[0], tip[1] - h * d[1] + 0.5 * h * n[1])
    b = (tip[0] - h * d[0] - 0.5 * h * n[0], tip[1] - h * d[1] - 0.5 * h * n[1])
    ops = _head(feed, power) + gg.polyline([tail, tip, a, tip, b])
    return _pat(f"arrow_{axis}", ops, c, kind="arrow", axis=axis, tail=tail, tip=tip)


def _square(c: tuple[float, float], half: float) -> list:
    return gg.polyline([(c[0] - half, c[1] - half), (c[0] + half, c[1] - half), (c[0] + half, c[1] + half),
                        (c[0] - half, c[1] + half)], close=True)


def _cross(c: tuple[float, float], ax: float, ay: float) -> list:
    return [MoveTo(c[0] - ax, c[1]), gg.LineTo(c[0] + ax, c[1]), MoveTo(c[0], c[1] - ay), gg.LineTo(c[0], c[1] + ay)]


def pat_square_cross(c: tuple[float, float], side: float, feed: float, power: float) -> Pattern:
    arm = min(side / 4, 5.0)
    ops = _head(feed, power) + _square(c, side / 2) + _cross(c, arm, arm)
    return _pat("scale_square", ops, c, kind="square", half=side / 2, side=side)


def pat_distortion(c: tuple[float, float], R: float, feed: float, power: float) -> Pattern:
    ops = _head(feed, power) + _square(c, R) + _cross(c, R, R)
    return _pat("distortion", ops, c, kind="distortion", R=R)


def pat_crosshair(c: tuple[float, float], arm: float, feed: float, power: float) -> Pattern:
    ops = _head(feed, power) + _cross(c, arm, arm)
    return _pat("crosshair", ops, c, kind="cross", arm=arm)


def pat_grid(box: Box, feed: float, power: float) -> Pattern:
    ops = gg.line_grid(box, feed, power, rows=6, cols=6)
    return _pat("delay_grid", ops, (box.cx, box.cy), kind="grid", box=box)


def pat_corners(box: Box, feed: float, power: float) -> Pattern:
    ops = gg.corner_test(box, feed, power)
    return _pat("corner_test", ops, (box.cx, box.cy), kind="corners", box=box)


def pat_jump(box: Box, feed: float, power: float, lengths: Sequence[float] = (2, 6, 12, 24),
             mark: float = 2.0, reps: int = 3) -> Pattern:
    """Rows of short marks separated by jumps of increasing length (shortest row at the bottom)."""
    ops = _head(feed, power)
    row_h = box.h / len(lengths)
    for i, L in enumerate(lengths):
        for k in range(reps):
            y = box.y + i * row_h + k * 0.5
            ops += [MoveTo(box.x, y), gg.LineTo(box.x + mark, y), MoveTo(box.x + L, y),
                    gg.LineTo(box.x + L + mark, y)]
    return _pat("jump_test", ops, (box.cx, box.cy), kind="jump", box=box, lengths=tuple(lengths))


def pat_power(box: Box, feed: float, powers: Sequence[float]) -> Pattern:
    ops = gg.power_ladder(box, feed, powers)
    return _pat("power_ladder", ops, (box.cx, box.cy), kind="power", box=box, powers=tuple(powers))


def pat_speed(box: Box, feeds: Sequence[float], power: float) -> Pattern:
    ops = gg.speed_ladder(box, feeds, power)
    return _pat("speed_ladder", ops, (box.cx, box.cy), kind="speed", box=box, feeds=tuple(feeds))


def pat_range(sq: cm.RangeSquare, feed: float) -> Pattern:
    """Guide-laser rectangle of the range step (corners from calibrate_math.range_square) plus the short
    tick that marks the +X side."""
    ops = _head(feed, 0.0) + gg.polyline(sq.corners_mm, close=True)
    ops += [MoveTo(*sq.tick_mm[0]), gg.LineTo(*sq.tick_mm[1])]
    return _pat("range_rectangle", ops, sq.centre_mm, kind="range", fx=sq.fx, fy=sq.fy, square=sq,
                corners_mm=list(sq.corners_mm))


def pat_focus_burst(c: tuple[float, float], r: float, laps: int, feed: float, power: float) -> Pattern:
    """`laps` identical laps of one circle (two G2 half arcs each, starting at +X)."""
    arcs = gg.circle_arcs(c[0], c[1], r)
    ops = _head(feed, power) + [arcs[0]] + list(arcs[1:]) * max(1, laps)
    return _pat("focus_circle", ops, c, kind="focus", r=r, laps=laps, box=Box(c[0] - r, c[1] - r, 2 * r, 2 * r))


def repeat_ops(ops: list, n: int) -> list:
    return list(ops) * max(1, n)


# ----------------------------------------------------------------- wizard --

class Wizard:
    def __init__(self, args: argparse.Namespace, client: RayforgeClient, logger: SessionLogger, out: Path,
                 provider: Any = None):
        self.a = args
        self.c = client
        self.log = logger
        self.out = out
        self.provider = provider or ConsoleInput()
        self.dev: dict[int, float] = {}       # last `$$` readback (4 decimals!)
        self.dev_int: set[int] = set()
        self.exact: dict[int, float] = {}     # exact values known to the wizard (see load_state)
        self.initial: dict[int, float] = {}
        self.initial_exact: dict[int, float] = {}
        self.changes: list[dict] = []
        self.results: dict[str, Any] = {}
        self.step = "connect"
        self.cap = float(args.max_power)
        self.last_pattern: Optional[Pattern] = None
        self._redraw: Optional[Callable[[], None]] = None
        self.fire_confirmed = False
        self.armed = False
        self.preview = False
        self.fired = False
        self.link_ok = False
        self.mark_power: Optional[float] = None
        self.regions: Optional[cm.RegionAllocator] = None
        self.connected = False
        self.device_info: dict[str, Any] = {}
        self.state_file = Path(args.state) if getattr(args, "state", None) else HERE / "calibrate" / "last_values.json"
        self.notes: list[str] = []
        self.range_ch: Optional[list[float]] = None    # usable half range in volts of DAC channel 0 / 1
        self.range_swap = 0.0                          # $143 when the range was measured
        self.range_source = ""
        self.focus_best: Optional[dict] = None

    # ---- console helpers
    def say(self, text: str = "") -> None:
        print(text, flush=True)
        if text.strip():
            self.log.note(text)

    def warn(self, text: str) -> None:
        self.say(f"WARNING: {text}")
        self.notes.append(f"[{self.step}] WARNING: {text}")

    def header(self, title: str) -> None:
        self.say("")
        self.say("=" * 72)
        self.say(title)
        self.say("=" * 72)

    def ask(self, q: Question) -> str:
        q.meta.setdefault("pattern", self.last_pattern)
        q.meta.setdefault("step", self.step)
        while True:
            raw = self.provider.ask(q).strip()
            self.log.note(f"ASK {q.key}: {q.prompt!r} -> {raw!r}")
            if raw == "" and q.default is not None:
                raw = str(q.default)
            if q.nav:
                low = raw.lower()
                if low == "s":
                    raise SkipStep()
                if low == "q":
                    raise QuitWizard("operator quit")
                if low == "again" and self._redraw:
                    self._redraw()
                    continue
            return raw

    def ask_yes(self, key: str, prompt: str, default: bool = True, meta: Optional[dict] = None) -> bool:
        for _ in range(MAX_RETRY):
            raw = self.ask(Question(key, f"{prompt} [{'Y/n' if default else 'y/N'}]", "yesno", None,
                                    meta=dict(meta or {}))).lower()
            if raw == "":
                return default
            if raw in ("y", "yes"):
                return True
            if raw in ("n", "no"):
                return False
            self.say("  please answer y or n")
        raise CalibError(f"no valid answer for {key}")

    def ask_nums(self, key: str, prompt: str, n_min: int = 1, n_max: int = 1, default: Optional[float] = None,
                 lo: Optional[float] = None, hi: Optional[float] = None, meta: Optional[dict] = None) -> list[float]:
        for _ in range(MAX_RETRY):
            raw = self.ask(Question(key, prompt, "floats", None if default is None else cm.fmt_val(default),
                                    meta=dict(meta or {})))
            try:
                vals = cm.parse_floats(raw, n_min, n_max)
            except CalibError as e:
                self.say(f"  invalid: {e}")
                continue
            bad = [v for v in vals if (lo is not None and v < lo) or (hi is not None and v > hi)]
            if bad:
                self.say(f"  {cm.fmt_val(bad[0])} is outside the allowed range {lo if lo is not None else '-inf'} .. "
                         f"{hi if hi is not None else 'inf'}")
                continue
            return vals
        raise CalibError(f"no valid answer for {key}")

    def ask_menu(self, key: str, title: str, options: dict[str, str], extra: str = "k = stop tuning this value",
                 meta: Optional[dict] = None) -> str:
        self.say(title)
        for k, v in options.items():
            self.say(f"   ({k}) {v}")
        valid = list(options) + ["k"]
        for _ in range(MAX_RETRY):
            raw = self.ask(Question(key, f"  choice [{'/'.join(options)}, {extra}]", "menu", None,
                                    tuple(valid), meta=dict(meta or {}))).lower()
            if raw in valid:
                return raw
            self.say("  please pick one of the listed numbers")
        raise CalibError(f"no valid answer for {key}")

    # ---- settings
    def get(self, n: int) -> float:
        if n in self.exact:
            return self.exact[n]
        if n in self.dev:
            return self.dev[n]
        return DEFAULTS.get(n, 0.0)

    def view(self) -> dict[int, float]:
        return {n: self.get(n) for n in set(DEFAULTS) | set(self.dev)}

    def geometry(self) -> dict[str, Any]:
        v = self.view()
        x0, x1, y0, y1 = cm.work_limits(v)
        cx, cy = cm.field_centre(v)
        return {"W": v[130], "H": v[131], "limits": (x0, x1, y0, y1), "centre": (cx, cy), "swap": v[143],
                "inv": int(v[3])}

    def read_settings(self) -> dict[int, float]:
        lines = self.c.command("$$", timeout=10.0)
        self.dev = cm.parse_dump(lines)
        # the firmware prints integral values without decimals: those readbacks are exact
        self.dev_int = {int(m.group(1)) for m in (re.match(r"\$(\d+)=[-+]?\d+$", ln.strip()) for ln in lines) if m}
        for n in list(self.exact):
            if n in self.dev and not cm.same_value(self.dev[n], self.exact[n]):
                del self.exact[n]                       # changed behind our back
        return self.dev

    def load_state(self) -> None:
        """$$ prints 4 decimals ($142 = 0.0000 for any |k1| < 5e-5!), so the wizard remembers the
        exact values it wrote last time and uses them while the readback still agrees."""
        try:
            data = json.loads(self.state_file.read_text())
            for k, v in data.get("values", {}).items():
                n = int(k)
                if n in self.dev and cm.same_value(self.dev[n], float(v)):
                    self.exact[n] = float(v)
            self.range_swap = float(data.get("range_swap", 0.0))
            ch = data.get("range_volts_ch")
            if isinstance(ch, list) and len(ch) == 2 and all(float(x) > 0 for x in ch):
                self.range_ch = [float(ch[0]), float(ch[1])]
            elif float(data.get("range_volts_x", 0)) > 0 and float(data.get("range_volts_y", 0)) > 0:
                vx, vy = float(data["range_volts_x"]), float(data["range_volts_y"])
                cx = cm.channel_for_axis("x", self.range_swap)
                self.range_ch = [vx, vy] if cx == 0 else [vy, vx]
            if self.range_ch:
                self.range_source = "state file"
            if isinstance(data.get("focus_best"), dict):
                self.focus_best = data["focus_best"]
        except (OSError, ValueError, AttributeError, TypeError):
            pass

    def save_state(self) -> None:
        try:
            self.state_file.parent.mkdir(parents=True, exist_ok=True)
            data: dict[str, Any] = {"saved": dt.datetime.now().isoformat(timespec="seconds"),
                                    "values": {str(n): v for n, v in sorted(self.exact.items())}}
            if self.range_ch:
                cx = cm.channel_for_axis("x", self.range_swap)
                data.update({"range_volts_x": self.range_ch[cx], "range_volts_y": self.range_ch[1 - cx],
                             "range_volts_ch": self.range_ch, "range_swap": self.range_swap})
            if self.focus_best:
                data.update({"focus_best_height": self.focus_best.get("height"), "focus_best": self.focus_best})
            self.state_file.write_text(json.dumps(data, indent=1))
        except OSError as e:
            self.log.note(f"could not save {self.state_file}: {e}")

    def table(self, rows: Sequence[tuple[int, float, float]]) -> None:
        for n, old, new in rows:
            self.say(f"   ${n:<4} {cm.NAMES.get(n, ''):<40} {cm.fmt_val(old):>12}  ->  {cm.fmt_val(new):>12}")

    def apply(self, changes: dict[int, float], why: str, confirm: bool = True) -> bool:
        """Show old -> new, ask, write with $N=v, check ok, read back with $$."""
        rows = []
        for n, v in sorted(changes.items()):
            cm.check_range(n, v)
            old = self.get(n)
            if abs(old - v) > 1e-9:
                rows.append((n, old, v))
        if not rows:
            self.say("  nothing to change")
            return True
        self.say(f"  proposed change ({why}):")
        self.table(rows)
        if confirm and not self.ask_yes(f"apply_{self.step}", "  Apply these values?", True, {"changes": dict(changes)}):
            self.say("  not applied")
            return False
        return self.write(rows, why)

    def write(self, rows: Sequence[tuple[int, float, float]], why: str, quiet: bool = False,
              record: bool = True) -> bool:
        for n, old, new in rows:
            reply = self.c.command(f"${n}={cm.fmt_val(new)}", timeout=5.0)
            if reply[-1] != "ok":
                raise CalibError(f"${n}={cm.fmt_val(new)} -> {reply[-1]}")
        self.read_settings()
        ok = True
        for n, old, new in rows:
            self.exact[n] = new = float(cm.fmt_val(new))        # what the firmware really parsed
            if n in self.dev and not cm.same_value(self.dev[n], new):
                ok = False
                self.warn(f"${n}: wrote {cm.fmt_val(new)} but $$ reads back {self.dev[n]:g}")
            if record:
                self.changes.append({"step": self.step, "setting": n, "name": cm.NAMES.get(n, ""), "old": old,
                                     "new": new, "why": why})
        if not quiet:
            self.say(f"  written and read back ({'verified' if ok else 'MISMATCH'}); "
                     "$$ shows only 4 decimals, the wizard keeps the exact values")
        self.save_state()
        return ok

    # ---- connection
    def ensure_connected(self) -> None:
        if self.connected:
            return
        info = self.c.handshake()
        self.device_info = {"version": info.version, "options": info.options, "rx_window": info.rx_window,
                            "machine": info.machine}
        self.read_settings()
        self.load_state()
        self.initial = dict(self.dev)
        self.initial_exact = {n: self.get(n) for n in self.dev}
        backup = {"time": dt.datetime.now().isoformat(timespec="seconds"), "device": self.device_info,
                  "settings": {str(n): v for n, v in sorted(self.dev.items())},
                  "exact": {str(n): v for n, v in sorted(self.initial_exact.items()) if n in cm.WRITABLE}}
        (self.out / "settings_before.json").write_text(json.dumps(backup, indent=1))
        self.say(f"backup of $$ written to {self.out / 'settings_before.json'}")
        self.say(f"  restore later with: python3 tools/calibrate.py PORT --restore {self.out / 'settings_before.json'}")
        self.connected = True
        self.refresh_stats()

    def refresh_stats(self) -> dict:
        st = self.c.stats_s()
        self.stats = st
        at = st.get("atmega", {})
        self.link_ok = at.get("link") == "1"
        return st

    def tick_us(self) -> float:
        try:
            return float(self.refresh_stats().get("galvo", {}).get("tick_us", 10.0))
        except (TypeError, ValueError):
            return 10.0

    # ---- drawing
    def check_inside(self, ops: list, what: str) -> None:
        box = gg.bbox(ops)
        x0, x1, y0, y1 = self.geometry()["limits"]
        if box and (box.x < x0 - 1e-6 or box.y < y0 - 1e-6 or box.x1 > x1 + 1e-6 or box.y1 > y1 + 1e-6):
            raise CalibError(f"{what} would leave the work area ({x0:g}..{x1:g}, {y0:g}..{y1:g}); "
                             "reduce its size or fix $130/$131/$144")

    def encode(self, ops: list, centre: tuple[float, float], repeat: int = 1, cap: Optional[float] = None) -> list[str]:
        cap = self.cap if cap is None else cap
        full = repeat_ops(ops, repeat) + [MoveTo(*centre)]
        self.check_inside(full, "pattern")
        code = gg.encode(full, "grbl", max_s=cap, work_area=None, postamble=False)
        lines = [ln for ln in (strip_comment(l) for l in code.lines) if ln]
        assert_safe(lines, cap)
        return lines

    def est_seconds(self, ops: list, feed: float) -> float:
        cut, travel = gg.path_length(ops)
        jump = max(self.get(201), 1.0)
        return cut / max(feed / 60.0, 1e-3) + travel / jump + 0.002 * len(ops)

    def stream(self, lines: list[str], est: float) -> None:
        res = self.c.stream(lines)
        if res.aborted or res.errors:
            detail = res.abort_reason or (res.errors[0].reply if res.errors else "")
            raise CalibError(f"stream failed: {detail}")
        st, _ = self.c.wait_idle(max(30.0, est * 3 + 10.0))
        if st is None:
            raise CalibError("controller did not return to Idle")

    def preview_on(self) -> None:
        self.c.command_ok("M66")
        self.preview = True
        # The ATmega link is asynchronous: M66 is acknowledged ~1 ms before
        # the guide laser is confirmed, so poll briefly.
        deadline = time.monotonic() + 1.0
        while self.c.stats_s().get("atmega", {}).get("guide") != "1":
            if time.monotonic() >= deadline:
                raise SafetyError("M66 sent but $S does not report guide=1")
            time.sleep(0.05)

    def preview_off(self) -> None:
        self.c.command("M67")
        self.preview = False

    def draw_guide(self, pat: Pattern, seconds: Optional[float] = None) -> None:
        """Trace the pattern repeatedly with the guide laser (M66 preview, S0, marking laser off)."""
        seconds = self.a.guide_seconds if seconds is None else seconds
        one = max(self.est_seconds(pat.ops, self.a.guide_feed), 1e-3)
        n = int(max(1, min(300, math.ceil(seconds / one))))
        ops = _retime(pat.ops, self.a.guide_feed, 0.0)
        lines = self.encode(ops, pat.centre, n, cap=0.0)
        self.say(f"  guide laser: tracing '{pat.name}' x{n} (~{n * one:.1f} s) - type 'again' at the next prompt to repeat")
        self.preview_on()
        try:
            self.stream(lines, n * one)
        finally:
            self.preview_off()

    def fire_prelude(self) -> None:
        if not self.a.fire:
            raise SafetyError("this step emits the real laser: re-run with --fire")
        if not self.link_ok:
            raise SafetyError("ATmega link is down ($S link=0): cannot arm or fire")
        if self.fire_confirmed:
            return
        self.say("")
        self.say("*** THE MARKING LASER WILL EMIT ***")
        self.say(f"S cap (--max-power): {self.cap:g}")
        if not self.a.yes:
            if self.ask(Question("fire", "Enclosure closed, eye protection on, test card in place. Type FIRE to continue",
                                 "confirm")).strip() != "FIRE":
                raise SafetyError("operator did not type FIRE")
        self.fire_confirmed = True

    def arm(self) -> None:
        self.say("  arming (M10)")
        self.c.command_ok("M10", timeout=10.0)
        st, _ = self.c.wait_idle(10.0)
        if st is None:
            raise SafetyError("controller not Idle after M10")
        end = time.monotonic() + self.a.arm_timeout
        while time.monotonic() < end:
            if self.c.stats_s().get("atmega", {}).get("armed") == "1":
                self.armed = True
                return
            time.sleep(0.25)
        raise SafetyError("ATmega did not report armed=1 after M10")

    def disarm(self) -> None:
        try:
            self.c.command("M5", timeout=3.0)
            self.c.command("M11", timeout=3.0)
        finally:
            self.armed = False

    def frame_pattern(self, box: Box, centre: tuple[float, float]) -> Pattern:
        """Guide-laser outline of a bounding box (S0)."""
        return Pattern("frame", _head(self.a.guide_feed, 0) + gg.polyline(
            [(box.x, box.y), (box.x1, box.y), (box.x1, box.y1), (box.x, box.y1)], close=True), centre, box)

    def draw_fire(self, pat: Pattern, repeat: int = 1) -> None:
        self.fire_prelude()
        mp = self.mark_power if self.mark_power is not None else min(100.0, self.cap)
        lines = self.encode(pat.ops, pat.centre, repeat)           # S already capped by the encoder
        frame = self.frame_pattern(pat.box, pat.centre)
        self.say(f"  framing the pattern box X {pat.box.x:.1f}..{pat.box.x1:.1f} Y {pat.box.y:.1f}..{pat.box.y1:.1f} "
                 f"with the guide laser (S <= {mp:g} when firing)")
        self.draw_guide(frame)
        if not self.a.yes and not self.ask_yes("frame_ok", "  Frame is on the test card?", False):
            raise SafetyError("operator rejected the frame")
        self.arm()
        try:
            self.say(f"  FIRING '{pat.name}'")
            self.fired = True
            est = self.est_seconds(pat.ops, self.a.feed) * repeat
            self.stream(lines, est)
            self.c.command("M5")
        finally:
            if not self.a.keep_armed:
                self.disarm()

    def draw(self, pat: Pattern, fire: bool, repeat: int = 1) -> None:
        self.last_pattern = pat
        self._redraw = (lambda: self.draw_fire(pat, repeat)) if fire else (lambda: self.draw_guide(pat))
        self._redraw()

    def style(self, fire: bool, feed: Optional[float] = None) -> tuple[float, float]:
        """(feed, S) for building a pattern: guide patterns use S0, fire patterns the mark power."""
        if not fire:
            return self.a.guide_feed, 0.0
        return (self.a.feed if feed is None else feed), (self.mark_power if self.mark_power is not None else 0.0)

    def use_fire(self, what: str) -> bool:
        if not self.a.fire:
            self.say("  (no --fire: using the guide laser on paper)")
            return False
        if not self.link_ok:
            self.warn("ATmega link=0: cannot fire, using the guide laser on paper")
            return False
        fire = self.ask_yes(f"fire_mode_{self.step}", f"Mark {what} with the real laser on a test card? "
                            "(n = guide laser on paper)", True)
        if fire:
            self.ensure_mark_power()
        return fire

    def ensure_mark_power(self) -> None:
        if self.mark_power is not None:
            return
        d = min(100.0, self.cap)
        self.mark_power = self.ask_nums("mark_power", f"S value for test marks (1..{self.cap:g}; start low)", default=d,
                                        lo=1, hi=self.cap)[0]
        self.say(f"  marking at S{self.mark_power:g}, F{self.a.feed:g} mm/min (--feed); every S is capped at {self.cap:g}")

    def ensure_card(self) -> None:
        if self.regions is not None:
            return
        g = self.geometry()
        x0, x1, y0, y1 = g["limits"]
        dw, dh = self.a.card
        cw, ch = min(dw, x1 - x0 - 2), min(dh, y1 - y0 - 2)
        c = g["centre"]
        self.card_box = (c[0] - cw / 2, c[1] - ch / 2, c[0] + cw / 2, c[1] + ch / 2)
        self.regions = cm.RegionAllocator(self.card_box)
        self.say(f"  test card area used: {cw:g} x {ch:g} mm centred on the field centre "
                 "(change with --card WxH); every pattern gets its own free spot")

    def new_region(self, w: float, h: float) -> Box:
        self.ensure_card()
        assert self.regions is not None
        r = self.regions.allocate(w, h)
        if r is None:
            self.warn("the test card is full")
            self.ask(Question("card_full", "Put a fresh card (or move it) and press Enter", "confirm", ""))
            self.regions.reset()
            r = self.regions.allocate(w, h)
            if r is None:
                raise CalibError(f"a {w:g} x {h:g} mm pattern does not fit on the card (--card)")
        return Box(*r)

    # ---- steps -------------------------------------------------------------
    def run_step(self, name: str, explicit: bool) -> None:
        self.step = name
        desc = STEP_INFO[name]
        if name in FIRE_STEPS and not self.a.fire:
            self.say(f"\n[{name}] needs --fire (real laser); skipped")
            self.results[name] = {"status": "skipped (no --fire)"}
            return
        if name in FIRE_STEPS and not self.link_ok:
            self.warn(f"step '{name}' needs the ATmega link (link down, link=0): skipped")
            self.results[name] = {"status": "skipped (ATmega link down)"}
            return
        if not explicit and name not in ("connect", "summary"):
            try:
                if not self.ask_yes(f"run_{name}", f"\nRun step '{name}' ({desc})?", True):
                    self.results[name] = {"status": "skipped by operator"}
                    return
            except SkipStep:
                self.results[name] = {"status": "skipped by operator"}
                return
        try:
            res = getattr(self, f"step_{name}")()
            self.results.setdefault(name, {})
            if isinstance(res, dict):
                self.results[name].update(res)
            self.results[name].setdefault("status", "done")
            if name in ("scale", "distortion"):
                self.offer_range_workarea(f"the scale changed in '{name}'")
        except SkipStep:
            self.say(f"  step '{name}' skipped")
            self.results[name] = {"status": "skipped by operator"}

    def step_connect(self) -> dict:
        self.header("connect")
        self.ensure_connected()
        st = self.refresh_stats()
        at, gv = st.get("atmega", {}), st.get("galvo", {})
        self.say(f"  {self.device_info.get('version', '')} {self.device_info.get('options', '')} "
                 f"({self.device_info.get('machine', '')})")
        self.say(f"  ATmega: link={at.get('link')} armed={at.get('armed')} ready={at.get('ready')} guide={at.get('guide')} "
                 f"err={at.get('err')}   galvo tick_us={gv.get('tick_us')}")
        if not self.link_ok:
            self.warn("ATmega link=0: arming and firing are impossible; the guide-laser steps still work "
                      "(check docs/WIRING.md, UART0 TX/RX and the level shifter)")
        self.say("  current calibration values:")
        for n in (3, 100, 101, 110, 130, 131, 140, 141, 142, 143, 144, 201, 210, 211, 212, 213, 214, 215, 216, 220,
                  221, 224, 225):
            if n in self.dev:
                self.say(f"   ${n:<4} {cm.NAMES.get(n, ''):<40} {cm.fmt_val(self.get(n)):>12}")
        return {"link": at.get("link"), "armed": at.get("armed"), "tick_us": gv.get("tick_us")}

    # -- focus
    def focus_new_spot(self, spots: list, r: float, power: float, feed: float, laps: int) -> dict:
        """Next free spot on the card (the allocator keeps the circles apart), framed with the guide laser."""
        box = self.new_region(2 * r, 2 * r)
        c = (box.cx, box.cy)
        spot = {"spot": len(spots) + 1, "x": round(c[0], 3), "y": round(c[1], 3), "height": None, "heights": [],
                "bursts": 0, "power": power, "radius": r, "laps": laps, "feed": feed}
        spots.append(spot)
        self.last_pattern = pat_focus_burst(c, r, laps, feed, power)
        self.say(f"  spot {spot['spot']}: circle R {r:g} mm at X {c[0]:.1f} Y {c[1]:.1f} "
                 f"(box X {c[0] - r:.1f}..{c[0] + r:.1f} Y {c[1] - r:.1f}..{c[1] + r:.1f}); framing it with the guide laser")
        self.draw_guide(self.frame_pattern(Box(c[0] - r, c[1] - r, 2 * r, 2 * r), c))
        if not self.a.yes and not self.ask_yes("frame_ok", "  Frame is on the test card?", False):
            raise SafetyError("operator rejected the frame")
        return spot

    def focus_burst(self, spot: dict) -> None:
        pat = pat_focus_burst((spot["x"], spot["y"]), spot["radius"], spot["laps"], spot["feed"], spot["power"])
        self.last_pattern = pat
        lines = self.encode(pat.ops, pat.centre)                   # S capped by the encoder
        est = self.est_seconds(pat.ops, spot["feed"])
        self.say(f"  FIRING {spot['laps']} laps of the circle at spot {spot['spot']} (S{spot['power']:g}, "
                 f"F{spot['feed']:g}, ~{est:.1f} s)")
        self.fired = True
        self.stream(lines, est)
        self.c.command("M5")
        spot["bursts"] += 1

    def focus_prompt(self, spot: dict) -> str:
        """'again' (mark again, same spot), 'next' or 'done'; 'h <value>' records the height and asks again."""
        for _ in range(MAX_RETRY):
            raw = self.ask(Question("focus_next", "[Enter] mark again at the same spot (adjust height first) / "
                                    "n = next spot / h <value> = record the height you just used (e.g. 'h 162.5') / done",
                                    "text", meta={"spot": spot})).strip()
            low = raw.lower()
            if low == "":
                return "again"
            if low in ("n", "next"):
                return "next"
            if low in ("done", "d"):
                return "done"
            m = re.match(r"h\s*[:=]?\s*(.*)$", low)
            if m and m.group(1).strip():
                label = m.group(1).strip().replace(",", ".")
                try:
                    label = cm.fmt_val(cm.parse_floats(label)[0])
                except CalibError:
                    pass                                          # free text such as "162.5 mm" is kept as typed
                spot["heights"].append(label)
                spot["height"] = label
                self.say(f"  recorded height {label} for spot {spot['spot']}")
                continue
            self.say("  answer Enter, n, 'h <value>' or done")
        raise CalibError("no valid answer for focus_next")

    def step_focus(self) -> dict:
        self.header("focus: height / focal distance (real laser)")
        self.say("Goal: find the Z height of the laser head at which the mark is sharpest. Circles are marked with the")
        self.say("real laser while YOU change the height between bursts; the wizard only keeps notes. The height is not a")
        self.say("firmware setting: the best value is recorded in the report and the state file.")
        self.say("Do this FIRST: scale, distortion and offset are only valid at the final working height.")
        self.ensure_card()
        cw = self.card_box[2] - self.card_box[0]
        ch = self.card_box[3] - self.card_box[1]
        rmax = min(cw, ch) / 2
        r = self.ask_nums("focus_radius", f"Circle radius in mm (0.5..{rmax:g})", default=min(self.a.focus_radius, rmax),
                          lo=0.5, hi=rmax)[0]
        if self.a.focus_power is not None:
            d = min(self.a.focus_power, self.cap)
            if d < self.a.focus_power:
                self.say(f"  --focus-power {self.a.focus_power:g} is above --max-power: capped to {self.cap:g}")
            power = self.ask_nums("focus_power", f"S for the focus circles (1..{self.cap:g})", default=d, lo=1,
                                  hi=self.cap)[0]
        else:
            self.ensure_mark_power()
            power = float(self.mark_power or 0.0)
        laps = int(self.ask_nums("focus_laps", "Laps per burst (the same circle is marked this many times)",
                                 default=FOCUS_LAPS, lo=1, hi=1000)[0])
        feed = self.ask_nums("focus_feed", "Marking speed in mm/min", default=min(self.a.feed, self.get(110)), lo=60,
                             hi=self.get(110))[0]
        spots: list[dict] = []
        res: dict[str, Any] = {"spots": spots, "radius": r, "power": power, "laps": laps, "feed": feed}
        self.results["focus"] = res
        self.fire_prelude()
        self.say("  NOTE: the laser stays ARMED (M10) between bursts, but nothing emits (M5). Keep hands and tools out of the")
        self.say("  beam path while you adjust the height; the step disarms (M11) when it ends (unless --keep-armed).")
        clean = False
        try:
            spot = self.focus_new_spot(spots, r, power, feed, laps)
            self.arm()
            self.focus_burst(spot)
            while True:
                self._redraw = lambda sp=spot: self.focus_burst(sp)           # 'again' == Enter
                act = self.focus_prompt(spot)
                if act == "done":
                    break
                if act == "next":
                    spot = self.focus_new_spot(spots, r, power, feed, laps)
                self.focus_burst(spot)
            clean = True
        except (SkipStep, QuitWizard):
            clean = True                       # no emergency follows these: disarm here
            raise
        finally:
            self._redraw = None
            # errors, SafetyError and Ctrl-C get the cancel sequence + M11 from run() (emergency)
            if clean and not self.a.keep_armed and self.armed:
                self.disarm()
        marked = [sp for sp in spots if sp["bursts"]]
        if marked:
            self.say("  Look at the circles with a loupe: the best focus is the THINNEST and brightest line, with the same")
            self.say("  width all the way around.")
            n = int(self.ask_nums("focus_best", f"Which spot looked best, 1..{len(spots)} (0 = none)", lo=0,
                                  hi=len(spots))[0])
            if n:
                sp = spots[n - 1]
                label = sp["height"]
                if not label:
                    label = self.ask(Question("focus_best_height", "Height you used for that spot (Enter = not recorded)",
                                              "text", meta={"spot": sp})).strip().replace(",", ".") or None
                self.focus_best = {"spot": n, "height": label, "x": sp["x"], "y": sp["y"], "power": sp["power"],
                                   "radius": sp["radius"], "laps": sp["laps"], "feed": sp["feed"]}
                res["best"] = self.focus_best
                self.say(f"  best: spot {n}, height {label if label else 'not recorded'}. Set the head to that height "
                         "before the next steps.")
                self.save_state()
        return res

    # -- range
    def range_volts_xy(self) -> tuple[float, float]:
        """Known usable half range (V) of the channels that drive physical X / Y under the CURRENT $143."""
        assert self.range_ch
        cx = cm.channel_for_axis("x", self.get(143))
        return self.range_ch[cx], self.range_ch[1 - cx]

    def range_workarea(self) -> Optional[tuple[float, float]]:
        """Suggested ($130, $131) from the known range volts and the current scale / offset (None = unknown)."""
        if not self.range_ch:
            return None
        swap = self.get(143)
        kx, ky = cm.scale_key_for_axis("x", swap), cm.scale_key_for_axis("y", swap)
        vx, vy = self.range_volts_xy()
        try:
            return cm.suggest_work_area(vx, vy, self.get(kx), self.get(ky), self.a.range_margin_v,
                                        self.get(kx + 40), self.get(ky + 40))
        except CalibError as e:
            self.warn(f"cannot suggest a work area: {e}")
            return None

    def offer_range_workarea(self, why: str) -> None:
        sug = self.range_workarea()
        if not sug:
            return
        W, H = sug
        if abs(W - self.get(130)) < 0.05 and abs(H - self.get(131)) < 0.05:
            return
        vx, vy = self.range_volts_xy()
        self.say(f"  Range (galvo limit +-{vx:g} V on X, +-{vy:g} V on Y, {self.range_source or 'this run'}): {why}, so the "
                 f"field in mm changed too. Suggested work area at the current scale: {W:g} x {H:g} mm "
                 f"(now {self.get(130):g} x {self.get(131):g}).")
        try:
            if self.ask_yes("range_reapply", "  Apply it as $130 / $131?", True):
                self.apply({130: W, 131: H}, "work area from the galvo range at the new scale", confirm=False)
                self.regions = None
        except SkipStep:
            pass

    def range_temp_values(self) -> dict[int, float]:
        """Work area big enough that the clamp in grbl.c never cuts the test rectangles (+-10 V plus the offset)."""
        swap = self.get(143)
        out = {}
        for axis, n in (("x", 130), ("y", 131)):
            k = cm.scale_key_for_axis(axis, swap)
            sc = abs(self.get(k))
            if sc < 1e-9:
                raise CalibError(f"${k} is zero: cannot convert volts to mm")
            need = 2.0 * (cm.FULL_SCALE_V + abs(self.get(k + 40))) / sc + 2.0
            out[n] = min(float(cm.RANGES[n][1]), math.ceil(need * 10) / 10)
        return out

    def range_restore(self, orig: dict[int, float], touched: set, err: Optional[BaseException]) -> bool:
        """Write the original $130 / $131 / $142 back and verify them against a `$$` readback."""
        if err is not None and not isinstance(err, (SkipStep, QuitWizard)):
            self.emergency()                   # stop the motion before touching settings
        ok = False
        try:
            self.write([(n, self.get(n), orig[n]) for n in sorted(touched)], "restore after the range test", quiet=True,
                       record=False)
            bad = [n for n in sorted(touched) if n in self.dev and not cm.same_value(self.dev[n], orig[n])]
            ok = not bad
            if ok:
                self.say("  restored " + ", ".join(f"${n}={cm.fmt_val(orig[n])}" for n in sorted(touched))
                         + " (verified by $$ readback)")
            else:
                self.warn("RESTORE MISMATCH: " + ", ".join(f"${n} reads {self.dev[n]:g}, wanted {cm.fmt_val(orig[n])}"
                                                           for n in bad))
        except Exception as e:                                       # noqa: BLE001
            self.warn(f"COULD NOT RESTORE the settings: {e}. Type by hand: "
                      + " ".join(f"${n}={cm.fmt_val(orig[n])}" for n in sorted(touched))
                      + f" (or --restore {self.out / 'settings_before.json'})")
        return ok

    def step_range(self) -> dict:
        self.header("range: largest correct galvo deflection (guide laser only)")
        self.say("Goal: the largest deflection the galvos display correctly, in DAC volts per axis (full scale is +-10 V),")
        self.say("independent of the mm scale. Rectangles are traced with the guide laser, never the marking laser.")
        self.say("Temporarily $142 = 0 (no radial correction) and the work area is enlarged so that nothing is clamped;")
        self.say("$130 / $131 / $142 are restored at the end, also after Ctrl-C.")
        self.say("X = the pair of sides that ends at the short tick drawn inside the rectangle (the +X side and its")
        self.say("opposite), Y = the other pair. Judge: complete straight edges and sharp corners; flattened or clipped")
        self.say("sides, a mirror buzzing or hitting its stop are failures.")
        swap = self.get(143)
        cx, cy = cm.channel_for_axis("x", swap), cm.channel_for_axis("y", swap)
        levels = tuple(self.a.range_levels)
        ladder = cm.RangeLadder(levels)
        orig = {n: self.get(n) for n in (130, 131, 142)}
        temp = self.range_temp_values()
        if orig[142] == 0.0 and not (142 in self.exact or 142 in self.dev_int):
            self.warn("$142 reads 0 but `$$` may hide a small k1 (fewer than 5 decimals): it is left untouched, so the "
                      "rectangles can be slightly distorted")
        res: dict[str, Any] = {"levels": list(levels), "history": ladder.history, "temporary": dict(temp),
                               "original": {str(n): v for n, v in orig.items()}}
        self.results["range"] = res
        touched: set = set()
        err: Optional[BaseException] = None
        try:
            rows = [(n, self.get(n), temp[n]) for n in (130, 131) if abs(self.get(n) - temp[n]) > 1e-9]
            if orig[142] != 0.0:
                rows.append((142, orig[142], 0.0))
            touched.update(n for n, _, _ in rows)
            if rows:
                self.say("  temporary: " + ", ".join(f"${n}={cm.fmt_val(new)}" for n, _, new in rows))
                self.write(rows, "temporary values for the range test", quiet=True, record=False)
            while not ladder.done:
                fx, fy = ladder.next_test()
                sq = cm.range_square(self.view(), fx, fy)
                pat = pat_range(sq, self.a.guide_feed)
                ax, ay = sq.actual
                lv = ladder.levels[ladder.i]
                self.say(f"  level {lv:g}: X sides +-{ax:.2f} V (asked {sq.target[0]:g}), Y sides +-{ay:.2f} V "
                         f"(asked {sq.target[1]:g}); corners " + " ".join(f"({a:+.2f},{b:+.2f})" for a, b in sq.corner_volts)
                         + " V")
                if sq.limited:
                    self.warn(f"level {lv:g} is limited by the work area ($130/$131 max 1000): only +-{ax:.2f} V / "
                              f"+-{ay:.2f} V are reached")
                self.draw(pat, False)
                raw = ""
                for _ in range(MAX_RETRY):
                    raw = self.ask(Question(
                        "range_ok", "Is the rectangle displayed correctly? ok = complete, straight edges / "
                        "x = the X sides (tick side + opposite) clipped, flattened or bent / y = the Y sides / "
                        "both / again = draw it again", "menu", meta={"level": lv, "fx": fx, "fy": fy})).strip().lower()
                    if raw in ("ok", "x", "y", "both"):
                        break
                    self.say("  answer ok, x, y, both or again")
                else:
                    raise CalibError("no valid answer for range_ok")
                ladder.answer(raw)
        except BaseException as e:                                   # noqa: BLE001
            err = e
            raise
        finally:
            restored = self.range_restore(orig, touched, err) if touched else True
            res["restored"] = restored
            if not restored and err is None:
                raise CalibError("the temporary settings could not be restored - see the warning above")
        # ---- result (axes are the commanded X / Y; the volts belong to the channel that drove them)
        fxg, fyg = ladder.good["x"], ladder.good["y"]
        vx, vy = round(fxg * cm.FULL_SCALE_V, 6), round(fyg * cm.FULL_SCALE_V, 6)
        res.update({"f_x": fxg, "f_y": fyg, "range_volts_x": vx, "range_volts_y": vy, "swap": swap,
                    "channel_x": cx, "channel_y": cy})
        self.say(f"  result: X good up to {fxg:g} of full scale = +-{vx:g} V (DAC channel {cx}), "
                 f"Y good up to {fyg:g} = +-{vy:g} V (DAC channel {cy})")
        for axis, f in (("X", fxg), ("Y", fyg)):
            if f >= 1.0 - 1e-9:
                self.warn(f"axis {axis} is displayed correctly at the full +-10 V: the DAC, not the galvo, limits the range")
            if f <= 0.0:
                self.warn(f"axis {axis} already failed at the lowest level {ladder.levels[0]:g}: check the galvo driver, "
                          "its input range and the wiring")
        if vx <= 0 or vy <= 0:
            res["suggested"] = None
            return res
        self.range_ch = [0.0, 0.0]
        self.range_ch[cx], self.range_ch[cy] = vx, vy
        self.range_swap = swap
        self.range_source = "this run"
        self.save_state()
        sug = self.range_workarea()
        if sug:
            W, H = sug
            res["suggested"] = {"W": W, "H": H, "margin_v": self.a.range_margin_v}
            self.say(f"  Suggested work area at the current scale (margin {self.a.range_margin_v:g} V): "
                     f"$130 = {W:g}, $131 = {H:g} mm (now {self.get(130):g} x {self.get(131):g}).")
            self.say("  It follows the scale: after 'scale' / 'distortion' the wizard offers the new values.")
            res["applied"] = False
            if self.ask_yes("range_apply", "  Apply it as $130 / $131?", False):
                res["applied"] = bool(self.apply({130: W, 131: H}, "work area from the galvo range", confirm=False))
                self.regions = None
        return res

    def step_field(self) -> dict:
        self.header("field: lens, origin and laser PRR")
        self.say("Enter the working field of the lens (e.g. 100 for an F-theta f=160 mm lens, 70, 150, 200).")
        self.say("Press Enter to keep the current value. Use the real, usable field - patterns are kept inside it.")
        g = self.geometry()
        dw, dh = g["W"], g["H"]
        sug = self.range_workarea()
        if sug:
            dw, dh = max(5.0, sug[0]), max(5.0, sug[1])
            self.say(f"  The 'range' step ({self.range_source or 'earlier'}) suggests {dw:g} x {dh:g} mm at the current "
                     f"scale (usable +-{self.range_volts_xy()[0]:g} V / +-{self.range_volts_xy()[1]:g} V minus the margin); "
                     "it is the default below. Run 'scale' afterwards: the field in mm follows the scale.")
        w = self.ask_nums("field_w", "Field width in mm", default=dw, lo=5, hi=1000)[0]
        h = self.ask_nums("field_h", "Field height in mm", default=dh, lo=5, hi=1000)[0]
        if w < 20 or h < 20:
            self.warn("a field below 20 mm leaves little room for the scale and distortion patterns")
        self.say("Origin: 0 = machine (0,0) is a corner of the work area (Rayforge default), "
                 "1 = (0,0) is the lens centre.")
        for _ in range(MAX_RETRY):
            o = self.ask(Question("origin", "Origin mode (0/1)", "text", cm.fmt_val(self.get(144)))).strip()
            if o in ("0", "1"):
                break
            self.say("  answer 0 or 1")
        else:
            raise CalibError("no valid origin mode")
        self.say("Laser PRR (pulse repetition rate, $220) and SYNC duty cycle ($221).")
        prr = self.ask_nums("prr_hz", "PRR in Hz (the laser's own range, e.g. 20000..200000 for a MOPA fibre laser)",
                            default=self.get(220), lo=0, hi=2_000_000)[0]
        if 0 < prr < 1000:
            self.warn(f"PRR {prr:g} Hz is very low for a fibre laser (typical 1 kHz .. 1 MHz)")
        duty = self.ask_nums("prr_duty", "SYNC duty cycle in % (1..99)", default=self.get(221), lo=1, hi=99)[0]
        ch = {130: w, 131: h, 144: float(o), 220: prr, 221: duty}
        applied = self.apply(ch, "field / origin / PRR")
        self.regions = None
        if applied:
            self.say("  Note: $100/$101 are volts per mm and do not depend on the field size, but run 'scale' again "
                     "after changing the lens.")
        return {"W": w, "H": h, "origin": int(float(o)), "prr": prr, "duty": duty, "applied": applied}

    # -- orientation
    def observe_arrow(self, axis: str, L: float, c: tuple[float, float]) -> cm.Direction:
        pat = pat_arrow(c, axis, L, self.a.guide_feed, 0.0)
        self.draw(pat, False)
        which = axis.upper()
        for _ in range(MAX_RETRY):
            raw = self.ask(Question(f"orient_{axis}",
                                    f"Which way does the +{which} arrow point when viewed from above, as you face the "
                                    "machine? [right/left/up/down] (up = away from you)", "direction",
                                    meta={"axis": axis}))
            try:
                return cm.parse_direction(raw)
            except CalibError as e:
                self.say(f"  {e}")
        raise CalibError("no valid direction")

    def step_orientation(self) -> dict:
        self.header("orientation: +X must go right, +Y must go away from you (guide laser)")
        self.say("Stand/sit at the front of the machine, looking down at the work surface. A red guide-laser arrow")
        self.say("is traced for a few seconds around the field centre; say where its HEAD (the pointed end) is.")
        g = self.geometry()
        c = g["centre"]
        L = max(10.0, min(40.0, 0.5 * min(g["W"], g["H"])))
        rounds = []
        for attempt in range(1, 5):
            sx = self.observe_arrow("x", L, c)
            sy = self.observe_arrow("y", L, c)
            seen = (cm.dir_name(sx), cm.dir_name(sy))
            rounds.append({"seen_x": seen[0], "seen_y": seen[1]})
            self.say(f"  you saw: +X -> {seen[0]}, +Y -> {seen[1]}")
            if sx == ("x", 1) and sy == ("y", 1):
                self.say("  orientation is correct")
                return {"rounds": rounds, "final": {"swap": self.get(143), "invert": int(self.get(3))}}
            try:
                swap, inv = cm.solve_orientation(self.get(143), int(self.get(3)), sx, sy, self.get(100), self.get(101))
            except CalibError as e:
                self.say(f"  ERROR: {e}")
                self.say("  looking again...")
                continue
            ok = self.apply({143: float(swap), 3: float(inv)}, "orientation: +X right, +Y away")
            if not ok:
                return {"rounds": rounds, "aborted": True}
            self.say("  re-drawing both arrows to confirm...")
        raise CalibError("orientation did not converge after 4 attempts - check the galvo wiring")

    # -- scale
    def instruct_measure(self, fire: bool, what: str) -> None:
        if fire:
            self.say(f"  Take the card out and measure {what} with calipers, to the CENTRES of the marked lines "
                     "(use a loupe); average both sides where asked.")
        else:
            self.say(f"  Guide laser on paper: mark the points with a pencil while the trace runs ('again' repeats it), "
                     f"then measure {what} with calipers or a ruler between the pencil marks.")

    def ask_len(self, key: str, prompt: str, nominal: float, n_max: int = 2) -> float:
        for _ in range(MAX_RETRY):
            vals = self.ask_nums(key, prompt, 1, n_max, lo=0.01)
            m = sum(vals) / len(vals)
            if not (0.25 * nominal <= m <= 4 * nominal):
                self.say(f"  {m:g} mm is very far from the expected {nominal:g} mm - typo? enter it again")
                continue
            return m
        raise CalibError("no valid length")

    def step_scale(self) -> dict:
        self.header("scale: $100 / $101 (volts per mm)")
        g = self.geometry()
        fire = self.use_fire("the test square")
        smax = 0.8 * min(g["W"], g["H"])
        self.say("A small square keeps lens distortion negligible; the distortion step handles the rest.")
        S = self.ask_nums("scale_side", "Square side in mm", default=min(20.0, smax), lo=5, hi=smax)[0]
        rounds = []
        for rnd in range(1, 6):
            feed, power = self.style(fire)
            c = self.geometry()["centre"]
            pat = pat_square_cross(c, S, feed, power)
            self.draw(pat, fire)
            self.instruct_measure(fire, f"the {S:g} mm square's X side (top and bottom) and Y side (left and right)")
            mx = self.ask_len("scale_x", "X side in mm (top and bottom, or one number)", S)
            my = self.ask_len("scale_y", "Y side in mm (left and right, or one number)", S)
            ex, ey = cm.rel_error(mx, S), cm.rel_error(my, S)
            rounds.append({"round": rnd, "measured_x": mx, "measured_y": my, "err_x_pct": ex * 100, "err_y_pct": ey * 100})
            self.say(f"  error: X {ex * 100:+.3f} %   Y {ey * 100:+.3f} %")
            if max(abs(ex), abs(ey)) < 0.001:
                self.say("  within 0.1 % - scale is good")
                break
            swap = self.get(143)
            kx, ky = cm.scale_key_for_axis("x", swap), cm.scale_key_for_axis("y", swap)
            ch = {kx: cm.scale_update(self.get(kx), S, mx), ky: cm.scale_update(self.get(ky), S, my)}
            if not self.apply(ch, "scale"):
                break
            if rnd == 5 or not self.ask_yes("scale_verify", "  Re-draw to verify?", True):
                break
        return {"rounds": rounds, "side": S, "fire": fire}

    # -- distortion
    def step_distortion(self) -> dict:
        self.header("distortion: $142 (k1) and the scales")
        g = self.geometry()
        fire = self.use_fire("the large square")
        rmax = 0.45 * min(g["W"], g["H"])
        R = self.ask_nums("dist_R", "Half side R in mm (square side = 2R)", default=round(0.4 * min(g["W"], g["H"]), 2),
                          lo=5, hi=rmax)[0]
        self.say(f"  square {2 * R:g} mm with a centre cross of total length {2 * R:g} mm on both axes; "
                 "run 'scale' first.")
        rounds = []
        for rnd in range(1, 5):
            feed, power = self.style(fire)
            c = self.geometry()["centre"]
            self.draw(pat_distortion(c, R, feed, power), fire)
            self.instruct_measure(fire, "the cross arms and the four square sides")
            nominal = 2 * R
            Lcx = self.ask_len("dist_cross_x", f"Cross X length in mm (nominal {nominal:g})", nominal, 1)
            Lcy = self.ask_len("dist_cross_y", f"Cross Y length in mm (nominal {nominal:g})", nominal, 1)
            Lsx = self.ask_len("dist_side_x", "Square top and bottom side lengths in mm (two numbers, or the average)", nominal)
            Lsy = self.ask_len("dist_side_y", "Square left and right side lengths in mm (two numbers, or the average)", nominal)
            bow = self.ask_menu("dist_bow", "How do the square's EDGES curve compared with a straight ruler?",
                                {"1": "bow inward / concave, corners stick out (pincushion)",
                                 "2": "bow outward / convex (barrel)", "3": "straight"}, extra="k = not sure")
            fit = cm.distortion_fit(R, Lcx, Lcy, Lsx, Lsy)
            rec = {"round": rnd, "Lcx": Lcx, "Lcy": Lcy, "Lsx": Lsx, "Lsy": Lsy, "b": fit.b, "a_x": fit.a_x,
                   "a_y": fit.a_y, "max_err_pct": fit.max_err * 100, "bow": bow}
            rounds.append(rec)
            self.say(f"  errors: cross X {fit.e_mx * 100:+.3f} % Y {fit.e_my * 100:+.3f} %   "
                     f"square X {fit.e_cx * 100:+.3f} % Y {fit.e_cy * 100:+.3f} %   "
                     f"fit e(r) = a + b r^2: b = {fit.b:.3e} /mm^2")
            ans = {"1": "inward", "2": "outward", "3": "straight"}.get(bow)
            if ans:
                cons = cm.bow_consistent(fit.b, R, ans)
                if cons is False:
                    self.warn(f"you saw edges {ans}, but the measured lengths imply the opposite curvature "
                              "(or none) - re-check which lengths went where before applying")
                elif cons is None:
                    self.say("  (distortion too small to judge the sign from the edge shape)")
            if fit.max_err < 0.0015:
                self.say("  all errors below 0.15 % - distortion is corrected")
                break
            swap = self.get(143)
            kx, ky = cm.scale_key_for_axis("x", swap), cm.scale_key_for_axis("y", swap)
            upd = cm.distortion_update(self.get(142), self.get(kx), self.get(ky), fit, DAMPING)
            for w_ in upd.warnings:
                self.warn(w_)
            ch = {142: upd.k1, kx: upd.scale_x, ky: upd.scale_y}
            if not self.apply(ch, f"distortion fit (damping {DAMPING:g})"):
                break
            if rnd == 4 or not self.ask_yes("dist_verify", "  Re-draw to verify?", True):
                break
        return {"rounds": rounds, "R": R, "fire": fire}

    # -- offset
    def step_offset(self) -> dict:
        self.header("offset: $140 / $141 (volts)")
        fire = self.use_fire("the crosshair")
        g = self.geometry()
        self.say(f"  The crosshair is drawn at the {'lens centre (0,0)' if self.get(144) else 'work-area centre'}.")
        self.say("  Measure where its centre lands relative to the point it SHOULD hit (a mark on the fixture, the lens "
                 "centreline, ...): dx = + when it is right of the reference, dy = + when it is further away from you.")
        rounds = []
        for rnd in range(1, 5):
            feed, power = self.style(fire)
            c = self.geometry()["centre"]
            self.draw(pat_crosshair(c, min(6.0, 0.1 * min(g["W"], g["H"])), feed, power), fire)
            dx, dy = self.ask_nums("offset_dxdy", "dx dy in mm (two numbers; 0 0 if it is dead on)", 2, 2, lo=-50, hi=50)
            rounds.append({"round": rnd, "dx": dx, "dy": dy})
            if max(abs(dx), abs(dy)) < 0.05:
                self.say("  within 0.05 mm - offset is good")
                break
            ch = cm.offset_update(self.view(), dx, dy)
            for n, v in ch.items():
                if abs(v) >= 9.99:
                    self.warn(f"${n} hit the +-10 V limit - the mirror mount is mechanically far off centre")
            if not self.apply(ch, "centre offset"):
                break
            if rnd == 4 or not self.ask_yes("offset_verify", "  Re-draw to verify?", True):
                break
        return {"rounds": rounds, "fire": fire}

    # -- delays
    def tune(self, title: str, items: list[dict], make: Callable[[Box], Pattern], size: tuple[float, float],
             feed: float, max_rounds: int = 10) -> dict:
        """Generic draw-ask-adjust loop.  item = {n, tuner, key, title, menu, dirs}."""
        hist = []
        label = title
        for rnd in range(1, max_rounds + 1):
            pat = make(self.new_region(*size))
            self.draw(pat, True)
            self.say("  Inspect the marks with a loupe.")
            ch: dict[int, float] = {}
            dirs: dict[int, int] = {}
            for it in items:
                t = it["tuner"]
                if t.done:
                    continue
                choice = self.ask_menu(it["key"], it["title"].format(v=cm.fmt_val(t.value)), it["menu"],
                                       meta={"param": it["n"], "value": t.value})
                dirs[it["n"]] = 0 if choice == "k" else it["dirs"][choice]
                new = t.update(dirs[it["n"]])
                if abs(new - self.get(it["n"])) > 1e-9:
                    ch[it["n"]] = new
            hist.append({"round": rnd, "directions": dirs, "new_values": dict(ch)})
            if ch and not self.apply(ch, label):
                break
            if all(it["tuner"].done for it in items):
                break
            if any(it["tuner"].at_limit for it in items):
                self.warn("a delay reached its limit - the cause is probably elsewhere (speed, scale, mechanics)")
                break
        else:
            self.say(f"  stopped after {max_rounds} rounds; current values kept")
        return {"rounds": hist}

    def step_delays(self) -> dict:
        self.header("delays: $210 .. $216 (real laser)")
        self.ensure_mark_power()
        feed = self.ask_nums("delay_feed", "Marking speed to tune at, in mm/min (the speed you will really use)",
                             default=self.a.feed, lo=60, hi=self.get(110))[0]
        self.a.feed = feed
        tick = self.tick_us()
        step0 = 20.0 if tick <= 10 else 2 * tick
        self.say(f"  tick = {tick:g} us: delays are rounded to whole ticks, so the finest useful step is {tick:g} us.")
        res: dict[str, Any] = {}
        pw = self.mark_power or 0.0

        def tuner(n: int, step: float = step0, min_step: float = tick) -> cm.DelayTuner:
            return cm.DelayTuner(f"${n}", self.get(n), step, min_step)

        on_off = [
            {"n": 210, "tuner": tuner(210), "key": "delay_start",
             "title": "Look at the START of each short mark ($210 laser-on delay, now {v} us):",
             "menu": {"1": "clean start", "2": "small dot / blob at the start (laser on too early) -> increase $210",
                      "3": "mark starts late, first part missing -> decrease $210"},
             "dirs": {"1": 0, "2": +1, "3": -1}},
            {"n": 211, "tuner": tuner(211), "key": "delay_end",
             "title": "Look at the END of each short mark ($211 laser-off delay, now {v} us):",
             "menu": {"1": "clean end", "2": "blob / tail at the end (laser off too late) -> decrease $211",
                      "3": "end is cut short / missing -> increase $211"},
             "dirs": {"1": 0, "2": -1, "3": +1}},
        ]
        res["on_off"] = self.tune("laser on/off delays ($210/$211)", on_off, lambda b: pat_grid(b, feed, pw), GRID_SIZE, feed)

        jump = [{"n": 212, "tuner": tuner(212, 2.5 * step0), "key": "delay_jump",
                 "title": "Look at the START of marks right after a jump ($212 jump delay, now {v} us):",
                 "menu": {"1": "clean, straight starts", "2": "wavy / hooked starts (galvo not settled) -> increase $212",
                          "3": "clean, and I want to find the smallest clean value -> decrease $212"},
                 "dirs": {"1": 0, "2": +1, "3": -1}}]
        res["jump"] = self.tune("jump delay ($212)", jump, lambda b: pat_jump(b, feed, pw), JUMP_SIZE, feed)

        per_mm = [{"n": 213, "tuner": cm.DelayTuner("$213", self.get(213), 2.0, 0.5, hi=1000), "key": "delay_jump_mm",
                   "title": "Rows are jumps of 2, 6, 12 and 24 mm (bottom to top). Do ONLY the long-jump rows still hook "
                            "($213 per-mm delay, now {v} us/mm)?",
                   "menu": {"1": "no, all rows look the same", "2": "yes, longer jumps hook more -> increase $213"},
                   "dirs": {"1": 0, "2": +1}}]
        res["jump_per_mm"] = self.tune("jump delay per mm ($213)", per_mm, lambda b: pat_jump(b, feed, pw), JUMP_SIZE, feed, 5)

        mark = [{"n": 214, "tuner": tuner(214), "key": "delay_mark",
                 "title": "Look at the END of marks just before a jump ($214 mark delay, now {v} us):",
                 "menu": {"1": "clean end", "2": "end smeared / hooked toward the next jump -> increase $214",
                          "3": "heavy dot at the end (dwell too long) -> decrease $214"},
                 "dirs": {"1": 0, "2": +1, "3": -1}}]
        res["mark"] = self.tune("mark delay ($214)", mark, lambda b: pat_grid(b, feed, pw), GRID_SIZE, feed)

        poly = [{"n": 215, "tuner": tuner(215), "key": "delay_poly",
                 "title": "Look at the corners of the V shapes ($215 polygon delay, now {v} us):",
                 "menu": {"1": "sharp, clean corners", "2": "rounded / cut corners -> increase $215",
                          "3": "burned spots at the corners -> decrease $215"},
                 "dirs": {"1": 0, "2": +1, "3": -1}}]
        res["polygon"] = self.tune("polygon delay ($215)", poly, lambda b: pat_corners(b, feed, pw), CORNER_SIZE, feed)
        res["final"] = {n: self.get(n) for n in (210, 211, 212, 213, 214, 215)}
        return res

    # -- power
    def step_power(self) -> dict:
        self.header("power: $224 / $225 (real laser)")
        self.ensure_mark_power()
        feed = self.ask_nums("power_feed", "Typical marking speed in mm/min", default=self.a.feed, lo=60,
                             hi=self.get(110))[0]
        powers = cm.ladder_values(self.cap, 9)
        smax = self.get(30)
        pmin, pmax = self.get(224), self.get(225)
        self.say("  patch  S     % power now")
        for i, s in enumerate(powers, 1):
            self.say(f"   {i:>2}   {s:<5} {cm.power_percent(s, smax, pmin, pmax):5.1f}")
        self.say("  Patch 1 is bottom-left (nearest you); numbers rise left to right, then the next row away from you.")
        pat = pat_power(self.new_region(*POWER_SIZE), feed, powers)
        self.draw(pat, True)
        n = len(powers)
        lo = int(self.ask_nums("power_lowest", f"Lowest patch that visibly marks (1..{n}, 0 = none)", default=None,
                               lo=0, hi=n)[0])
        if lo == 0:
            self.say("  nothing marked: raise --max-power, slow down (--feed) or check focus/PRR, then repeat")
            return {"powers": powers, "lowest": 0}
        sat = int(self.ask_nums("power_sat", f"Patch where the result stops getting stronger (lowest patch that "
                                f"looks saturated, {lo + 1}..{n}; 0 = never)", default=None, lo=0, hi=n)[0])
        if lo == 1:
            self.warn("the lowest patch already marks: the real threshold is lower - repeat with a smaller --max-power")
        if sat and sat <= lo:
            self.say("  saturation cannot be at or below the lowest marking patch; ignoring it")
            sat = 0
        try:
            ch = cm.power_map(powers[lo - 1], powers[sat - 1] if sat else None, smax, pmin, pmax)
        except CalibError as e:
            self.warn(str(e))
            return {"powers": powers, "lowest": lo, "saturated": sat, "error": str(e)}
        if not sat:
            self.say(f"  no saturation seen within S <= {self.cap:g}: only $224 is proposed")
        applied = self.apply(ch, "power mapping")
        p224, p225 = self.get(224), self.get(225)
        self.say(f"  Rayforge: keep the laser's max power at $30 = {smax:g}. With the new mapping S=1..{smax:g} spans "
                 f"{p224:g}% .. {p225:g}% of the ATmega power word; 1% .. 100% in Rayforge now covers "
                 "'just marks' to 'saturated'.")
        return {"powers": powers, "lowest": lo, "saturated": sat, "proposed": ch, "applied": applied}

    # -- speed
    def step_speed(self) -> dict:
        self.header("speed: $110 max marking speed, $201 jump speed")
        res: dict[str, Any] = {}
        fire = self.use_fire("the speed patches")
        if fire:
            self.ensure_mark_power()
            feeds = cm.speed_ladder_values(self.get(110))
            if len(feeds) < 2:
                self.warn("$110 is too low for a speed ladder")
            else:
                self.say("  patch  F (mm/min)   mm/s")
                for i, f in enumerate(feeds, 1):
                    self.say(f"   {i:>2}   {f:<11g} {f / 60:g}")
                self.say("  Patch 1 is bottom-left (nearest you); numbers rise left to right, then the next row away.")
                self.draw(pat_speed(self.new_region(*SPEED_SIZE), feeds, self.mark_power or 0.0), True)
                n = len(feeds)
                bad = int(self.ask_nums("speed_bad", f"First patch where corners/lines degrade (rounded corners, wavy or "
                                        f"broken lines), 1..{n}, 0 = none", lo=0, hi=n)[0])
                rec = cm.recommend_speed(feeds, bad)
                if rec is None:
                    self.warn("even the slowest patch is bad - fix scale/delays first, or lower the speeds")
                else:
                    res["marking"] = {"feeds": feeds, "first_bad": bad, "recommended": rec}
                    self.apply({110: rec}, "max marking speed ($111 is an alias)")
        else:
            self.say("  (marking-speed ladder needs the real laser: skipped)")
        res["jump"] = self.speed_jump(fire)
        return res

    def speed_jump(self, fire: bool) -> dict:
        trials = [1000.0, 2000.0, 3000.0, 5000.0, 8000.0, 12000.0]
        orig = self.get(201)
        last_clean: Optional[float] = None
        tried = []
        feed = self.a.feed
        self.say("  Jump speed test: marks separated by jumps of 2..24 mm are drawn with rising $201.")
        self.say(f"  $201 is changed temporarily and restored to {orig:g} unless you accept a new value.")
        try:
            for v in trials:
                self.write([(201, self.get(201), v)], f"temporary jump speed trial {v:g}", quiet=True)
                if fire:
                    pat = pat_jump(self.new_region(*JUMP_SIZE), feed, self.mark_power or 0.0)
                    self.draw(pat, True)
                    q = "Are all marks clean (no hooks, doubled or smeared starts)?"
                else:
                    g = self.geometry()
                    c = g["centre"]
                    pat = pat_jump(Box(c[0] - 14, c[1] - 7, JUMP_SIZE[0], JUMP_SIZE[1]), self.a.guide_feed, 0.0)
                    self.draw(pat, False)
                    q = "Is the red trace steady (no ringing or wobble after the jumps)?"
                ok = self.ask_yes("speed_jump_clean", f"  $201 = {v:g} mm/s: {q}", True, {"jump_speed": v})
                tried.append({"jump_speed": v, "clean": ok})
                if not ok:
                    break
                last_clean = v
        finally:
            self.write([(201, self.get(201), orig)], "restore jump speed", quiet=True) if abs(self.get(201) - orig) > 1e-9 else None
        if last_clean is None:
            self.warn("even the slowest jump speed looked bad - check $212 jump delay and the galvo tuning")
            return {"tried": tried}
        rec = last_clean
        self.apply({201: rec}, "jump speed (fastest clean trial)")
        return {"tried": tried, "recommended": rec}

    # -- summary
    def step_summary(self) -> dict:
        self.header("summary")
        if self.connected:
            try:
                self.read_settings()
            except Exception as e:                                   # noqa: BLE001
                self.log.note(f"summary: could not read $$: {e}")
        if not self.changes:
            self.say("  no settings were changed")
        else:
            self.say("  changes (old -> new):")
            last = {}
            for ch in self.changes:
                last[ch["setting"]] = ch
            for n, ch in sorted(last.items()):
                first = next(c for c in self.changes if c["setting"] == n)
                if abs(first["old"] - ch["new"]) < 1e-9:
                    continue
                self.say(f"   ${n:<4} {cm.NAMES.get(n, ''):<40} {cm.fmt_val(first['old']):>12} -> {cm.fmt_val(ch['new']):>12}"
                         f"   ({ch['step']})")
        self.say("  next steps: run  python3 tools/rftest/run_tests.py PORT fire-shapes --fire  to check real marks;")
        self.say("  re-run 'scale' and 'distortion' after any change of lens or working distance.")
        return {}

    # ---- report ------------------------------------------------------------
    def write_outputs(self, status: str) -> None:
        final = {str(n): v for n, v in sorted(self.dev.items())}
        exact = {str(n): self.get(n) for n in sorted(self.dev) if n in cm.WRITABLE}
        (self.out / "settings_after.json").write_text(json.dumps(
            {"time": dt.datetime.now().isoformat(timespec="seconds"), "settings": final, "exact": exact}, indent=1))
        md = ["# WaveMaster calibration report", "",
              f"- date: {dt.datetime.now().isoformat(timespec='seconds')}",
              f"- command: `calibrate.py {' '.join(sys.argv[1:])}`",
              f"- device: {self.device_info.get('version', '')} {self.device_info.get('options', '')}",
              f"- result: {status}", f"- S cap: {self.cap:g}   fire enabled: {self.a.fire}", ""]
        md += ["## Settings changed", ""]
        if self.changes:
            md += ["| step | setting | name | old | new | why |", "|---|---|---|---|---|---|"]
            for ch in self.changes:
                md.append(f"| {ch['step']} | ${ch['setting']} | {ch['name']} | {cm.fmt_val(ch['old'])} | "
                          f"{cm.fmt_val(ch['new'])} | {ch['why']} |")
        else:
            md.append("none")
        md += ["", "## Steps", ""]
        for name in STEP_ORDER:
            if name in self.results:
                md += [f"### {name}", "", "```json", json.dumps(self.results[name], indent=1, default=str), "```", ""]
        fb = self.focus_best
        if fb:
            md += ["## Focus", "", f"- best height: **{fb.get('height') or 'not recorded'}** (spot {fb.get('spot')}, "
                   f"S{fb.get('power'):g}, R {fb.get('radius'):g} mm, {fb.get('laps')} laps, F{fb.get('feed'):g}); "
                   "not a firmware setting - set the head to this height", ""]
        if self.range_ch:
            cx = cm.channel_for_axis("x", self.range_swap)
            md += ["## Range", "", f"- usable galvo range ({self.range_source or 'this run'}): "
                   f"X +-{self.range_ch[cx]:g} V, Y +-{self.range_ch[1 - cx]:g} V (DAC channel 0: +-{self.range_ch[0]:g} V, "
                   f"channel 1: +-{self.range_ch[1]:g} V)", ""]
        if self.notes:
            md += ["## Warnings", ""] + [f"- {n}" for n in self.notes] + [""]
        md += ["## Restore", "", f"`python3 tools/calibrate.py PORT --restore {self.out / 'settings_before.json'}`", "",
               "Note: `$$` prints only 4 decimals, so values such as `$142` below 5e-5 read as 0.0000. "
               "`settings_before.json` therefore also stores the exact values the wizard knew (`exact`).", ""]
        (self.out / "calibration_report.md").write_text("\n".join(md))

    # ---- safety ------------------------------------------------------------
    def emergency(self) -> None:
        if not self.connected:
            return
        self.say("EMERGENCY: cancel sequence (0x18, M5, M9) and M11")
        try:
            self.c.cancel()
            self.c.command("M11", timeout=3.0)
        except Exception as e:                                   # noqa: BLE001
            self.log.note(f"emergency cleanup failed: {e}")
        self.armed = False
        self.preview = False

    def finish_safe(self) -> None:
        for cmd in (["M67"] if self.preview else []) + ["M5"] + ([] if self.a.keep_armed else ["M11"]):
            try:
                self.c.command(cmd, timeout=3.0)
            except Exception as e:                               # noqa: BLE001
                self.log.note(f"cleanup {cmd} failed: {e}")
        if not self.a.keep_armed:
            self.armed = False

    def offer_restore(self) -> None:
        if not self.changes:
            return
        try:
            if self.ask_yes("restore_on_quit", "Restore the settings from the backup taken at the start?", False):
                self.restore_values(self.initial_exact or self.initial, confirm=False)
        except (QuitWizard, SkipStep, CalibError):
            pass

    def restore_values(self, target: dict[int, float], confirm: bool = True) -> bool:
        self.step = "restore"
        self.read_settings()
        rows = []
        for n, v in sorted(target.items()):
            if n not in cm.WRITABLE or n not in self.dev:
                continue
            # `$$` hides anything below 5e-5 (k1!), so only skip values that are provably equal:
            # known exactly, or integral (the dump prints integers without decimals).
            provable = (n in self.exact and abs(self.exact[n] - v) < 1e-9) or (n in self.dev_int and self.dev[n] == v
                                                                               and n not in self.exact)
            if not provable:
                rows.append((n, self.get(n), v))
        if not rows:
            self.say("  settings already equal the backup")
            return True
        self.say("  restoring (current -> backup):")
        self.table(rows)
        if confirm and not self.a.yes and not self.ask_yes("restore_confirm", "  Write these values?", False):
            self.say("  not restored")
            return False
        return self.write(rows, "restore")

    # ---- driver ------------------------------------------------------------
    def run(self, steps: Sequence[str], explicit: bool) -> str:
        status = "completed"
        try:
            self.say("At any prompt: a number (12.34 or 12,34), s = skip this step, q = quit (offers to restore the "
                     "backup), again = draw the pattern again.")
            self.ensure_connected()
            for name in STEP_ORDER:
                if name in steps and name != "summary":
                    self.run_step(name, explicit and name in self.a.step)
        except QuitWizard as e:
            status = f"quit ({e})"
            self.say("\nquitting")
        return status


def _retime(ops: list, feed: float, power: float) -> list:
    """Same geometry with another F / S (guide tracing uses S0 and the guide feed)."""
    out = []
    for op in ops:
        if isinstance(op, SetSpeed):
            out.append(SetSpeed(feed))
        elif isinstance(op, SetPower):
            out.append(SetPower(power))
        else:
            out.append(op)
    return out


# ------------------------------------------------------------------- main --

def parse_card(text: str) -> tuple[float, float]:
    try:
        a, b = text.lower().replace(",", "x").split("x")
        return float(a), float(b)
    except ValueError:
        raise argparse.ArgumentTypeError("--card expects WxH in mm, e.g. 60x60")


def parse_levels(text: str) -> tuple:
    try:
        return cm.parse_levels(text)
    except CalibError as e:
        raise argparse.ArgumentTypeError(str(e))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="WaveMaster interactive calibration wizard (see tools/CALIBRATION.md)",
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("port", nargs="?", help="serial port of the ESP32-S3 native USB (not needed with --mock)")
    p.add_argument("--mock", action="store_true", help="talk to tools/rftest/mock_grbl.py instead of hardware")
    p.add_argument("--step", action="append", choices=STEP_ORDER, help="run only this step (repeatable)")
    p.add_argument("--fire", action="store_true", help="allow steps that emit the real laser (focus, delays, power, "
                   "and real marks in scale/distortion/offset/speed)")
    p.add_argument("--yes", action="store_true", help="skip the typed FIRE and the frame confirmation")
    p.add_argument("--max-power", type=float, default=300.0, help="cap for every S value (default 300)")
    p.add_argument("--out", help="output directory (default tools/calibrate/runs/<timestamp>)")
    p.add_argument("--restore", metavar="FILE", help="write a backed-up settings file back and exit")
    p.add_argument("--keep-armed", action="store_true", help="stay armed between patterns and at the end")
    p.add_argument("--feed", type=float, default=12000.0, help="marking speed for fire patterns, mm/min (default 12000)")
    p.add_argument("--guide-feed", type=float, default=6000.0, help="guide-laser trace speed, mm/min (default 6000)")
    p.add_argument("--guide-seconds", type=float, default=3.0, help="how long a guide pattern is traced (default 3)")
    p.add_argument("--card", type=parse_card, default=(60.0, 60.0), help="test card size WxH in mm (default 60x60)")
    p.add_argument("--focus-radius", type=float, default=5.0, help="focus step: circle radius in mm (default 5)")
    p.add_argument("--focus-power", type=float, default=None,
                   help="focus step: S for the circles (default: the normal mark-power prompt); always capped at --max-power")
    p.add_argument("--range-levels", type=parse_levels, default=cm.RANGE_LEVELS,
                   help="range step: comma list of fractions of full scale +-10 V "
                        "(default 0.2,0.4,0.6,0.7,0.8,0.9,1.0)")
    p.add_argument("--range-margin-v", type=float, default=0.5,
                   help="range step: safety margin in volts kept off the usable range when suggesting $130/$131 (default 0.5)")
    p.add_argument("--arm-timeout", type=float, default=10.0)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--state", help="file remembering exact values between runs (default tools/calibrate/last_values.json)")
    p.add_argument("--mock-speedup", type=float, default=20.0)
    p.add_argument("--verbose", action="store_true", help="echo all serial traffic")
    return p


def load_backup(path: Path) -> dict[int, float]:
    """settings_before.json (prefers its exact values) or a raw `$$` text dump."""
    text = path.read_text()
    try:
        data = json.loads(text)
        src = data.get("exact") or data.get("settings") or {}
        return {int(k): float(v) for k, v in src.items()}
    except ValueError:
        return cm.parse_dump(text.splitlines())


def run(args: argparse.Namespace, provider: Any = None, mock: Any = None) -> int:
    if args.max_power < 0:
        print("--max-power must be >= 0", file=sys.stderr)
        return 2
    if args.range_margin_v < 0 or args.focus_radius <= 0:
        print("--range-margin-v must be >= 0 and --focus-radius > 0", file=sys.stderr)
        return 2
    explicit = bool(args.step)
    steps = list(args.step) if args.step else list(STEP_ORDER)
    if not args.restore and explicit and any(s in FIRE_STEPS for s in steps) and not args.fire:
        print("REFUSING: steps 'focus', 'delays' and 'power' emit the real laser. Re-run with --fire "
              "(and read tools/CALIBRATION.md).", file=sys.stderr)
        return 2
    out = Path(args.out) if args.out else HERE / "calibrate" / "runs" / dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    if args.mock and mock is None:
        from mock_grbl import MockGrbl
        mock = MockGrbl(speedup=args.mock_speedup)
        mock.start()
    port = mock.path if (args.mock and mock is not None) else args.port
    if not port:
        print("PORT is required (or use --mock)", file=sys.stderr)
        return 2
    logger = SessionLogger(out, verbose=args.verbose)
    try:
        client = RayforgeClient(port, logger, baud=args.baud).open()
    except Exception as e:                                       # noqa: BLE001
        print(f"cannot open {port}: {e}", file=sys.stderr)
        logger.close()
        return 1
    if not getattr(args, "state", None) and args.mock:
        args.state = str(out / "last_values.json")
    wiz = Wizard(args, client, logger, out, provider)
    code, status = 0, "completed"
    try:
        if args.restore:
            wiz.step = "restore"
            wiz.ensure_connected()
            target = load_backup(Path(args.restore))
            ok = wiz.restore_values(target)
            wiz.say("restore complete" if ok else "restore finished with mismatches")
            code = 0 if ok else 1
            status = "restore"
        else:
            status = wiz.run(steps, explicit)
    except KeyboardInterrupt:
        status, code = "interrupted by Ctrl-C", 130
        wiz.emergency()
    except SafetyError as e:
        status, code = f"safety stop: {e}", 2
        print(f"SAFETY STOP: {e}", file=sys.stderr)
        wiz.emergency()
    except Exception as e:                                       # noqa: BLE001
        status, code = f"error: {type(e).__name__}: {e}", 1
        print(f"ERROR: {type(e).__name__}: {e}", file=sys.stderr)
        wiz.emergency()
    try:
        if status.startswith("quit"):
            wiz.step = "summary"
            wiz.offer_restore()
        if wiz.connected and not args.restore:
            wiz.step = "summary"
            wiz.step_summary()
            wiz.finish_safe()
    except (KeyboardInterrupt, QuitWizard, SkipStep):
        pass
    except Exception as e:                                       # noqa: BLE001
        print(f"ERROR during cleanup: {e}", file=sys.stderr)
        code = code or 1
    finally:
        try:
            if wiz.connected:
                wiz.write_outputs(status)
                print(f"report: {out / 'calibration_report.md'}")
        except Exception as e:                                   # noqa: BLE001
            print(f"could not write report: {e}", file=sys.stderr)
        client.close()
        if mock is not None and args.mock:
            mock.stop()
        logger.close()
    return code


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
