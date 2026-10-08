#!/usr/bin/env python3
"""
Offline self-test for the calibration wizard (no hardware, no pytest).

    python3 tools/calibrate_selftest.py [-v] [--quick] [--keep]

Part 1  pure math (tools/calibrate_math.py): parsing, orientation solve for all 8 axis combinations
        (x 8 hidden galvo wirings x scale signs) round-tripped through a Python reimplementation of
        galvo_out.c, offset sign derivation, scale/distortion fits, delay bisection, card allocator.
Part 2  pattern builders stay inside the work area (both origin modes).
Part 3  scripted end-to-end runs of the real wizard against tools/rftest/mock_grbl.py.  A fake
        physical system (hidden true scale error, galvo wiring, F-theta distortion, mirror offset,
        laser/jump delay optima, power threshold, speed limits) computes what an operator would
        measure from the pattern the wizard just drew, using the mock's CURRENT $ settings, so
        convergence of the real loop is tested.
Exit status 0 = everything passed.
"""
from __future__ import annotations

import contextlib
import io
import json
import math
import random
import re
import shutil
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "rftest"))

import calibrate as cal                          # noqa: E402
import calibrate_math as cm                      # noqa: E402
from calibrate_math import CalibError            # noqa: E402
from mock_grbl import MockGrbl                   # noqa: E402

FAILS: list[str] = []
VERBOSE = False


def check(name: str, cond: bool, detail: str = "") -> bool:
    print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f"  [{detail}]" if detail and (not cond or VERBOSE) else ""))
    if not cond:
        FAILS.append(name)
    return cond


def raises(fn, exc=CalibError) -> bool:
    try:
        fn()
    except exc:
        return True
    return False


# --------------------------------------------------- hidden physical system

class Hidden:
    """The unknown machine: which axis each DAC channel moves (and which way), true volts per
    mm, F-theta distortion d (physical = pos (1 + d |pos|^2)) and the voltage of the reference
    mark (mirror offset)."""

    def __init__(self, axes=("x", "y"), signs=(1, 1), T=(0.1, 0.1), d=0.0, v0=(0.0, 0.0)):
        self.axes, self.signs, self.T, self.d, self.v0 = axes, signs, T, d, v0

    def phys(self, settings, x: float, y: float) -> tuple[float, float]:
        v = cm.galvo_transform(settings, x, y)
        pos = {"x": 0.0, "y": 0.0}
        for ch in (0, 1):
            pos[self.axes[ch]] += self.signs[ch] * (v[ch] - self.v0[ch]) / self.T[ch]
        f = 1.0 + self.d * (pos["x"] ** 2 + pos["y"] ** 2)
        return pos["x"] * f, pos["y"] * f


def classify(dx: float, dy: float) -> cm.Direction:
    if abs(dx) >= abs(dy):
        return ("x", 1 if dx > 0 else -1)
    return ("y", 1 if dy > 0 else -1)


def all_hidden(**kw) -> list[Hidden]:
    out = []
    for axes in (("x", "y"), ("y", "x")):
        for s0 in (1, -1):
            for s1 in (1, -1):
                out.append(Hidden(axes, (s0, s1), **kw))
    return out


def seen_dirs(h: Hidden, s: dict, centre=(50.0, 50.0), arm=10.0):
    cx, cy = centre
    p0, px, py = h.phys(s, cx, cy), h.phys(s, cx + arm, cy), h.phys(s, cx, cy + arm)
    return classify(px[0] - p0[0], px[1] - p0[1]), classify(py[0] - p0[0], py[1] - p0[1])


def base_settings(kw: dict | None = None) -> dict:
    s = {3: 0, 100: 0.1, 101: 0.1, 130: 100.0, 131: 100.0, 140: 0.0, 141: 0.0, 142: 0.0, 143: 0, 144: 0}
    s.update(kw or {})
    return s


# ------------------------------------------------------------------ part 1

def test_parsing() -> None:
    print("parsing and formatting")
    check("12.34", cm.parse_floats("12.34") == [12.34])
    check("12,34 (decimal comma)", cm.parse_floats("12,34") == [12.34])
    check("pair with decimal commas", cm.parse_floats("20,1 20,2", 1, 2) == [20.1, 20.2])
    check("pair with semicolon / slash", cm.parse_floats("20.1;20.2", 1, 2) == [20.1, 20.2]
          and cm.parse_floats("20.1 / 20.2", 1, 2) == [20.1, 20.2])
    check("negative and leading dot", cm.parse_floats("-.5 +2", 2, 2) == [-0.5, 2.0])
    check("garbage rejected", raises(lambda: cm.parse_floats("abc")) and raises(lambda: cm.parse_floats("1.2.3"))
          and raises(lambda: cm.parse_floats("")) and raises(lambda: cm.parse_floats("1 2 3", 1, 2)))
    check("count enforced", raises(lambda: cm.parse_floats("1", 2, 2)))
    check("fmt_val never uses an exponent", "e" not in cm.fmt_val(6.12e-6) and cm.fmt_val(6.12e-6) == "0.00000612")
    check("fmt_val integers / zero", cm.fmt_val(100000.0) == "100000" and cm.fmt_val(-0.0) == "0"
          and cm.fmt_val(0.0987654321) == "0.09876543")
    d = cm.parse_dump(["$3=0", "$100=0.1000", "$142=0.0000", "ok", "[MSG:x]", "$13=0", "garbage"])
    check("parse_dump", d == {3: 0.0, 100: 0.1, 142: 0.0, 13: 0.0}, str(d))
    check("same_value tolerates the 4-decimal dump", cm.same_value(0.0988, 0.098765) and not cm.same_value(0.0989, 0.098765)
          and cm.same_value(0.0, 6e-6))
    check("directions", cm.parse_direction("Right") == ("x", 1) and cm.parse_direction("away") == ("y", 1)
          and cm.parse_direction("down") == ("y", -1) and raises(lambda: cm.parse_direction("sideways")))
    check("range check", raises(lambda: cm.check_range(142, 1.5)) and raises(lambda: cm.check_range(13, 0))
          and cm.check_range(142, 0.2) is None)


def test_orientation() -> None:
    print("orientation solve (all 8 invert/swap combinations x 8 galvo wirings x scale signs)")
    n = bad = 0
    for h in all_hidden():
        for swap in (0, 1):
            for inv in range(4):
                for sgn0 in (1, -1):
                    for sgn1 in (1, -1):
                        s = base_settings()
                        s.update({143: swap, 3: inv, 100: 0.1 * sgn0, 101: 0.1 * sgn1})
                        sx, sy = seen_dirs(h, s)
                        if sx[0] == sy[0]:                 # impossible for a real galvo pair
                            continue
                        ns, ni = cm.solve_orientation(swap, inv, sx, sy, s[100], s[101])
                        s2 = dict(s)
                        s2.update({143: ns, 3: ni})
                        n += 1
                        if seen_dirs(h, s2) != (("x", 1), ("y", 1)):
                            bad += 1
    check(f"{n} round trips end with +X right, +Y away", bad == 0 and n == 8 * 8 * 4, f"bad={bad} n={n}")
    check("both arrows along one axis -> error", raises(lambda: cm.solve_orientation(0, 0, ("x", 1), ("x", -1))))
    # a few hand-derived cases
    check("already correct stays (0, 0)", cm.solve_orientation(0, 0, ("x", 1), ("y", 1)) == (0, 0))
    check("X went up, Y went right -> swap", cm.solve_orientation(0, 0, ("y", 1), ("x", 1)) == (1, 0))
    check("X left -> invert X", cm.solve_orientation(0, 0, ("x", -1), ("y", 1)) == (0, 1))
    check("Y down -> invert Y", cm.solve_orientation(0, 0, ("x", 1), ("y", -1)) == (0, 2))
    check("swap+invert: X went down, Y went left, from swap=1 inv=0 -> ch0 moves X backwards, ch1 moves Y backwards",
          cm.solve_orientation(1, 0, ("y", -1), ("x", -1)) == (0, 3))
    check("axis maps follow $143", cm.channel_for_axis("x", 0) == 0 and cm.channel_for_axis("x", 1) == 1
          and cm.scale_key_for_axis("y", 1) == 100 and cm.offset_key_for_axis("x", 1) == 141)


def test_offset() -> None:
    print("offset sign derivation (oriented machine, random scales/offsets/k1)")
    rng = random.Random(7)
    worst = 0.0
    n = 0
    for h in all_hidden(T=(0.1043, 0.0957), d=0.0, v0=(0.07, -0.05)):
        for swap in (0, 1):
            for inv in range(4):
                for sg in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
                    s = base_settings()
                    s.update({143: swap, 3: inv, 100: 0.1 * sg[0], 101: 0.1 * sg[1], 142: rng.uniform(-1e-5, 1e-5),
                              140: rng.uniform(-0.2, 0.2), 141: rng.uniform(-0.2, 0.2)})
                    sx, sy = seen_dirs(h, s)
                    if sx[0] == sy[0]:
                        continue
                    ns, ni = cm.solve_orientation(swap, inv, sx, sy, s[100], s[101])
                    s.update({143: ns, 3: ni})
                    # calibrate the scale magnitudes to the truth, keeping the sign the solve needs
                    ch_x, ch_y = cm.channel_for_axis("x", ns), cm.channel_for_axis("y", ns)
                    for k in (100, 101):
                        s[k] = math.copysign(h.T[k - 100], s[k])
                    centre = (50.0, 50.0)
                    px, py = h.phys(s, *centre)
                    upd = cm.offset_update(s, px, py)
                    s.update(upd)
                    qx, qy = h.phys(s, *centre)
                    worst = max(worst, abs(qx), abs(qy))
                    n += 1
                    assert ch_x != ch_y
    check(f"{n} cases: landing error after one update", worst < 1e-9 and n > 0, f"worst={worst:.2e} mm")
    s = base_settings()
    s.update({143: 1, 3: 2})
    up = cm.offset_update(s, 1.0, 2.0)
    check("explicit case swap=1 inv=Y: dx=1 -> $141 -= 1*0.1, dy=2 -> $140 += 2*0.1 (Y inverted)",
          abs(up[141] + 0.1) < 1e-12 and abs(up[140] - 0.2) < 1e-12, str(up))
    s = base_settings()
    s[140] = 9.95
    check("offset clamped to +-10 V", cm.offset_update(s, -5.0, 0.0)[140] == 10.0)


def test_scale_distortion() -> None:
    print("scale and distortion")
    check("scale_update", abs(cm.scale_update(0.1, 20.0, 20.4) - 0.1 * 20 / 20.4) < 1e-12)
    check("scale_update rejects <= 0", raises(lambda: cm.scale_update(0.1, 20.0, 0.0)))
    # exact model e(r) = a + b r^2
    R, a, b = 40.0, 0.004, 3.0e-6
    ex, ey = a + b * R * R, a + b * R * R
    ec = a + b * 2 * R * R
    fit = cm.distortion_fit(R, 2 * R * (1 + ex), 2 * R * (1 + ey), 2 * R * (1 + ec), 2 * R * (1 + ec))
    check("fit recovers a and b", abs(fit.b - b) < 1e-12 and abs(fit.a_x - a) < 1e-9, f"b={fit.b} a={fit.a_x}")
    up = cm.distortion_update(0.0, 0.1, 0.1, fit, damping=1.0)
    check("k1_new = k1_old - b, scale /= (1 + a)", abs(up.k1 + b) < 1e-12 and abs(up.scale_x - 0.1 / 1.004) < 1e-9)
    up2 = cm.distortion_update(1e-6, 0.1, 0.1, fit, damping=0.8)
    check("damping 0.8", abs(up2.k1 - (1e-6 - 0.8 * b)) < 1e-12)
    big = cm.DistortionFit(40, 0, 0, 0.5, 0.5, 5.0, 0, 0)
    u = cm.distortion_update(0.0, 0.1, 0.1, big)
    check("k1 clamped to [-1, 1] and warned", u.k1 == -1.0 and u.k1_clamped and u.warnings)
    u = cm.distortion_update(0.0, 0.1, 0.1, cm.DistortionFit(40, 0, 0, 0, 0, 1e-4, 0, 0), 1.0)
    check("warn when |k1 R^2| > 0.1", any("0.1" in w for w in u.warnings))
    check("bow sign check", cm.bow_consistent(3e-6, 40, "inward") is True and cm.bow_consistent(3e-6, 40, "outward") is False
          and cm.bow_consistent(-3e-6, 40, "outward") is True and cm.bow_consistent(1e-9, 40, "inward") is None)

    # closed loop through the transform reimplementation: converges in <= 4 rounds
    for d in (1.2e-5, -9e-6):
        h = Hidden(("y", "x"), (-1, 1), T=(0.1043, 0.0957), d=d)
        s = base_settings()
        s.update({143: 1, 3: 2, 100: 0.1, 101: 0.1})
        R = 40.0
        c = (50.0, 50.0)
        rounds = 0
        for rounds in range(1, 6):
            P = lambda x, y: h.phys(s, c[0] + x, c[1] + y)
            dist = lambda a_, b_: math.dist(a_, b_)
            Lcx, Lcy = dist(P(R, 0), P(-R, 0)), dist(P(0, R), P(0, -R))
            Lsx = (dist(P(R, R), P(-R, R)) + dist(P(R, -R), P(-R, -R))) / 2
            Lsy = (dist(P(R, R), P(R, -R)) + dist(P(-R, R), P(-R, -R))) / 2
            fit = cm.distortion_fit(R, Lcx, Lcy, Lsx, Lsy)
            if fit.max_err < 0.0015:
                break
            kx, ky = cm.scale_key_for_axis("x", s[143]), cm.scale_key_for_axis("y", s[143])
            u = cm.distortion_update(s[142], s[kx], s[ky], fit)
            s.update({142: u.k1, kx: u.scale_x, ky: u.scale_y})
        check(f"closed loop d={d:g}: converged in {rounds} rounds, k1 -> {-d:g}",
              rounds <= 4 and abs(s[142] + d) < 0.2 * abs(d), f"rounds={rounds} k1={s[142]:g} err={fit.max_err:.4%}")


def test_delays_misc() -> None:
    print("delay bisection, card allocator, ladders")
    for target, start, tol in ((150, 100, 15), (60, 120, 15), (40, 0, 15), (500, 300, 15), (0, 100, 15)):
        t = cm.DelayTuner("x", start, 20.0, 10.0)
        steps = 0
        while not t.done and steps < 20:
            direction = 0 if abs(t.value - target) <= tol else (1 if t.value < target else -1)
            t.update(direction)
            steps += 1
        check(f"tuner {start} -> {target} (+-{tol}) in {steps} steps", t.done and abs(t.value - target) <= tol and steps <= 12,
              f"value={t.value}")
    t = cm.DelayTuner("x", 100, 20.0, 5.0)
    t.update(1)
    t.update(-1)
    s1 = t.step
    t.update(1)
    t.update(-1)
    check("step halves on every direction flip, never below min_step", s1 == 10.0 and t.step == 5.0 and t.flips == 3)
    t = cm.DelayTuner("x", 10, 20.0, 10.0)
    t.update(-1)
    check("clamped at 0 and flagged", t.value == 0 and not t.at_limit)
    t.update(-1)
    check("stuck at the limit -> done", t.done and t.at_limit)

    al = cm.RegionAllocator((0, 0, 60, 60))
    regs = []
    while True:
        r = al.allocate(30, 18)
        if r is None:
            break
        regs.append(r)
    overlap = any(a[0] < b[0] + b[2] and b[0] < a[0] + a[2] and a[1] < b[1] + b[3] and b[1] < a[1] + a[3]
                  for i, a in enumerate(regs) for b in regs[i + 1:])
    inside = all(r[0] >= 0 and r[1] >= 0 and r[0] + r[2] <= 60 and r[1] + r[3] <= 60 for r in regs)
    check(f"allocator: {len(regs)} regions of 30x18 on 60x60, no overlap, all inside, then None",
          len(regs) == 3 and not overlap and inside and al.allocate(30, 18) is None)
    al.reset()
    check("allocator reset", al.allocate(30, 18) == (0, 0, 30, 18) and al.allocate(70, 5) is None)

    check("ladder values end at the cap, distinct", cm.ladder_values(300, 9)[-1] == 300 and len(set(cm.ladder_values(300, 9))) == 9
          and len(cm.ladder_values(5, 9)) < 9)
    check("power_percent", abs(cm.power_percent(500, 1000, 10, 60) - 35) < 1e-9 and cm.power_percent(5000, 1000, 0, 80) == 80)
    pm = cm.power_map(100, 400, 1000, 0, 100)
    check("power_map -> $224/$225", pm == {224: 10.0, 225: 40.0}, str(pm))
    check("power_map without saturation only sets $224", list(cm.power_map(100, None, 1000, 0, 100)) == [224])
    check("power_map errors", raises(lambda: cm.power_map(300, 200, 1000, 0, 100)) and raises(lambda: cm.power_map(0, None, 1000, 0, 100)))
    pm2 = cm.power_map(200, 600, 1000, 10, 60)          # old mapping 10..60 % -> thresholds in the old %
    check("power_map respects the old mapping", pm2 == {224: 20.0, 225: 40.0}, str(pm2))
    feeds = cm.speed_ladder_values(100000)
    check("speed ladder respects $110, recommend", feeds == [12000.0, 24000.0, 42000.0, 60000.0, 90000.0]
          and cm.recommend_speed(feeds, 3) == 24000.0 and cm.recommend_speed(feeds, 0) == 90000.0
          and cm.recommend_speed(feeds, 1) is None)
    check("work limits / centre", cm.work_limits({130: 80, 131: 60, 144: 1}) == (-40, 40, -30, 30)
          and cm.field_centre({130: 80, 131: 60, 144: 0}) == (40, 30))
    check("transform: corner origin centre -> 0 V", cm.galvo_transform(base_settings(), 50, 50) == (0.0, 0.0)
          and cm.galvo_transform(base_settings({144: 1}), 0, 0) == (0.0, 0.0))
    v = cm.galvo_transform({3: 1, 143: 1, 100: 0.2, 101: 0.1, 140: 0.5, 141: -0.5, 130: 100, 131: 100, 144: 0}, 60, 70)
    # x=(60-50)*-1=-10, y=20 -> swap -> x=20, y=-10 -> V0 = 20*0.2+0.5, V1 = -10*0.1-0.5
    check("transform: invert, then swap, then scale, then offset", abs(v[0] - 4.5) < 1e-9 and abs(v[1] + 1.5) < 1e-9, str(v))


# ------------------------------------------------------------------ part 2

def test_patterns() -> None:
    print("pattern builders stay inside the work area")
    for origin in (0, 1):
        for W, H in ((100.0, 100.0), (70.0, 50.0), (200.0, 150.0)):
            s = {130: W, 131: H, 144: origin}
            x0, x1, y0, y1 = cm.work_limits(s)
            c = cm.field_centre(s)
            m = min(W, H)
            pats = [cal.pat_arrow(c, "x", min(40, m / 2), 6000, 0), cal.pat_arrow(c, "y", min(40, m / 2), 6000, 0),
                    cal.pat_square_cross(c, min(20, 0.8 * m), 1000, 100),
                    cal.pat_distortion(c, 0.4 * m, 1000, 100), cal.pat_crosshair(c, min(6, 0.1 * m), 1000, 100)]
            card = (c[0] - min(30, m / 2 - 1), c[1] - min(30, m / 2 - 1), c[0] + min(30, m / 2 - 1), c[1] + min(30, m / 2 - 1))
            al = cm.RegionAllocator(card)
            for (w, h), fn in ((cal.GRID_SIZE, lambda b: cal.pat_grid(b, 1000, 100)),
                               (cal.JUMP_SIZE, lambda b: cal.pat_jump(b, 1000, 100)),
                               (cal.CORNER_SIZE, lambda b: cal.pat_corners(b, 1000, 100)),
                               (cal.POWER_SIZE, lambda b: cal.pat_power(b, 1000, cm.ladder_values(300))),
                               (cal.SPEED_SIZE, lambda b: cal.pat_speed(b, [1000.0] * 6, 100))):
                r = al.allocate(w, h)
                if r is not None:
                    pats.append(fn(cal.Box(*r)))
            ok = all(p.box.x >= x0 - 1e-6 and p.box.x1 <= x1 + 1e-6 and p.box.y >= y0 - 1e-6 and p.box.y1 <= y1 + 1e-6
                     for p in pats)
            check(f"origin {origin}, field {W:g}x{H:g}: {len(pats)} patterns inside", ok and len(pats) >= 5,
                  str([p.name for p in pats if not (p.box.x >= x0 and p.box.x1 <= x1 and p.box.y >= y0 and p.box.y1 <= y1)]))
    a = cal.pat_arrow((50, 50), "x", 40, 6000, 0)
    check("arrow +X: tip at +X, head barbs behind the tip", a.meta["tip"] == (70.0, 50.0) and a.meta["tail"] == (30.0, 50.0)
          and a.box.w == 40.0 and abs(a.box.h - 2 * 0.5 * 0.3 * 40) < 1e-9)
    ay = cal.pat_arrow((50, 50), "y", 40, 6000, 0)
    check("arrow +Y: tip at +Y", ay.meta["tip"] == (50.0, 70.0))


# ------------------------------------------------------------------ part 3

def fw_fmt(v: float) -> str:
    """calib.c fmt_value: integers plain, others %.4f."""
    return str(int(v)) if v == float(int(v)) else f"{v:.4f}"


class FwMock(MockGrbl):
    """mock_grbl with the firmware's 4-decimal `$$` format and a switchable ATmega link."""
    link = 1
    ever_armed = False
    ever_fired = False

    def _dollar(self, line, gen):
        if line.rstrip() == "$$":
            self._write("".join(f"${k}={fw_fmt(v)}\r\n" for k, v in sorted(self.settings.items())))
            return 0
        return super()._dollar(line, gen)

    def _mcode(self, m):
        if m == 10:
            self.ever_armed = True
        super()._mcode(m)

    def _stats_text(self):
        return super()._stats_text().replace("link=1", f"link={self.link}")

    def _move(self, mo, w, k, jog, gen):
        if mo in (1, 2, 3) and self.laser in (3, 4) and self.s > 0 and not self.preview:
            self.ever_fired = True
        return super()._move(mo, w, k, jog, gen)


class Bench:
    """Answers the wizard's questions like an operator measuring the Hidden machine."""

    def __init__(self, mock: FwMock, hidden: Hidden, quant: float = 0.01, true_delays=None, true_power=(12.0, 45.0),
                 max_clean_feed_mm_s: float = 1000.0, max_clean_jump: float = 5000.0, fire: bool = True,
                 inject: dict | None = None, raise_on: dict | None = None):
        self.mock, self.h, self.quant = mock, hidden, quant
        self.true = true_delays or {210: 150.0, 211: 60.0, 212: 500.0, 213: 3.0, 214: 140.0, 215: 40.0}
        self.tol = 15.0
        self.p_thr, self.p_sat = true_power
        self.max_feed, self.max_jump = max_clean_feed_mm_s, max_clean_jump
        self.fire = fire
        self.inject = inject or {}            # key -> list of canned answers served first
        self.raise_on = raise_on or {}        # key -> exception class
        self.log: list[tuple[str, str]] = []
        self.count = 0

    def S(self):
        return dict(self.mock.settings)

    def P(self, c, dx, dy):
        return self.h.phys(self.S(), c[0] + dx, c[1] + dy)

    def num(self, v: float) -> str:
        v = round(v / self.quant) * self.quant
        self.count += 1
        return (f"{v:.2f}".replace(".", ",")) if self.count % 3 == 0 else f"{v:.2f}"

    def ask(self, q: cal.Question) -> str:
        ans = self._answer(q)
        self.log.append((q.key, ans))
        return ans

    def _answer(self, q: cal.Question) -> str:
        k = q.key
        if k in self.raise_on:
            raise self.raise_on[k]()
        if self.inject.get(k):
            return self.inject[k].pop(0)
        pat = q.meta.get("pattern")
        S = self.S()
        if k.startswith("run_") or k.startswith("apply_") or k.endswith("_verify") or k in ("frame_ok", "restore_on_quit",
                                                                                           "restore_confirm"):
            return "y"
        if k == "fire":
            return "FIRE"
        if k.startswith("fire_mode_"):
            return "y" if self.fire else "n"
        if k == "mark_power":
            return "100"
        if k == "card_full":
            return ""
        if k in ("field_w", "field_h"):
            return "100"
        if k == "origin":
            return "0"
        if k == "prr_hz":
            return "40000"
        if k == "prr_duty":
            return "50"
        if k in ("scale_side", "dist_R", "delay_feed", "power_feed"):
            return ""
        if k.startswith("orient_"):
            p0, p1 = self.h.phys(S, *pat.meta["tail"]), self.h.phys(S, *pat.meta["tip"])
            return cm.dir_name(classify(p1[0] - p0[0], p1[1] - p0[1]))
        if k in ("scale_x", "scale_y"):
            c, h = pat.centre, pat.meta["half"]
            if k == "scale_x":
                a = math.dist(self.P(c, -h, h), self.P(c, h, h))
                b = math.dist(self.P(c, -h, -h), self.P(c, h, -h))
            else:
                a = math.dist(self.P(c, -h, -h), self.P(c, -h, h))
                b = math.dist(self.P(c, h, -h), self.P(c, h, h))
            return f"{self.num(a)} {self.num(b)}"
        if k.startswith("dist_"):
            c, R = pat.centre, pat.meta["R"]
            d = math.dist
            if k == "dist_cross_x":
                return self.num(d(self.P(c, -R, 0), self.P(c, R, 0)))
            if k == "dist_cross_y":
                return self.num(d(self.P(c, 0, -R), self.P(c, 0, R)))
            if k == "dist_side_x":
                return f"{self.num(d(self.P(c, -R, R), self.P(c, R, R)))} {self.num(d(self.P(c, -R, -R), self.P(c, R, -R)))}"
            if k == "dist_side_y":
                return f"{self.num(d(self.P(c, -R, -R), self.P(c, -R, R)))} {self.num(d(self.P(c, R, -R), self.P(c, R, R)))}"
            if k == "dist_bow":
                mid = d(self.P(c, -R, 0), self.P(c, R, 0)) / (2 * R)
                cor = d(self.P(c, -R, R), self.P(c, R, R)) / (2 * R)
                diff = cor - mid
                return "3" if abs(diff) < 1e-4 else ("1" if diff > 0 else "2")
        if k == "offset_dxdy":
            x, y = self.h.phys(S, *pat.centre)
            return f"{self.num(x)} {self.num(y)}"
        if k.startswith("delay_"):
            v, n = q.meta["value"], q.meta["param"]
            t, tol = self.true[n], self.tol
            if k == "delay_start":
                return "2" if v < t - tol else ("3" if v > t + tol else "1")
            if k == "delay_end":
                return "2" if v > t + tol else ("3" if v < t - tol else "1")
            if k == "delay_jump":
                return "2" if v < t - tol else ("3" if v > t + tol else "1")
            if k == "delay_jump_mm":
                return "2" if v < t - 0.5 else "1"
            if k == "delay_mark":
                return "2" if v < t - tol else ("3" if v > t + tol else "1")
            if k == "delay_poly":
                return "2" if v < t - tol else ("3" if v > t + tol else "1")
        if k == "power_lowest" or k == "power_sat":
            powers = pat.meta["powers"]
            pct = [cm.power_percent(s, S[30], S[224], S[225]) for s in powers]
            lo = next((i + 1 for i, p in enumerate(pct) if p >= self.p_thr), 0)
            if k == "power_lowest":
                return str(lo)
            return str(next((i + 1 for i, p in enumerate(pct) if p >= self.p_sat), 0))
        if k == "speed_bad":
            feeds = pat.meta["feeds"]
            return str(next((i + 1 for i, f in enumerate(feeds) if f / 60 > self.max_feed), 0))
        if k == "speed_jump_clean":
            return "y" if q.meta["jump_speed"] <= self.max_jump else "n"
        raise AssertionError(f"bench has no answer for {k!r}")


def run_wizard(tmp: Path, name: str, hidden: Hidden, argv: list[str], settings: dict | None = None,
               link: int = 1, **bench_kw) -> tuple[int, FwMock, Bench, Path, str]:
    out = tmp / name
    mock = FwMock(speedup=200.0)
    mock.link = link
    if settings:
        mock.settings.update(settings)
    mock.start()
    bench = Bench(mock, hidden, **bench_kw)
    args = cal.build_parser().parse_args(["--mock", "--out", str(out), "--state", str(tmp / f"{name}_state.json"),
                                          "--guide-seconds", "1"] + argv)
    buf = io.StringIO()
    errbuf = io.StringIO()
    t0 = time.monotonic()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(errbuf):
        code = cal.run(args, provider=bench, mock=mock)
    if VERBOSE:
        print(buf.getvalue())
        print(errbuf.getvalue())
    print(f"      ({name}: exit {code}, {time.monotonic() - t0:.1f} s)")
    return code, mock, bench, out, buf.getvalue() + errbuf.getvalue()


def s_values_in_log(out: Path) -> list[float]:
    vals = []
    for ln in (out / "events.jsonl").read_text().splitlines():
        ev = json.loads(ln)
        if ev.get("kind") == "tx_line":
            vals += [float(v) for v in re.findall(r"(?<![A-Za-z])S([-+]?\d*\.?\d+)", ev["line"])]
    return vals


def test_e2e_full(tmp: Path) -> Path:
    print("end to end: every step, fake galvo (swapped, Y inverted, +3 % scale error, pincushion, mirror offset)")
    T = (0.1043, 0.0957)
    d = 1.2e-5
    v0 = (0.03, -0.02)
    h = Hidden(("y", "x"), (-1, 1), T=T, d=d, v0=v0)
    code, mock, bench, out, text = run_wizard(tmp, "full", h, ["--fire", "--max-power", "600"])
    s = mock.settings
    check("exit code 0", code == 0, text[-400:])
    check("orientation: $143=1, $3=2 (derived by hand for this wiring)", s[143] == 1 and int(s[3]) == 2, f"{s[143]} {s[3]}")
    check("scale: X driven by ch1 -> $101 = 0.0957 within 0.3 %", abs(s[101] / T[1] - 1) < 0.003, f"{s[101]}")
    check("scale: Y driven by ch0 -> $100 = 0.1043 within 0.3 %", abs(s[100] / T[0] - 1) < 0.003, f"{s[100]}")
    check("distortion: k1 ~ -d within 20 %", abs(s[142] + d) < 0.2 * d, f"k1={s[142]:g} want {-d:g}")
    check("offset: $140/$141 = mirror offset volts (+-5 mV)", abs(s[140] - v0[0]) < 0.005 and abs(s[141] - v0[1]) < 0.005,
          f"{s[140]} {s[141]} want {v0}")
    check("field: PRR set", s[220] == 40000 and s[221] == 50)
    tr = bench.true
    check("delays within the bench's tolerance band", all(abs(s[n] - tr[n]) <= 15.5 for n in (210, 211, 212, 214, 215))
          and s[213] >= 2.5, str({n: s[n] for n in (210, 211, 212, 213, 214, 215)}))
    check("power: $224 at the visible threshold (13.3), $225 at saturation (46.7)",
          abs(s[224] - 13.3) < 0.2 and abs(s[225] - 46.7) < 0.2, f"{s[224]} {s[225]}")
    check("speed: $110 = last clean ladder feed (60000), $201 = fastest clean jump (5000)", s[110] == 60000 and s[201] == 5000,
          f"{s[110]} {s[201]}")
    # final machine: physical error at the pattern points
    c = (50.0, 50.0)
    R = 40.0
    err = max(abs(math.dist(h.phys(s, c[0] - R, c[1] - R), h.phys(s, c[0] + R, c[1] - R)) / (2 * R) - 1),
              abs(math.dist(h.phys(s, c[0] - R, c[1] - R), h.phys(s, c[0] - R, c[1] + R)) / (2 * R) - 1))
    check("final physical size error of the 80 mm square < 0.3 %", err < 0.003, f"{err:.4%}")
    check("laser disarmed at the end, M5 state", not mock.armed and mock.laser not in (3, 4) and not mock.guide)
    check("every S sent <= --max-power 600", max(s_values_in_log(out)) <= 600, str(max(s_values_in_log(out))))
    check("laser fired (fire steps ran) and no protocol violations", mock.ever_fired and not mock.violations,
          str(mock.violations[:3]))
    for f in ("session.log", "events.jsonl", "firmware_msgs.log", "statuses.csv", "settings_before.json",
              "settings_after.json", "calibration_report.md"):
        check(f"output file {f}", (out / f).is_file() and (out / f).stat().st_size > 0)
    rep = (out / "calibration_report.md").read_text()
    check("report lists the changed settings", "| orientation |" in rep and "$142" in rep and "## Restore" in rep)
    before = json.loads((out / "settings_before.json").read_text())
    check("backup holds the original $$", before["settings"]["100"] == 0.1 and before["settings"]["143"] == 0)
    check("exact k1 saved in the state file although $$ shows 0.0000",
          fw_fmt(s[142]) in ("0.0000", "-0.0000") and
          abs(json.loads((tmp / "full_state.json").read_text())["values"]["142"] - s[142]) < 1e-12)
    return out


def test_e2e_orientations(tmp: Path, quick: bool) -> None:
    print("end to end (guide laser only): orientation + scale + offset for several galvo wirings")
    cases = [("ident", Hidden(("x", "y"), (1, 1), T=(0.0933, 0.1072), v0=(0.05, 0.04))),
             ("flipboth", Hidden(("x", "y"), (-1, -1), T=(0.1, 0.1), v0=(-0.04, 0.02))),
             ("swapinv", Hidden(("y", "x"), (1, -1), T=(0.0905, 0.1105), v0=(0.0, -0.06)))]
    for name, h in (cases[:2] if quick else cases):
        code, mock, bench, out, text = run_wizard(tmp, name, h, ["--step", "orientation", "--step", "scale",
                                                                 "--step", "offset"])
        s = mock.settings
        ok_o = seen_dirs(h, s) == (("x", 1), ("y", 1))
        kx, ky = cm.scale_key_for_axis("x", s[143]), cm.scale_key_for_axis("y", s[143])
        chx, chy = cm.channel_for_axis("x", s[143]), cm.channel_for_axis("y", s[143])
        ok_s = abs(abs(s[kx]) / h.T[chx] - 1) < 0.002 and abs(abs(s[ky]) / h.T[chy] - 1) < 0.002
        ok_f = abs(s[140] - h.v0[0]) < 0.004 and abs(s[141] - h.v0[1]) < 0.004
        check(f"{name}: exit 0, +X right/+Y away, scales within 0.2 %, offsets within 4 mV, never fired",
              code == 0 and ok_o and ok_s and ok_f and not mock.ever_armed and not mock.ever_fired,
              f"code={code} orient={ok_o} scale={ok_s} off={ok_f} s140={s[140]} s141={s[141]} {text[-300:]}")


def test_e2e_safety(tmp: Path) -> None:
    print("end to end: safety, input handling, quit/restore")
    h = Hidden(("x", "y"), (1, 1), T=(0.1, 0.1))
    # 1. fire step without --fire is refused before anything is sent
    args = cal.build_parser().parse_args(["--mock", "--step", "delays", "--out", str(tmp / "refuse")])
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        code = cal.run(args, provider=None, mock=None)
    check("--step delays without --fire is refused (exit 2)", code == 2 and "REFUSING" in err.getvalue())

    # 2. full default run without --fire: fire steps skipped, never armed, guide steps work
    code, mock, bench, out, text = run_wizard(tmp, "nofire", Hidden(("x", "y"), (1, 1), T=(0.1, 0.1)), [])
    check("run without --fire: exit 0, 'delays'/'power' skipped, laser never armed or fired",
          code == 0 and "needs --fire" in text and not mock.ever_armed and not mock.ever_fired, text[-300:])

    # 3. wrong confirmation word
    code, mock, bench, out, text = run_wizard(tmp, "nofirecode", h, ["--fire", "--step", "delays"],
                                              inject={"fire": ["fire"]})
    check("typing anything but FIRE stops with a safety error, never armed", code == 2 and not mock.ever_armed
          and "SAFETY" in text, f"code={code}")

    # 4. link down: fire steps skipped, warning
    code, mock, bench, out, text = run_wizard(tmp, "nolink", h, ["--fire", "--step", "connect", "--step", "delays",
                                                                 "--step", "power"], link=0)
    check("ATmega link=0: warning, fire steps skipped, never armed", code == 0 and "link=0" in text and not mock.ever_armed
          and "link down" in text, text[-300:])

    # 5. Ctrl-C while the wizard waits during a fire step
    code, mock, bench, out, text = run_wizard(tmp, "ctrlc", h, ["--fire", "--yes", "--step", "delays"],
                                              raise_on={"delay_start": KeyboardInterrupt})
    log = (out / "session.log").read_text()
    check("Ctrl-C: exit 130, cancel sequence 0x18 + M5 + M9 + M11, disarmed",
          code == 130 and "<0x18>" in log and "TX  M9" in log and "TX  M11" in log and not mock.armed, f"code={code}")

    # 6. garbage answers are re-asked, comma decimals work
    code, mock, bench, out, text = run_wizard(
        tmp, "garbage", Hidden(("x", "y"), (1, 1), T=(0.0952, 0.1), v0=(0, 0)), ["--step", "scale"],
        inject={"scale_x": ["abc", "", "20.0.1", "0"], "scale_y": ["1 2 3"]})
    check("invalid/empty/out-of-range numbers are asked again; wizard converges",
          code == 0 and abs(mock.settings[100] / 0.0952 - 1) < 0.002 and text.count("invalid") >= 3,
          f"code={code} $100={mock.settings[100]} {text[-400:]}")

    # 7. both arrows along the same axis -> error message, asks again
    code, mock, bench, out, text = run_wizard(
        tmp, "sameaxis", Hidden(("x", "y"), (1, 1), T=(0.1, 0.1)), ["--step", "orientation"],
        inject={"orient_x": ["right"], "orient_y": ["left"]})
    check("same-axis answers produce an error and a second look", code == 0 and "same axis" in text, text[-300:])

    # 8. quit with q -> offer to restore the backup
    hq = Hidden(("y", "x"), (1, 1), T=(0.1, 0.1))
    code, mock, bench, out, text = run_wizard(tmp, "quit", hq, ["--step", "orientation", "--step", "scale"],
                                              inject={"scale_x": ["q"]})
    check("'q' quits, offers the restore, original settings are back", code == 0 and mock.settings[143] == 0
          and mock.settings[3] == 0 and ("restoring" in text) and "quit" in (out / "calibration_report.md").read_text(),
          f"$143={mock.settings[143]} $3={mock.settings[3]} {text[-300:]}")
    check("restore was asked once", [k for k, _ in bench.log].count("restore_on_quit") == 1)

    # 9. 's' skips a step
    code, mock, bench, out, text = run_wizard(tmp, "skip", h, ["--step", "scale"], inject={"scale_x": ["s"]})
    check("'s' skips the step without changes", code == 0 and mock.settings[100] == 0.1 and "skipped" in text)


def test_e2e_restore(tmp: Path, full_out: Path) -> None:
    print("end to end: --restore")
    before = full_out / "settings_before.json"
    after = json.loads((full_out / "settings_after.json").read_text())
    settings = {int(k): float(v) for k, v in after["exact"].items()}
    out = tmp / "restore"
    mock = FwMock(speedup=200.0)
    mock.settings.update(settings)
    mock.start()
    args = cal.build_parser().parse_args(["--mock", "--yes", "--out", str(out), "--state", str(tmp / "restore_state.json"),
                                          "--restore", str(before)])
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        code = cal.run(args, provider=None, mock=mock)
    s = mock.settings
    check("--restore writes the backed-up values back exactly (k1 back to 0, swap/invert back)",
          code == 0 and s[142] == 0 and s[143] == 0 and s[3] == 0 and s[100] == 0.1 and s[101] == 0.1 and s[110] == 300000
          and s[201] == 3000, f"code={code} " + str({n: s[n] for n in (3, 100, 101, 110, 142, 143, 201)}))
    # raw `$$` text file
    raw = tmp / "dump.txt"
    raw.write_text("$3=1\n$100=0.0900\nok\n")
    mock = FwMock(speedup=200.0)
    mock.start()
    args = cal.build_parser().parse_args(["--mock", "--yes", "--out", str(tmp / "restore2"), "--state",
                                          str(tmp / "restore_state2.json"), "--restore", str(raw)])
    with contextlib.redirect_stdout(io.StringIO()):
        code = cal.run(args, provider=None, mock=mock)
    check("--restore also reads a raw `$$` text dump", code == 0 and mock.settings[3] == 1 and mock.settings[100] == 0.09)


def main() -> int:
    global VERBOSE
    VERBOSE = "-v" in sys.argv
    quick = "--quick" in sys.argv
    keep = "--keep" in sys.argv
    tmp = Path(tempfile.mkdtemp(prefix="calibrate_selftest_"))
    t0 = time.monotonic()
    try:
        test_parsing()
        test_orientation()
        test_offset()
        test_scale_distortion()
        test_delays_misc()
        test_patterns()
        full_out = test_e2e_full(tmp)
        test_e2e_orientations(tmp, quick)
        test_e2e_safety(tmp)
        test_e2e_restore(tmp, full_out)
    finally:
        if keep:
            print(f"kept {tmp}")
        else:
            shutil.rmtree(tmp, ignore_errors=True)
    print(f"\n{'ALL PASSED' if not FAILS else str(len(FAILS)) + ' FAILED: ' + '; '.join(FAILS)}  ({time.monotonic() - t0:.1f} s)")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
