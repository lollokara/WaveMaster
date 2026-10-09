#!/usr/bin/env python3
"""
WaveMaster test runner - drives the controller exactly like Rayforge does.

    python3 tools/rftest/run_tests.py PORT PLAN [options]
    python3 tools/rftest/run_tests.py --mock PLAN [options]     # offline, no hardware

Plans
  guide plans (M66 ... M67 preview: guide laser on, marking laser forced off)
    guide-shapes   square, circle (polyline + arcs), star, spiral, target, corner test
    guide-frame    frame of the job box, looped --frame-loops times
    guide-raster   gradient and dither rasters with overscan (stream without firing)
    guide-stress   2000-segment circle + dense spiral: lines/s and underruns
    all-guide      the four guide plans in a row
  fire plans (REAL LASER - need --fire, typed FIRE, framing, arm)
    fire-power-ladder  hatched patches at --powers (capped by --max-power)
    fire-speed-ladder  one square at --feeds, fixed power
    fire-shapes        the shape set at one power
    fire-raster        dither + gradient + checkerboard
    fire-delay-grid    rows of short marks for $210/$211/$212 tuning
  smoke            connect, $I, $$, $S, $RB, tiny guide square, cancel test

Output (--out, default tools/rftest/runs/<timestamp>_<plan>/): job .gcode files as
streamed, session.log, events.jsonl, firmware_msgs.log, statuses.csv, report.md,
report.json.  See tools/rftest/README.md.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import re
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Optional, Sequence

import gcode_gen as gg
from gcode_gen import Box, Op
from rfclient import RayforgeClient, RfError, SessionLogger, StreamResult, strip_comment

HERE = Path(__file__).resolve().parent
GUIDE_PLANS = ("guide-shapes", "guide-frame", "guide-raster", "guide-stress")
FIRE_PLANS = ("fire-power-ladder", "fire-speed-ladder", "fire-shapes", "fire-raster", "fire-delay-grid")
ALL_PLANS = GUIDE_PLANS + ("all-guide",) + FIRE_PLANS + ("smoke",)
RASTER_PLANS = ("guide-raster", "fire-raster")
S_WORD = re.compile(r"(?<![A-Za-z])S([-+]?\d*\.?\d+)")


class SafetyError(RuntimeError):
    pass


# ----------------------------------------------------------------- args ----

def parse_pair(text: str, what: str, sep: str = ",") -> tuple[float, float]:
    try:
        a, b = text.lower().replace("x", sep).split(sep)
        return float(a), float(b)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{what}: expected two numbers, got {text!r}")


def parse_floats(text: str) -> list[float]:
    return [float(v) for v in text.split(",") if v.strip()]


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("items", nargs="+", metavar="PORT PLAN", help="serial port and plan (only PLAN with --mock)")
    p.add_argument("--mock", action="store_true", help="start mock_grbl in-process and use its pty")
    p.add_argument("--mock-speedup", type=float, default=20.0, help="mock: run simulated motion N times faster")
    p.add_argument("--mock-slow-ms", type=float, default=0.0, help="mock: extra delay per line")
    p.add_argument("--mock-error-on", help="mock: regex of lines answered with an error")
    p.add_argument("--mock-work-area", help="mock: device work area WxH ($130/$131, default: --work-area or 100x100)")
    p.add_argument("--mock-centered", action="store_true", help="mock: device with $144=1 (centred origin)")
    p.add_argument("--dialect", choices=gg.DIALECTS, help="default: grbl for vectors, grbl_raster for raster plans")
    p.add_argument("--origin", help="pattern origin x,y (lower-left corner) in machine mm "
                                    "(default: patterns centred on the work-area centre)")
    p.add_argument("--size", type=float, help="pattern size in mm (default min(50, 0.8 * min(W, H)))")
    p.add_argument("--feed", type=float, default=30000, help="marking speed mm/min (default 30000)")
    p.add_argument("--power", type=float, help="S value (default 100, always capped by --max-power)")
    p.add_argument("--max-power", type=float, default=300, help="hard cap for every S sent (default 300)")
    p.add_argument("--work-area", default="auto",
                   help="work area WxH in mm; 'auto' (default) reads $130/$131/$144 from the device "
                        "($$) after the handshake (100x100 for --dry-run). An explicit value must match the device.")
    p.add_argument("--centered", action="store_true",
                   help="with an explicit --work-area: the device uses a centred origin ($144=1, "
                        "coordinates -W/2..W/2)")
    p.add_argument("--powers", default="100,200,300,500,750,1000", help="fire-power-ladder S values")
    p.add_argument("--feeds", default="6000,15000,30000,60000,120000,180000", help="fire-speed-ladder F values")
    p.add_argument("--spacing", type=float, default=0.25, help="hatch spacing in mm (default 0.25)")
    p.add_argument("--frame-loops", type=int, default=3, help="guide-frame / fire framing loops (default 3)")
    p.add_argument("--no-arcs", action="store_true", help="linearise arcs instead of emitting G2/G3")
    p.add_argument("--modal-feed", action="store_true", help="grbl dialect: F only when it changes")
    p.add_argument("--poll-ms", type=float, default=250, help="'?' polling during jobs, 0 = off like Rayforge (default 250)")
    p.add_argument("--stall-timeout", type=float, default=30.0, help="seconds without ack before polling '?' (default 30)")
    p.add_argument("--idle-timeout", type=float, default=120.0, help="max wait for Idle after a job (default 120)")
    p.add_argument("--out", help="output dir (default tools/rftest/runs/<timestamp>_<plan>/)")
    p.add_argument("--dry-run", action="store_true", help="only write the .gcode files")
    p.add_argument("--repeat", type=int, default=1, help="run the plan N times")
    p.add_argument("--fire", action="store_true", help="REQUIRED for fire plans: really mark")
    p.add_argument("--yes", action="store_true", help="skip the FIRE / frame-OK prompts (does not skip framing)")
    p.add_argument("--keep-armed", action="store_true", help="do not send M11 at the end of a fire plan")
    p.add_argument("--arm-timeout", type=float, default=8.0, help="seconds to wait for armed=1 after M10")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--verbose", action="store_true", help="echo every TX/RX line to the console")
    return p


@dataclass
class Config:
    port: Optional[str]
    plan: str
    args: argparse.Namespace
    area: tuple[float, float]
    box: Box
    power: float
    cap: float
    powers: list[float]
    feeds: list[float]
    out: Path
    auto: bool = False             # work area still to be read from the device
    centered: bool = False         # $144=1: coordinates -W/2..W/2

    @property
    def is_fire(self) -> bool:
        return self.plan in FIRE_PLANS


class ConfigError(RuntimeError):
    """The requested pattern cannot be placed in the (now known) work area."""


def default_size(area: tuple[float, float]) -> float:
    return math.floor(min(50.0, 0.8 * min(area)) * 10 + 1e-9) / 10


def area_limits(area: tuple[float, float], centered: bool) -> tuple[float, float, float, float]:
    """Machine-coordinate limits x0, y0, x1, y1 of the work area."""
    if centered:
        return -area[0] / 2, -area[1] / 2, area[0] / 2, area[1] / 2
    return 0.0, 0.0, area[0], area[1]


def place_box(args: argparse.Namespace, area: tuple[float, float], centered: bool) -> Box:
    """Pattern box for a known work area; raises ConfigError with a suggested --size if it does not fit."""
    size = args.size if args.size is not None else default_size(area)
    x0, y0, x1, y1 = area_limits(area, centered)
    if args.origin is not None:
        ox, oy = parse_pair(args.origin, "--origin")
        how = f"--origin {ox:g},{oy:g}"
    else:
        ox, oy = (x0 + x1) / 2 - size / 2, (y0 + y1) / 2 - size / 2
        ox, oy = round(ox, 4), round(oy, 4)
        how = "centred"
    box = Box(ox, oy, size, size)
    if not box.inside_area(*area, centered=centered):
        if args.origin is None:
            fit = default_size(area)
        else:
            fit = math.floor(min(x1 - ox, y1 - oy) * 10) / 10 if ox >= x0 and oy >= y0 else 0.0
        hint = f"try --size {fit:g}" if fit >= 1 else "choose a smaller --size / another --origin"
        frame = "centred origin, X %g..%g Y %g..%g" % (x0, x1, y0, y1) if centered else "X 0..%g Y 0..%g" % (x1, y1)
        raise ConfigError(f"pattern box x={box.x:g} y={box.y:g} size {size:g} ({how}) does not fit the work area "
                          f"{area[0]:g}x{area[1]:g} ({frame}); {hint}")
    return box


def apply_geometry(cfg: "Config", area: tuple[float, float], centered: bool) -> None:
    cfg.box = place_box(cfg.args, area, centered)
    cfg.area, cfg.centered = area, centered


def make_config(argv: Optional[Sequence[str]] = None) -> Config:
    p = build_parser()
    a = p.parse_args(argv)
    plan = a.items[-1]
    if plan not in ALL_PLANS:
        p.error(f"unknown plan {plan!r}; choose from {', '.join(ALL_PLANS)}")
    port = a.items[0] if len(a.items) > 1 else None
    if not a.mock and not port and not a.dry_run:
        p.error("PORT is required (or use --mock)")
    auto = a.work_area.strip().lower() == "auto"
    if a.origin is not None:
        parse_pair(a.origin, "--origin")
    if a.size is not None and a.size <= 0:
        p.error("--size must be > 0")
    # auto: provisional 100x100 (dry-run, mock default); the real values come from the device later
    area = (100.0, 100.0) if auto else parse_pair(a.work_area, "--work-area", "x")
    centered = bool(a.centered) and not auto
    try:
        box = place_box(a, area, centered)
    except ConfigError as e:
        p.error(str(e))
    cap = a.max_power
    power = min(a.power if a.power is not None else 100.0, cap)
    powers: list[float] = []
    for v in parse_floats(a.powers):
        v = min(v, cap)
        if v not in powers:
            powers.append(v)
    out = Path(a.out) if a.out else HERE / "runs" / f"{dt.datetime.now():%Y%m%d-%H%M%S}_{plan}"
    return Config(port, plan, a, area, box, power, cap, powers, parse_floats(a.feeds), out, auto, centered)


# ---------------------------------------------------------------- jobs -----

@dataclass
class JobSpec:
    name: str
    ops: list[Op]
    dialect: str
    mode: str = "guide"            # guide (M66 preview) | fire | frame
    note: str = ""


@dataclass
class BuiltJob:
    spec: JobSpec
    index: int
    full_text: str                 # generated text incl. comments (what Rayforge would write)
    lines: list[str]               # comment-stripped lines exactly as streamed
    bbox: Optional[Box]
    clamped_coords: int
    clamped_power: int
    gcode_file: str = ""

    @property
    def name(self) -> str:
        return self.spec.name

    @property
    def s_values(self) -> list[float]:
        vals = {float(v) for ln in self.lines for v in S_WORD.findall(ln)}
        return sorted(v for v in vals if v > 0)

    @property
    def f_values(self) -> list[float]:
        return sorted({float(v) for ln in self.lines for v in re.findall(r"(?<![A-Za-z])F([\d.]+)", ln)})


def fmt_list(vals: Sequence[float], n: int = 8) -> str:
    if not vals:
        return "-"
    txt = ", ".join(f"{v:g}" for v in vals[:n])
    return txt + (f", ... ({len(vals)} values, max {max(vals):g})" if len(vals) > n else "")


def encode_job(cfg: Config, spec: JobSpec, index: int) -> BuiltJob:
    a = cfg.args
    code = gg.encode(spec.ops, spec.dialect, arcs=not a.no_arcs, modal_feed=a.modal_feed,
                     max_s=cfg.cap, work_area=cfg.area, centered=cfg.centered)
    lines = [s for s in (strip_comment(l) for l in code.lines) if s]
    assert_safe(lines, cfg.cap)
    return BuiltJob(spec, index, code.text, lines, gg.bbox(spec.ops), code.stats.clamped_coords,
                    code.stats.clamped_power)


def assert_safe(lines: Sequence[str], cap: float) -> None:
    """Last line of defence: never let an S above the cap reach the wire."""
    for ln in lines:
        for v in S_WORD.findall(ln):
            if float(v) > cap:
                raise SafetyError(f"refusing to send S{v} (> --max-power {cap:g}): {ln}")


def dialect_for(cfg: Config, plan: str) -> str:
    return cfg.args.dialect or ("grbl_raster" if plan in RASTER_PLANS else "grbl")


def shape_specs(cfg: Config, mode: str) -> list[JobSpec]:
    b, f, p = cfg.box, cfg.args.feed, cfg.power
    d = dialect_for(cfg, cfg.plan)
    items = [
        ("square", gg.square(b, f, p)),
        ("circle_poly", gg.circle_poly(b, f, p, 64)),
        ("circle_arcs", gg.circle_arc(b, f, p)),
        ("star", gg.star(b, f, p)),
        ("spiral", gg.spiral(b, f, p)),
        ("target", gg.target(b, f, p)),
        ("corner_test", gg.corner_test(b, f, p)),
    ]
    return [JobSpec(n, ops, d, mode) for n, ops in items]


def plan_guide_shapes(cfg: Config) -> list[JobSpec]:
    return shape_specs(cfg, "guide")


def plan_guide_frame(cfg: Config) -> list[JobSpec]:
    ops: list[Op] = []
    for k in range(cfg.args.frame_loops):
        ops += gg.frame_ops(cfg.box, cfg.args.feed, cfg.power) if k == 0 else \
            gg.polyline([(cfg.box.x, cfg.box.y), (cfg.box.x1, cfg.box.y), (cfg.box.x1, cfg.box.y1),
                         (cfg.box.x, cfg.box.y1)], close=True)
    return [JobSpec(f"frame_x{cfg.args.frame_loops}", ops, dialect_for(cfg, cfg.plan), "guide")]


def raster_specs(cfg: Config, mode: str, checker: bool) -> list[JobSpec]:
    b, f, p = cfg.box, cfg.args.feed, cfg.power
    d = dialect_for(cfg, cfg.plan)
    items = [("raster_gradient", gg.raster_gradient(b, f, p)), ("raster_dither", gg.raster_dither(b, f, p))]
    if checker:
        items.append(("raster_checkerboard", gg.checkerboard(b, f, p)))
    return [JobSpec(n, ops, d, mode) for n, ops in items]


def plan_guide_raster(cfg: Config) -> list[JobSpec]:
    return raster_specs(cfg, "guide", checker=False)


def plan_guide_stress(cfg: Config) -> list[JobSpec]:
    b, f, p = cfg.box, cfg.args.feed, cfg.power
    d = dialect_for(cfg, cfg.plan)
    return [JobSpec("circle_2000", gg.circle_poly(b, f, p, 2000), d, "guide"),
            JobSpec("spiral_dense", gg.spiral(b, f, p, turns=30, pts_per_turn=100), d, "guide")]


def capped_note(cfg: Config) -> str:
    asked = parse_floats(cfg.args.powers)
    over = [v for v in asked if v > cfg.cap]
    return f"  [capped: requested {', '.join(f'{v:g}' for v in asked)}; {len(over)} above --max-power {cfg.cap:g}]" if over else ""


def plan_fire_power_ladder(cfg: Config) -> list[JobSpec]:
    return [JobSpec("power_ladder", gg.power_ladder(cfg.box, cfg.args.feed, cfg.powers, cfg.args.spacing),
                    dialect_for(cfg, cfg.plan), "fire",
                    "patches left->right then up (+Y), S = " + ", ".join(f"{v:g}" for v in cfg.powers)
                    + capped_note(cfg))]


def plan_fire_speed_ladder(cfg: Config) -> list[JobSpec]:
    return [JobSpec("speed_ladder", gg.speed_ladder(cfg.box, cfg.feeds, cfg.power),
                    dialect_for(cfg, cfg.plan), "fire",
                    "patches left->right then up (+Y), F = " + ", ".join(f"{v:g}" for v in cfg.feeds))]


def plan_fire_shapes(cfg: Config) -> list[JobSpec]:
    return shape_specs(cfg, "fire")


def plan_fire_raster(cfg: Config) -> list[JobSpec]:
    return raster_specs(cfg, "fire", checker=True)


def plan_fire_delay_grid(cfg: Config) -> list[JobSpec]:
    return [JobSpec("delay_grid", gg.line_grid(cfg.box, cfg.args.feed, cfg.power),
                    dialect_for(cfg, cfg.plan), "fire", "rows of marks, lengths grow left->right")]


PLAN_BUILDERS: dict[str, Callable[[Config], list[JobSpec]]] = {
    "guide-shapes": plan_guide_shapes, "guide-frame": plan_guide_frame,
    "guide-raster": plan_guide_raster, "guide-stress": plan_guide_stress,
    "fire-power-ladder": plan_fire_power_ladder, "fire-speed-ladder": plan_fire_speed_ladder,
    "fire-shapes": plan_fire_shapes, "fire-raster": plan_fire_raster,
    "fire-delay-grid": plan_fire_delay_grid,
}


def smoke_square_spec(cfg: Config) -> JobSpec:
    b = Box(cfg.box.x, cfg.box.y, 5.0, 5.0)
    return JobSpec("tiny_square", gg.square(b, cfg.args.feed, cfg.power), dialect_for(cfg, "smoke"), "guide")


def cancel_job_lines(cfg: Config) -> list[str]:
    """700 slow zig-zag lines (F6000): more than the 512-segment queue and minutes of motion,
    so the stream is still blocked mid-job when the cancel arrives."""
    b = cfg.box
    ops: list[Op] = [gg.SetSpeed(6000), gg.SetPower(0), gg.MoveTo(b.x, b.y)]
    for k in range(700):
        ops.append(gg.LineTo(b.x1 if k % 2 == 0 else b.x, b.y + (k % 50) * b.h / 49))
    code = gg.encode(ops, "grbl", max_s=cfg.cap, work_area=cfg.area, centered=cfg.centered)
    return [s for s in (strip_comment(l) for l in code.lines) if s]


def build_jobs(cfg: Config) -> list[BuiltJob]:
    if cfg.plan == "smoke":
        specs = [smoke_square_spec(cfg)]
    elif cfg.plan == "all-guide":
        specs = [s for n in GUIDE_PLANS for s in PLAN_BUILDERS[n](Config(**{**cfg.__dict__, "plan": n}))]
    else:
        specs = PLAN_BUILDERS[cfg.plan](cfg)
    jobs: list[BuiltJob] = []
    for rep in range(max(1, cfg.args.repeat)):
        for s in specs:
            spec = s if cfg.args.repeat == 1 else JobSpec(f"{s.name}#{rep + 1}", s.ops, s.dialect, s.mode, s.note)
            jobs.append(encode_job(cfg, spec, len(jobs) + 1))
    return jobs


def write_job_files(cfg: Config, jobs: Sequence[BuiltJob]) -> None:
    cfg.out.mkdir(parents=True, exist_ok=True)
    for j in jobs:
        stem = f"{j.index:02d}_{re.sub(r'[^A-Za-z0-9_.#-]', '_', j.name)}"
        j.gcode_file = f"{stem}.gcode"
        (cfg.out / j.gcode_file).write_text("\n".join(j.lines) + "\n")
        (cfg.out / f"{stem}.rayforge.gcode").write_text(j.full_text)


# -------------------------------------------------------------- reports ----

@dataclass
class JobResult:
    name: str
    mode: str
    dialect: str
    gcode_file: str
    lines: int = 0
    lines_total: int = 0
    bytes: int = 0
    duration_s: float = 0.0
    lines_per_s: float = 0.0
    bytes_per_s: float = 0.0
    max_inflight: int = 0
    window: int = 0
    errors: list[str] = field(default_factory=list)
    aborted: bool = False
    abort_reason: str = ""
    stall_polls: int = 0
    ack_to_idle_s: Optional[float] = None
    underruns_delta: Optional[int] = None
    barriers_delta: Optional[int] = None
    chunks_delta: Optional[int] = None
    clamped_delta: Optional[int] = None
    atmega_link: Optional[str] = None
    guide_on_in_preview: Optional[bool] = None
    transitions: list[list] = field(default_factory=list)
    encoder_clamps: int = 0
    passed: bool = False
    reasons: list[str] = field(default_factory=list)


@dataclass
class Check:
    name: str
    passed: bool
    detail: str = ""


@dataclass
class Report:
    plan: str
    started: str
    port: str
    dialect: str
    command: str
    device: dict = field(default_factory=dict)
    settings: dict = field(default_factory=dict)
    jobs: list[JobResult] = field(default_factory=list)
    checks: list[Check] = field(default_factory=list)
    cancel: dict = field(default_factory=dict)
    stats_before: dict = field(default_factory=dict)
    stats_after: dict = field(default_factory=dict)
    mock_violations: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    passed: bool = False
    interrupted: bool = False

    def finish(self) -> None:
        self.passed = (all(j.passed for j in self.jobs) and all(c.passed for c in self.checks)
                       and not self.mock_violations and not self.interrupted
                       and bool(self.jobs or self.checks))


def sdelta(pre: dict, post: dict, kind: str, key: str) -> Optional[int]:
    try:
        return int(post[kind][key]) - int(pre[kind][key])
    except (KeyError, ValueError):
        return None


def transitions(logger: SessionLogger, t0: float, t1: float) -> list[list]:
    out: list[list] = []
    last = None
    for st in logger.statuses:
        if t0 <= st.t <= t1 and st.state != last:
            out.append([round(st.t, 3), st.state])
            last = st.state
    return out[:60]


def render_md(r: Report) -> str:
    L = [f"# WaveMaster test report: {r.plan}", "",
         f"- started: {r.started}", f"- port: `{r.port}`  dialect: `{r.dialect}`",
         f"- command: `{r.command}`",
         f"- firmware: `{r.device.get('version', '?')}` `{r.device.get('options', '')}` "
         f"machine `{r.device.get('machine', '')}` (RX window {r.device.get('rx_window', '?')})",
         f"- result: **{'PASS' if r.passed else 'FAIL'}**", ""]
    if r.jobs:
        L += ["## Jobs", "",
              "| job | mode | lines | bytes | s | lines/s | B/s | max in-flight | err | ack->Idle s | underruns | barriers | result |",
              "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|"]
        for j in r.jobs:
            idle = "-" if j.ack_to_idle_s is None else f"{j.ack_to_idle_s:.2f}"
            L.append(f"| {j.name} | {j.mode} | {j.lines} | {j.bytes} | {j.duration_s:.2f} | {j.lines_per_s:.0f} | "
                     f"{j.bytes_per_s:.0f} | {j.max_inflight}/{j.window} | {len(j.errors)} | {idle} | "
                     f"{j.underruns_delta} | {j.barriers_delta} | {'PASS' if j.passed else 'FAIL'} |")
        L.append("")
        for j in r.jobs:
            if j.reasons or j.errors:
                L += [f"### {j.name}", ""] + [f"- {x}" for x in j.reasons + j.errors] + [""]
    if r.checks:
        L += ["## Checks", ""] + [f"- {'PASS' if c.passed else 'FAIL'} {c.name}: {c.detail}" for c in r.checks] + [""]
    if r.cancel:
        L += ["## Cancel", ""] + [f"- {k}: {v}" for k, v in r.cancel.items()] + [""]
    L += ["## $S before / after", "", "```", json.dumps(r.stats_before, indent=1), json.dumps(r.stats_after, indent=1), "```", ""]
    if r.mock_violations:
        L += ["## Mock protocol violations", ""] + [f"- {v}" for v in r.mock_violations] + [""]
    if r.notes:
        L += ["## Notes", ""] + [f"- {n}" for n in r.notes] + [""]
    L += ["Files: `session.log` (transcript), `events.jsonl`, `firmware_msgs.log`, `statuses.csv`, job `.gcode` files."]
    return "\n".join(L) + "\n"


def print_table(r: Report) -> None:
    print()
    if r.jobs:
        hdr = f"{'job':<24}{'mode':<6}{'lines':>7}{'bytes':>8}{'sec':>8}{'l/s':>8}{'maxIF':>7}{'err':>4}{'->Idle':>8}{'urun':>5}  result"
        print(hdr)
        print("-" * len(hdr))
        for j in r.jobs:
            idle = "-" if j.ack_to_idle_s is None else f"{j.ack_to_idle_s:.2f}"
            print(f"{j.name[:23]:<24}{j.mode:<6}{j.lines:>7}{j.bytes:>8}{j.duration_s:>8.2f}{j.lines_per_s:>8.0f}"
                  f"{j.max_inflight:>7}{len(j.errors):>4}{idle:>8}{str(j.underruns_delta):>5}  "
                  f"{'PASS' if j.passed else 'FAIL: ' + '; '.join(j.reasons)}")
    for c in r.checks:
        if not c.passed:
            print(f"FAIL  {c.name}: {c.detail}")
    for v in r.mock_violations:
        print("MOCK VIOLATION:", v)
    print(f"\nRESULT: {'PASS' if r.passed else 'FAIL'}")


# -------------------------------------------------------------- session ----

class Session:
    def __init__(self, cfg: Config, client: RayforgeClient, logger: SessionLogger, report: Report,
                 input_fn: Callable[[str], str] = input):
        self.cfg, self.c, self.log, self.rep, self.input = cfg, client, logger, report, input_fn
        self.window = client.rx_window

    # ---- generic helpers
    def say(self, text: str) -> None:
        print(text, flush=True)
        self.log.note(text)

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        self.rep.checks.append(Check(name, ok, detail))
        self.say(f"  [{'ok' if ok else 'FAIL'}] {name}: {detail}")
        return ok

    def preview_on(self) -> bool:
        self.c.command_ok("M66")
        # M66 is acknowledged before the ATmega has switched the guide laser
        # (the link is asynchronous, ~1 ms round trip), so poll briefly.
        deadline = time.monotonic() + 1.0
        while True:
            guide = self.c.stats_s().get("atmega", {}).get("guide")
            if guide == "1" or time.monotonic() >= deadline:
                return guide == "1"
            time.sleep(0.05)

    def preview_off(self) -> None:
        self.c.command("M67")

    # ---- one job
    def run_job(self, job: BuiltJob) -> JobResult:
        cfg, c = self.cfg, self.c
        jr = JobResult(job.name, job.spec.mode, job.spec.dialect, job.gcode_file, window=self.window,
                       lines_total=len(job.lines), encoder_clamps=job.clamped_coords)
        pre = c.stats_s()
        preview = job.spec.mode in ("guide", "frame")
        if preview:
            jr.guide_on_in_preview = self.preview_on()
        c.poll_ms = cfg.args.poll_ms
        res = c.stream(job.lines)
        self.fill_stream(jr, res)
        t_end = self.log.now()
        st, _ = c.wait_idle(cfg.args.idle_timeout) if not res.aborted else (None, 0.0)
        if st is not None and res.last_ack_t is not None:
            jr.ack_to_idle_s = max(0.0, self.log.now() - res.last_ack_t)
        if preview and not res.aborted:
            self.preview_off()
        c.command("M5")
        post = c.stats_s()
        jr.transitions = transitions(self.log, res.t_start, self.log.now())
        jr.underruns_delta = sdelta(pre, post, "galvo", "underruns")
        jr.barriers_delta = sdelta(pre, post, "galvo", "barriers")
        jr.chunks_delta = sdelta(pre, post, "galvo", "chunks")
        jr.clamped_delta = sdelta(pre, post, "host", "clamped")
        jr.atmega_link = post.get("atmega", {}).get("link")
        self.rep.stats_after = post
        self.judge(jr, res, st is not None, job)
        self.say(f"  {'PASS' if jr.passed else 'FAIL'} {job.name}: {jr.lines} lines {jr.bytes} B in "
                 f"{jr.duration_s:.2f}s ({jr.lines_per_s:.0f} l/s)" + ("" if jr.passed else " - " + "; ".join(jr.reasons)))
        return jr

    @staticmethod
    def fill_stream(jr: JobResult, res: StreamResult) -> None:
        jr.lines, jr.bytes, jr.duration_s = res.lines_sent, res.bytes_sent, res.duration
        jr.lines_per_s, jr.bytes_per_s, jr.max_inflight = res.lines_per_s, res.bytes_per_s, res.max_inflight
        jr.errors = [f"line {e.index}: {e.reply} <- {e.line}" for e in res.errors]
        jr.aborted, jr.abort_reason, jr.stall_polls = res.aborted, res.abort_reason, res.stall_polls

    def judge(self, jr: JobResult, res: StreamResult, idle: bool, job: BuiltJob) -> None:
        why = jr.reasons
        if jr.errors:
            why.append(f"{len(jr.errors)} line error(s)")
        if jr.aborted:
            why.append(f"aborted: {jr.abort_reason}")
        if res.acks != res.lines_sent or res.lines_sent != res.lines_total:
            why.append(f"acks {res.acks} / sent {res.lines_sent} / total {res.lines_total}")
        if not idle and not jr.aborted:
            why.append(f"no Idle within {self.cfg.args.idle_timeout:g} s after the last ack")
        if jr.underruns_delta:
            why.append(f"underruns +{jr.underruns_delta}")
        if jr.clamped_delta:
            why.append(f"firmware clamped {jr.clamped_delta} target(s): check $130/$131/$144 vs --work-area")
        if job.clamped_coords or job.clamped_power:
            why.append(f"generator clamped {job.clamped_coords} coords / {job.clamped_power} powers")
        if jr.atmega_link not in (None, "1"):
            why.append(f"ATmega link={jr.atmega_link}")
        if jr.guide_on_in_preview is False:
            why.append("guide laser not on after M66")
        if jr.max_inflight > self.window:
            why.append(f"in-flight {jr.max_inflight} > window {self.window}")
        jr.passed = not why

    # ---- plans
    def run_jobs(self, jobs: Sequence[BuiltJob], stop_on_fail: bool) -> None:
        for j in jobs:
            self.say(f"job {j.index}/{len(jobs)} {j.name} [{j.spec.mode}, {j.spec.dialect}] {len(j.lines)} lines")
            jr = self.run_job(j)
            self.rep.jobs.append(jr)
            if not jr.passed and stop_on_fail:
                self.say("stopping the plan after a failed fire job")
                break

    def device_settings(self) -> dict[str, float]:
        out: dict[str, float] = {}
        for ln in self.c.command("$$", timeout=10.0):
            m = re.match(r"\$(\d+)=([-+0-9.eE]+)", ln)
            if m:
                out[m.group(1)] = float(m.group(2))
        return out

    def check_limits(self, settings: dict[str, float]) -> bool:
        w, h, o = settings.get("130"), settings.get("131"), settings.get("144", 0.0)
        cfg = self.cfg
        ok = (w, h) == cfg.area and bool(o) == cfg.centered
        what = f"--work-area {cfg.area[0]:g}x{cfg.area[1]:g}" + (" --centered" if cfg.centered else "")
        if cfg.auto:
            what = "work area adopted from the device"
        return self.check("work area matches device", ok,
                          f"device $130={w} $131={h} $144={o}, {what}"
                          + ("" if ok else " (omit --work-area to adopt the device's, or fix --work-area / --centered)"))

    # ---- fire safety
    def fire_summary(self, jobs: Sequence[BuiltJob]) -> str:
        union: Optional[Box] = None
        rows = []
        for j in jobs:
            if j.bbox:
                union = j.bbox if union is None else union.union(j.bbox)
            s, f = fmt_list(j.s_values), fmt_list(j.f_values)
            rows.append(f"  {j.name:<22} {len(j.lines):>6} lines  S[{s}]  F[{f}]  {j.spec.note}")
        bb = f"X {union.x:.2f}..{union.x1:.2f}  Y {union.y:.2f}..{union.y1:.2f}" if union else "-"
        return "\n".join(["", "*** FIRE PLAN - THE MARKING LASER WILL EMIT ***", f"plan: {self.cfg.plan}",
                          f"S cap (--max-power): {self.cfg.cap:g}   feed: {self.cfg.args.feed:g} mm/min",
                          f"bounding box (incl. raster overscan): {bb}", "jobs:"] + rows + [""])

    def fire_prelude(self, jobs: Sequence[BuiltJob]) -> None:
        cfg = self.cfg
        print(self.fire_summary(jobs))
        if not cfg.args.yes:
            if self.input("Enclosure closed, eye protection on, workpiece in place. Type FIRE to continue: ").strip() != "FIRE":
                raise SafetyError("operator did not type FIRE")
        union = None
        for j in jobs:
            if j.bbox:
                union = j.bbox if union is None else union.union(j.bbox)
        if union is None:
            raise SafetyError("jobs have no geometry")
        loops = max(1, cfg.args.frame_loops)
        ops = gg.frame_ops(union, cfg.args.feed, 0)
        for _ in range(loops - 1):
            ops += gg.polyline([(union.x, union.y), (union.x1, union.y), (union.x1, union.y1), (union.x, union.y1)], close=True)
        frame = encode_job(cfg, JobSpec("guide_frame", ops, "grbl", "frame"), 0)
        frame.gcode_file = "00_guide_frame.gcode"
        (cfg.out / frame.gcode_file).write_text("\n".join(frame.lines) + "\n")
        self.say(f"framing the job bounding box with the guide laser ({loops} loops)")
        fr = self.run_job(frame)
        self.rep.jobs.append(fr)
        if not fr.passed:
            raise SafetyError("guide frame failed: " + "; ".join(fr.reasons))
        if not cfg.args.yes and self.input("frame OK? [y/N] ").strip().lower() not in ("y", "yes"):
            raise SafetyError("operator rejected the frame")
        self.arm()

    def arm(self) -> None:
        self.say("arming (M10)")
        self.c.command_ok("M10", timeout=10.0)
        st, _ = self.c.wait_idle(10.0)
        if st is None:
            raise SafetyError("controller not Idle after M10")
        end = time.monotonic() + self.cfg.args.arm_timeout
        while time.monotonic() < end:
            armed = self.c.stats_s().get("atmega", {}).get("armed")
            if armed == "1":
                self.check("armed", True, "atmega armed=1")
                return
            time.sleep(0.5)
        raise SafetyError("ATmega did not report armed=1 after M10")

    def finish_safe(self) -> None:
        for cmd in ("M5",) + (() if self.cfg.args.keep_armed or not self.cfg.is_fire else ("M11",)):
            try:
                self.c.command(cmd, timeout=3.0)
            except Exception as e:                            # noqa: BLE001
                self.log.note(f"cleanup {cmd} failed: {e}")

    def emergency(self) -> None:
        """Ctrl-C / exception: cancel sequence, then disarm."""
        self.say("EMERGENCY: cancel sequence (0x18, M5, M9) and M11")
        try:
            self.c.cancel()
            self.c.command("M11", timeout=3.0)
        except Exception as e:                                # noqa: BLE001
            self.log.note(f"emergency cleanup failed: {e}")

    # ---- smoke
    def run_smoke(self, jobs: Sequence[BuiltJob]) -> None:
        c = self.c
        info = self.rep.device
        self.check("handshake", bool(info.get("version")), f"{info.get('version')} {info.get('options')}")
        self.check("RX window from $I", info.get("rx_window") not in (None, 127) or "OPT" not in info.get("options", ""),
                   f"{info.get('rx_window')} bytes")
        settings = self.rep.settings
        self.check("$$ lists settings", len(settings) >= 10 and all(k in settings for k in ("30", "130", "131")),
                   f"{len(settings)} settings, $30={settings.get('30')}")
        self.check_limits(settings)
        st = c.stats_s()
        self.rep.stats_before = st
        self.check("$S galvo+atmega", "galvo" in st and "atmega" in st, json.dumps(st.get("atmega", {})))
        self.check("ATmega link", st.get("atmega", {}).get("link") == "1", f"link={st.get('atmega', {}).get('link')}")
        rb = c.command("$RB")
        self.check("$RB", any(re.match(r"\[MSG:RB X=0x[0-9A-F]{4} Y=0x[0-9A-F]{4}\]", x) for x in rb), " ".join(rb[:2]))
        self.run_jobs(jobs, stop_on_fail=False)
        self.cancel_test()

    def cancel_test(self) -> None:
        c, cfg = self.c, self.cfg
        self.say("cancel test: long guide job, cancel after 1 s")
        lines = cancel_job_lines(cfg)
        assert_safe(lines, cfg.cap)
        if not self.preview_on():
            self.check("cancel test: guide on", False, "guide not on after M66")
        box: list[StreamResult] = []
        th = threading.Thread(target=lambda: box.append(c.stream(lines)), daemon=True)
        th.start()
        time.sleep(1.0)
        st_before = c.status(1.0)
        probe_stop = threading.Event()

        def probe() -> None:                  # fast '?' so the first Idle after 0x18 is timestamped
            while not probe_stop.wait(0.05):
                c.send_realtime(ord("?"))
        pt = threading.Thread(target=probe, daemon=True)
        pt.start()
        cr = c.cancel()
        th.join(10.0)
        time.sleep(0.3)
        probe_stop.set()
        pt.join(1.0)
        idle_t = next((s.t for s in self.log.statuses if s.t > cr.t_tx and s.state == "Idle"), None)
        to_idle = None if idle_t is None else idle_t - cr.t_tx
        deadline = time.monotonic() + 1.0       # async ATmega link: poll briefly
        guide = c.stats_s().get("atmega", {}).get("guide")
        while guide != "0" and time.monotonic() < deadline:
            time.sleep(0.05)
            guide = c.stats_s().get("atmega", {}).get("guide")
        self.rep.cancel = {
            "state_before_cancel": st_before.state if st_before else None,
            "banner_latency_s": cr.reset_latency, "cancel_to_idle_s": to_idle,
            "m5_reply": cr.m5, "m9_reply": cr.m9, "guide_after_cancel": guide,
            "stream_aborted": bool(box and box[0].aborted), "lines_acked_before_cancel": box[0].acks if box else None,
        }
        self.check("job running at cancel", bool(st_before and st_before.state == "Run"),
                   f"state={st_before.state if st_before else None}")
        self.check("stream stopped by cancel", bool(box and box[0].aborted and box[0].abort_reason == "cancelled"),
                   f"{box[0].abort_reason if box else 'no result'}")
        self.check("Idle within 2 s of 0x18", to_idle is not None and to_idle <= 2.0,
                   "never idle" if to_idle is None else f"{to_idle * 1000:.0f} ms")
        self.check("banner after 0x18", cr.reset_latency is not None,
                   "none" if cr.reset_latency is None else f"{cr.reset_latency * 1000:.1f} ms")
        self.check("M5/M9 accepted after cancel", cr.ok, f"M5={cr.m5} M9={cr.m9}")
        self.check("guide off after cancel", guide == "0", f"guide={guide}")


# ----------------------------------------------------------------- main ----

def connect(cfg: Config, logger: SessionLogger, mock) -> RayforgeClient:
    port = mock.path if mock else cfg.port
    c = RayforgeClient(port, logger, baud=cfg.args.baud, stall_timeout=cfg.args.stall_timeout,
                       poll_ms=cfg.args.poll_ms)
    return c.open()


def describe_jobs(cfg: Config, jobs: Sequence[BuiltJob]) -> None:
    total = sum(len(j.lines) for j in jobs)
    wa = f"{cfg.area[0]:g}x{cfg.area[1]:g}" + (" centred ($144=1)" if cfg.centered else "")
    print(f"plan {cfg.plan}: {len(jobs)} job(s), {total} lines, S cap {cfg.cap:g}, work area {wa}, "
          f"pattern box X {cfg.box.x:g}..{cfg.box.x1:g} Y {cfg.box.y:g}..{cfg.box.y1:g}, files in {cfg.out}")


def adopt_device_area(cfg: Config, settings: dict[str, float], sess: "Session") -> None:
    w, h = settings.get("130"), settings.get("131")
    if w is None or h is None or w <= 0 or h <= 0:
        raise ConfigError("cannot read $130/$131 from the device ($$); give --work-area WxH explicitly")
    centered = bool(settings.get("144", 0.0))
    apply_geometry(cfg, (float(w), float(h)), centered)
    sess.say(f"work area adopted from the device: {w:g}x{h:g} mm, "
             + ("centred origin ($144=1)" if centered else "origin at a corner ($144=0)"))


def run(cfg: Config, input_fn: Callable[[str], str] = input) -> int:
    a = cfg.args
    cfg.out.mkdir(parents=True, exist_ok=True)
    if cfg.is_fire and not a.fire and not a.dry_run:
        print("REFUSING: fire plans emit the real laser. Re-run with --fire (and read tools/rftest/README.md).",
              file=sys.stderr)
        return 2
    if a.dry_run:
        jobs = build_jobs(cfg)
        write_job_files(cfg, jobs)
        describe_jobs(cfg, jobs)
        for j in jobs:
            bb = j.bbox
            print(f"  {j.index:02d} {j.name:<22} {len(j.lines):>6} lines  "
                  + (f"bbox X {bb.x:.1f}..{bb.x1:.1f} Y {bb.y:.1f}..{bb.y1:.1f}" if bb else "")
                  + f"  S{j.s_values[:6]} clamps {j.clamped_coords}/{j.clamped_power}")
        return 0

    mock = None
    if a.mock:
        from mock_grbl import MockGrbl
        marea = parse_pair(a.mock_work_area, "--mock-work-area", "x") if a.mock_work_area else cfg.area
        mock = MockGrbl(slow_ms=a.mock_slow_ms, error_on=a.mock_error_on, speedup=a.mock_speedup,
                        work_area=marea)
        if a.mock_centered:
            mock.settings[144] = 1
        mock.start()
    logger = SessionLogger(cfg.out, verbose=a.verbose)
    rep = Report(cfg.plan, dt.datetime.now().isoformat(timespec="seconds"), mock.path if mock else str(cfg.port),
                 a.dialect or ("grbl_raster" if cfg.plan in RASTER_PLANS else "grbl"), "run_tests.py " + " ".join(sys.argv[1:]))
    client = connect(cfg, logger, mock)
    sess = Session(cfg, client, logger, rep, input_fn)
    code = 0
    try:
        info = client.handshake()
        sess.window = info.rx_window
        rep.device = {"version": info.version, "options": info.options, "rx_window": info.rx_window,
                      "machine": info.machine, "planner_blocks": info.planner_blocks}
        rep.settings = {k: v for k, v in sess.device_settings().items()}
        if cfg.auto:
            adopt_device_area(cfg, rep.settings, sess)
        jobs = build_jobs(cfg)
        write_job_files(cfg, jobs)
        describe_jobs(cfg, jobs)
        rep.stats_before = client.stats_s()
        sess.say(f"connected: {info.version} {info.options} window {info.rx_window}")
        if cfg.plan != "smoke":
            ok = sess.check_limits(rep.settings)
            if not ok and cfg.is_fire:
                raise SafetyError("work area / origin does not match the device (omit --work-area to adopt the device's)")
        if cfg.is_fire:
            sess.fire_prelude(jobs)
        if cfg.plan == "smoke":
            sess.run_smoke(jobs)
        else:
            sess.run_jobs(jobs, stop_on_fail=cfg.is_fire)
        sess.finish_safe()
        rep.stats_after = client.stats_s()
    except ConfigError as e:
        rep.notes.append(f"configuration error: {e}")
        print(f"ERROR: {e}", file=sys.stderr)
        sess.finish_safe()
        code = 2
    except KeyboardInterrupt:
        rep.interrupted = True
        rep.notes.append("interrupted by Ctrl-C")
        sess.emergency()
        code = 130
    except SafetyError as e:
        rep.notes.append(f"safety stop: {e}")
        print(f"SAFETY STOP: {e}", file=sys.stderr)
        sess.emergency()
        code = 2
    except Exception as e:                                    # noqa: BLE001
        rep.notes.append(f"exception: {type(e).__name__}: {e}")
        print(f"ERROR: {type(e).__name__}: {e}", file=sys.stderr)
        sess.emergency()
        code = 1
    finally:
        client.close()
        if mock:
            mock.check_counts()
            rep.mock_violations = list(mock.violations)
            mock.stop()
        logger.close()
    rep.finish()
    (cfg.out / "report.json").write_text(json.dumps(asdict(rep), indent=1, default=str))
    (cfg.out / "report.md").write_text(render_md(rep))
    print_table(rep)
    print(f"report: {cfg.out / 'report.md'}")
    return code or (0 if rep.passed else 1)


def main(argv: Optional[Sequence[str]] = None) -> int:
    cfg = make_config(argv)
    try:
        return run(cfg)
    except SafetyError as e:
        print(f"SAFETY: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
