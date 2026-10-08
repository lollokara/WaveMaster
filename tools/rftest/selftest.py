#!/usr/bin/env python3
"""
Offline self-test for the rftest suite (no hardware, no pytest).

    python3 tools/rftest/selftest.py [-v] [--keep]

Part 1: gcode_gen golden checks (exact Rayforge-style text, run-length raster
        linearisation, power cap, coordinate clamping, shapes inside the box).
Part 2: end-to-end runs against mock_grbl: smoke (incl. cancel test),
        guide-shapes, guide-raster (grbl_raster), guide-stress, fire safety
        gates, error injection, Ctrl-C handling, underrun and RX-overflow
        detection.
Exit status 0 = everything passed.
"""
from __future__ import annotations

import contextlib
import io
import json
import re
import shutil
import sys
import tempfile
import time
from pathlib import Path

import gcode_gen as gg
import run_tests
from gcode_gen import Box
from mock_grbl import MockGrbl
from rfclient import RayforgeClient, SessionLogger, parse_status, strip_comment

FAILS: list[str] = []
VERBOSE = False


def check(name: str, cond: bool, detail: str = "") -> bool:
    print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f"  [{detail}]" if detail and (not cond or VERBOSE) else ""))
    if not cond:
        FAILS.append(name)
    return cond


def lines_of(text: str) -> list[str]:
    return text.splitlines()


# ------------------------------------------------------------ part 1 -------

def test_golden() -> None:
    print("gcode_gen golden checks")
    sq = gg.encode(gg.square(Box(10, 10, 10, 10), 3000, 500), "grbl")
    expected = ("G21 ;Set units to mm\nG90 ;Absolute positioning\nG54\nG0 X10 Y10\nM4 S500\n"
                "G1 X20 F3000\nG1 Y20 F3000\nG1 X10 F3000\nG1 Y10 F3000\n"
                "M5 ;Ensure laser is off\nG0 X0 Y0 ;Return to origin\n")
    check("grbl: 10 mm square exact text", sq.text == expected, repr(sq.text))

    ops = [gg.SetSpeed(3000), gg.SetPower(500), gg.MoveTo(10, 10), gg.LineTo(20, 10),
           gg.MoveTo(20, 10), gg.MoveTo(30, 30), gg.SetPower(250), gg.LineTo(40, 30)]
    t = lines_of(gg.encode(ops, "grbl", preamble=False, postamble=False).text)
    check("grbl: M5 ends a cut before the next travel, zero-length travel dropped",
          t == ["G0 X10 Y10", "M4 S500", "G1 X20 F3000", "M5", "G0 X30 Y30", "M4 S250", "G1 X40 F3000"], str(t))
    t = lines_of(gg.encode([gg.SetSpeed(1000), gg.SetPower(100), gg.MoveTo(1, 1), gg.LineTo(5, 1),
                            gg.SetPower(200), gg.LineTo(5, 5)], "grbl", preamble=False, postamble=False).text)
    check("grbl: power change while active re-emits M4 S", t == ["G0 X1 Y1", "M4 S100", "G1 X5 F1000", "M4 S200", "G1 Y5 F1000"], str(t))

    arc = lines_of(gg.encode([gg.SetSpeed(3000), gg.SetPower(500)] + gg.circle_arcs(35, 35, 5), "grbl",
                             preamble=False, postamble=False).text)
    check("grbl: arcs are G2 X Y I J F with centre-relative I/J",
          arc == ["G0 X40 Y35", "M4 S500", "G2 X30 Y35 I-5 J0 F3000", "G2 X40 Y35 I5 J0 F3000"], str(arc))
    lin = gg.encode([gg.SetSpeed(3000), gg.SetPower(500)] + gg.circle_arcs(35, 35, 5), "grbl", arcs=False)
    check("grbl: --no-arcs linearises", not re.search(r"^G[23] ", lin.text, re.M) and lin.text.count("G1") > 20)

    row = gg.ScanLine(y=5, x0=10, powers=(0, 0, 255, 255, 128, 0), pixel_w=1.0, overscan=2.0, max_power=1000)
    r = lines_of(gg.encode([gg.SetSpeed(3000), row], "grbl_raster", preamble=False, postamble=False).text)
    check("raster: one G1 per RUN (4 runs for 6 pixels), overscan merged at S0",
          r == ["G0 X8 Y5 S0", "M4 S0", "G1 X12 S0 F3000", "G1 X14 S1000", "G1 X15 S502", "G1 X18 S0"], str(r))
    rev = gg.ScanLine(y=6, x0=10, powers=(0, 0, 255, 255, 128, 0), pixel_w=1.0, overscan=2.0, direction=-1, max_power=1000)
    r2 = lines_of(gg.encode([gg.SetSpeed(3000), row, rev], "grbl_raster", preamble=False, postamble=False).text)
    check("raster: bidirectional row runs right-to-left, F modal, S on every move, one M4 S0",
          r2[6:] == ["G0 Y6 S0", "G1 X15 S0", "G1 X14 S502", "G1 X12 S1000", "G1 X8 S0"] and
          sum(1 for x in r2 if x.startswith("M4")) == 1 and all("F" not in x for x in r2[6:]), str(r2))
    r3 = lines_of(gg.encode([gg.SetSpeed(3000), row], "grbl", preamble=False, postamble=False).text)
    check("raster rows in the grbl dialect re-emit M4 S per power change",
          [x for x in r3 if x.startswith("M4")] == ["M4 S0", "M4 S1000", "M4 S502", "M4 S0"], str(r3))
    blank = gg.encode([gg.ScanLine(1, 0, (0, 0, 0), 1.0, 2.0)], "grbl_raster", preamble=False, postamble=False)
    check("raster: blank rows are skipped", blank.text.strip() == "")

    rs = gg.encode(gg.square(Box(10, 10, 10, 10), 3000, 400) + gg.square(Box(30, 30, 5, 5), 3000, 400), "grbl_raster")
    t = lines_of(rs.text)
    check("grbl_raster: G0 carries S0, M4 S0 once, M5 only in postamble, F modal",
          sum(x.startswith("M4") for x in t) == 1 and sum(x.startswith("M5") for x in t) == 1
          and all(x.endswith("S0") for x in t if x.startswith("G0 X") and ";" not in x)
          and sum("F3000" in x for x in t) == 1 and "G1 X20 S400 F3000" in t, str(t))

    capped = gg.encode([gg.SetPower(1000), gg.MoveTo(1, 1), gg.LineTo(2, 2)], "grbl", max_s=300)
    check("power cap enforced (S1000 -> S300)", "M4 S300" in capped.text and "S1000" not in capped.text
          and capped.stats.clamped_power == 1, capped.text)
    cr = gg.encode([gg.SetPower(1000), gg.MoveTo(1, 1), gg.LineTo(2, 2)], "grbl_raster", max_s=250)
    check("power cap enforced in raster dialect", "S250" in cr.text and "S1000" not in cr.text)
    cl = gg.encode([gg.SetPower(100), gg.MoveTo(-5, 50), gg.LineTo(150, 120)], "grbl", work_area=(100, 100))
    check("coordinates clamped to the work area", "G0 X0 Y50" in cl.text and "G1 X100 Y100" in cl.text
          and cl.stats.clamped_coords == 2, cl.text)

    check("fmt_num", [gg.fmt_num(v) for v in (10, 2.5, -0.1254, 0.0004, 3.0005)] == ["10", "2.5", "-0.125", "0", "3.001"] or
          [gg.fmt_num(v) for v in (10, 2.5, -0.1254, 0.0004)] == ["10", "2.5", "-0.125", "0"])
    check("strip_comment", strip_comment("G21 ;Set units to mm") == "G21" and strip_comment("G0 X1 (move) ;x") == "G0 X1"
          and strip_comment("; only comment") == "" and strip_comment("  M5  ") == "M5")
    st = parse_status("<Run|MPos:1.500,2.000,0.000|FS:30000,100|Bf:511,1000|WCO:0.000,0.000,0.000>")
    check("status parser", bool(st) and st.state == "Run" and st.xy == (1.5, 2.0) and st.feed == 30000
          and st.spindle == 100 and st.bf == (511, 1000) and st.wco == (0.0, 0.0, 0.0))

    box = Box(25, 25, 50, 50)
    bad = []
    for name, fn in gg.SHAPES.items():
        ops = fn(box, 30000, 100)
        bb = gg.bbox(ops)
        enc = gg.encode(ops, "grbl", work_area=(100, 100))
        if (enc.stats.clamped_coords or bb is None or bb.x < 24.999 or bb.y < 24.999
                or bb.x1 > 75.001 or bb.y1 > 75.001):
            bad.append(name)
    check(f"all {len(gg.SHAPES)} shapes stay inside their box and the work area", not bad, str(bad))
    rows = [op.powers for op in gg.raster_dither(box, 3000, 255) if isinstance(op, gg.ScanLine)]
    half = len(rows[0]) // 2
    check("dither pixels are only 0/255 and the density ramps up left to right",
          all(set(r) <= {0, 255} for r in rows) and sum(sum(r[half:]) for r in rows) > 3 * sum(sum(r[:half]) for r in rows))
    lad = gg.encode(gg.power_ladder(box, 3000, [100, 200, 300]), "grbl").text
    check("power ladder patches use their S values", all(f"M4 S{s}" in lad for s in (100, 200, 300)))


# ------------------------------------------------------------ part 2 -------

def run_cli(args: list[str], out: Path, **kw) -> tuple[int, dict, str]:
    buf = io.StringIO()
    err = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(err):
        rc = run_tests.main(args + ["--out", str(out)])
    rep = {}
    if (out / "report.json").exists():
        rep = json.loads((out / "report.json").read_text())
    if VERBOSE:
        print(buf.getvalue(), err.getvalue())
    return rc, rep, buf.getvalue() + err.getvalue()


def files_ok(out: Path) -> bool:
    return all((out / f).exists() and (out / f).stat().st_size > 0
               for f in ("session.log", "events.jsonl", "firmware_msgs.log", "statuses.csv", "report.md", "report.json"))


def test_mock(tmp: Path) -> None:
    print("end-to-end against mock_grbl")
    base = ["--mock", "--mock-speedup", "40", "--poll-ms", "100"]

    rc, rep, _ = run_cli(base + ["smoke"], tmp / "smoke")
    check("smoke passes", rc == 0 and rep.get("passed"), str([c for c in rep.get("checks", []) if not c["passed"]]))
    check("smoke writes all log/report files", files_ok(tmp / "smoke"))
    c = rep.get("cancel", {})
    check("cancel test: state Run before, Idle within 2 s of 0x18, M5/M9 ok",
          c.get("state_before_cancel") == "Run" and (c.get("cancel_to_idle_s") or 9) <= 2.0 and c.get("m5_reply") == ["ok"]
          and c.get("m9_reply") == ["ok"] and c.get("stream_aborted"), json.dumps(c))
    check("smoke: no mock protocol violations", not rep.get("mock_violations"), str(rep.get("mock_violations")))

    for plan, dialect in (("guide-shapes", "grbl"), ("guide-raster", "grbl_raster"), ("guide-stress", "grbl"),
                          ("guide-frame", "grbl")):
        out = tmp / plan
        rc, rep, _ = run_cli(base + [plan], out)
        jobs = rep.get("jobs", [])
        check(f"{plan} passes ({len(jobs)} jobs, {sum(j['lines'] for j in jobs)} lines)", rc == 0 and rep.get("passed"),
              str([(j["name"], j["reasons"]) for j in jobs if not j["passed"]]))
        check(f"{plan}: dialect {dialect}, no violations, files written",
              rep.get("dialect") == dialect and not rep.get("mock_violations") and files_ok(out))
        check(f"{plan}: window from $I used (1024), in-flight never above it",
              all(j["window"] == 1024 and j["max_inflight"] <= 1024 for j in jobs))
    jt = (tmp / "guide-raster").glob("*raster_dither.gcode")
    txt = next(jt).read_text()
    check("raster job: M4 S0 once, every G0/G1 carries S, no comments streamed",
          txt.count("M4 S0") == 1 and all(" S" in l for l in lines_of(txt) if l.startswith(("G0 X", "G1 X", "G0 Y", "G1 Y"))
                                         and not l.startswith("G0 X0 Y0")) and ";" not in txt, txt[:200])
    sg = json.loads((tmp / "guide-stress" / "report.json").read_text())
    check("stress: lines/s measured, underruns 0",
          all(j["lines_per_s"] > 0 and j["underruns_delta"] == 0 for j in sg["jobs"]))
    ev = [json.loads(l) for l in (tmp / "guide-shapes" / "events.jsonl").read_text().splitlines()]
    kinds = {e["kind"] for e in ev}
    check("events.jsonl has tx_line/rx_line/ack/status/msg/realtime", {"tx_line", "rx_line", "ack", "status", "msg", "realtime"} <= kinds, str(kinds))
    slog = (tmp / "guide-shapes" / "session.log").read_text()
    check("session.log shows TX/RX and realtime bytes", " TX  " in slog and " RX  " in slog and "TX  ?" in slog)
    check("firmware_msgs.log holds [MSG:...] lines", "[MSG:galvo" in (tmp / "guide-shapes" / "firmware_msgs.log").read_text())

    # --- fire plans: safety gates
    rc, _, out = run_cli(base + ["fire-power-ladder"], tmp / "nofire")
    check("fire plan refused without --fire (rc 2)", rc == 2 and "REFUSING" in out)
    rc, rep, out = run_cli(base + ["fire-shapes", "--fire", "--yes"], tmp / "fire1")
    check("fire-shapes --fire --yes passes against the mock", rc == 0 and rep.get("passed"),
          str([(j["name"], j["reasons"]) for j in rep.get("jobs", []) if not j["passed"]]))
    sl = (tmp / "fire1" / "session.log").read_text()
    order = [sl.find(x) for x in ("TX  M66", "TX  M67", "TX  M10", "TX  M11")]
    check("fire order: guide frame (M66..M67) -> M10 arm -> job -> M5 -> M11",
          all(i >= 0 for i in order) and order == sorted(order) and sl.find("TX  M5", order[2]) < sl.rfind("TX  M11"), str(order))
    check("fire: frame job precedes first fire job", rep["jobs"][0]["mode"] == "frame")

    rc, rep, _ = run_cli(base + ["fire-power-ladder", "--fire", "--yes", "--powers", "100,500,1000", "--max-power", "200"],
                         tmp / "cap")
    allg = "\n".join(p.read_text() for p in (tmp / "cap").glob("*.gcode"))
    svals = [float(v) for v in re.findall(r"(?<![A-Za-z])S([\d.]+)", allg)]
    check("--max-power 200: no S above 200 in any streamed file", svals and max(svals) <= 200 and rc == 0, str(max(svals or [0])))
    rc, _, out = run_cli(["--mock", "fire-shapes", "--fire", "--power", "900", "--max-power", "50", "--dry-run"], tmp / "drycap")
    svals = [float(v) for p in (tmp / "drycap").glob("*.gcode") for v in re.findall(r"(?<![A-Za-z])S([\d.]+)", p.read_text())]
    check("dry-run writes capped gcode only (no device access)", rc == 0 and max(svals) <= 50 and not (tmp / "drycap" / "session.log").exists())

    # --- prompts and Ctrl-C
    cfg = run_tests.make_config(base + ["fire-shapes", "--fire", "--out", str(tmp / "cc")])
    buf = io.StringIO()

    def interrupt(prompt: str) -> str:
        if "FIRE" in prompt:
            return "FIRE"
        raise KeyboardInterrupt
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        rc = run_tests.run(cfg, input_fn=interrupt)
    sl = (tmp / "cc" / "session.log").read_text()
    tail = sl[sl.rfind("EMERGENCY"):] if "EMERGENCY" in sl else ""
    check("Ctrl-C at the frame prompt: 0x18 -> M5 -> M9 -> M11 and rc 130",
          rc == 130 and tail.find("<0x18>") < tail.find("TX  M5") < tail.find("TX  M9") < tail.find("TX  M11") and "<0x18>" in tail, tail[:300])
    cfg = run_tests.make_config(base + ["fire-shapes", "--fire", "--out", str(tmp / "nofire2")])
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        rc = run_tests.run(cfg, input_fn=lambda p: "no")
    sl = (tmp / "nofire2" / "session.log").read_text()
    check("not typing FIRE: nothing fired, M10 never sent (rc 2)", rc == 2 and "TX  M10" not in sl and "TX  M4" not in sl)

    # --- error injection (fire): abort -> cancel sequence -> M5 -> M11
    rc, rep, _ = run_cli(base + ["fire-shapes", "--fire", "--yes", "--mock-error-on", r"G1 X59\.567 Y73"], tmp / "err")
    sl = (tmp / "err" / "session.log").read_text()
    i = sl.find("job aborted")
    seq = [sl.find(x, i) for x in ("<0x18>", "TX  M5", "TX  M9", "TX  M11")]
    check("error: during a fire job aborts it with 0x18, M5, M9 and ends disarmed",
          rc == 1 and i > 0 and all(s > 0 for s in seq) and seq == sorted(seq) and len(rep["jobs"]) < 9, str(seq))

    # --- device behaviour detection
    rc, rep, _ = run_cli(base + ["guide-frame", "--frame-loops", "1", "--mock-slow-ms", "80"], tmp / "slow")
    check("slow device (80 ms/line) is reported as underruns (fail)",
          rc == 1 and any((j["underruns_delta"] or 0) > 0 for j in rep.get("jobs", [])),
          str([(j["underruns_delta"], j["reasons"]) for j in rep.get("jobs", [])]))
    test_overflow_detection(tmp)
    test_stall(tmp)


def test_overflow_detection(tmp: Path) -> None:
    mock = MockGrbl(speedup=100, slow_ms=2.0)       # slow device: bytes pile up in its RX buffer
    path = mock.start()
    log = SessionLogger(tmp / "ovf")
    c = RayforgeClient(path, log).open()
    try:
        c.handshake()
        good = [f"G1 X{10 + (i % 40)} Y{10 + (i % 7)} F6000" for i in range(300)]
        res = c.stream(good)
        check("RayforgeClient streams 300 lines with character counting, no overflow",
              res.ok and not mock.violations and res.max_inflight <= 1024, str(mock.violations))
        c.wait_idle(20)
        c.rx_window = 4096                      # lie about the buffer: client overruns the device
        c.command("M5")
        res = c.stream(good)
        check("mock records OVERFLOW when the client exceeds the 1024-byte window",
              any("OVERFLOW" in v for v in mock.violations) and bool(res.errors or res.aborted), str(mock.violations[:2]))
        c.rx_window = 1024
        c.send_line("G1 X" + "1" * 300)
        time.sleep(0.3)
        check("mock records LONG_LINE for lines >= 255 chars", any("LONG_LINE" in v for v in mock.violations))
    finally:
        c.close()
        mock.stop()
        log.close()


def test_stall(tmp: Path) -> None:
    mock = MockGrbl(speedup=100, slow_ms=700)
    path = mock.start()
    log = SessionLogger(tmp / "stall")
    c = RayforgeClient(path, log, stall_timeout=0.3).open()
    try:
        c.handshake()
        res = c.stream(["G0 X1 Y1", "G0 X2 Y2", "G0 X3 Y3"])
        check("stall handling: slow acks make the client poll '?', device answers, job completes",
              res.ok and res.stall_polls >= 1, f"stall_polls={res.stall_polls} ok={res.ok}")
    finally:
        c.close()
        mock.stop()
        log.close()


def main() -> int:
    global VERBOSE
    VERBOSE = "-v" in sys.argv
    keep = "--keep" in sys.argv
    tmp = Path(tempfile.mkdtemp(prefix="rftest_selftest_"))
    t0 = time.monotonic()
    try:
        test_golden()
        test_mock(tmp)
    finally:
        if keep:
            print(f"kept {tmp}")
        else:
            shutil.rmtree(tmp, ignore_errors=True)
    print(f"\n{'ALL PASSED' if not FAILS else str(len(FAILS)) + ' FAILED: ' + '; '.join(FAILS)}  ({time.monotonic() - t0:.1f} s)")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
