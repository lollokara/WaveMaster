#!/usr/bin/env python3
"""WaveMaster flashing wizard (macOS/Linux): ATmega328P companion + ESP32-S3.

Usage: python tools/flash_boards.py [--no-color] [SUBCOMMAND] [options]
Subcommands: detect, build, flash-atmega, flash-esp32, verify, backup,
restore FILE, all (default), selftest.  See tools/README_FLASHING.md.
"""
from __future__ import annotations

import argparse
import platform
import datetime as dt
import json
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    sys.stderr.write("pyserial missing. Run: bash tools/setup_mac.sh  (or pip install pyserial)\n")
    sys.exit(2)

REPO = Path(__file__).resolve().parent.parent
ATMEGA_DIR = REPO / "ArduinoCompanion"
BACKUP_DIR = REPO / "tools" / "backups"
ENV_ESP = "esp32-s3-devkitc-1"
ENV_AVR = "arduino_mini"
ESP_VID, ESP_PID = 0x303A, 0x1001
ARDUINO_VIDS = {0x1A86: "CH340", 0x0403: "FTDI", 0x10C4: "CP210x"}

EXIT_OK, EXIT_FAIL, EXIT_USAGE, EXIT_ABORT = 0, 1, 2, 130


# ------------------------------------------------------------- output ------
class Ui:
    color = True

    @classmethod
    def c(cls, code: str, s: str) -> str:
        return f"\033[{code}m{s}\033[0m" if cls.color else s


def info(s: str) -> None: print(Ui.c("36", "[..] ") + s)
def ok(s: str) -> None: print(Ui.c("32", "[OK] ") + s)
def warn(s: str) -> None: print(Ui.c("33", "[!!] ") + s)
def err(s: str) -> None: print(Ui.c("31", "[XX] ") + s)
def head(s: str) -> None: print("\n" + Ui.c("1;34", f"== {s} =="))


class Abort(Exception):
    pass


def ask_yes(prompt: str, default: bool = False) -> bool:
    suffix = " [Y/n] " if default else " [y/N] "
    try:
        ans = input(Ui.c("1", prompt) + suffix).strip().lower()
    except EOFError:
        return default
    return default if not ans else ans.startswith("y")


def pause(prompt: str) -> None:
    try:
        input(Ui.c("1", prompt + " (Enter to continue, Ctrl-C to abort) "))
    except EOFError:
        pass


# -------------------------------------------------------------- ports ------
@dataclass
class PortInfo:
    device: str
    vid: int | None
    pid: int | None
    desc: str
    kind: str  # "esp32", "arduino", "unknown"


def classify(vid: int | None, pid: int | None) -> str:
    if vid == ESP_VID and pid == ESP_PID:
        return "esp32"
    if vid in ARDUINO_VIDS:
        return "arduino"
    return "unknown"


def scan_ports() -> list[PortInfo]:
    out = []
    for p in sorted(list_ports.comports(), key=lambda x: x.device):
        out.append(PortInfo(p.device, p.vid, p.pid, p.description or "n/a", classify(p.vid, p.pid)))
    # macOS exposes both /dev/tty.* and /dev/cu.*; use cu.* only.
    cu = [p for p in out if "/cu." in p.device]
    return cu if cu else out


def cmd_detect(_: argparse.Namespace | None = None) -> int:
    head("Serial ports")
    ports = scan_ports()
    if not ports:
        warn("No serial ports found. Check the USB cables (data cable, not charge-only).")
        return EXIT_FAIL
    labels = {"esp32": "ESP32-S3 (USB-Serial-JTAG)", "arduino": "Arduino/ATmega USB-serial",
              "unknown": "unknown"}
    for p in ports:
        vp = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None and p.pid is not None else "----:----"
        chip = ARDUINO_VIDS.get(p.vid or -1, "")
        extra = f" {chip}" if p.kind == "arduino" else ""
        print(f"  {p.device:<34} {vp}  {labels[p.kind]}{extra}  ({p.desc})")
    return EXIT_OK


def pick_port(kind: str, explicit: str | None, what: str) -> str:
    """Return a port for `kind`: explicit, auto if unambiguous, else ask."""
    if explicit:
        return explicit
    cands = [p for p in scan_ports() if p.kind == kind]
    if len(cands) == 1:
        ok(f"{what}: auto-selected {cands[0].device}")
        return cands[0].device
    pool = cands or scan_ports()
    if not pool:
        raise Abort(f"No serial port for {what}. Plug it in and retry, or pass --port.")
    if not cands:
        warn(f"No port identified as {what}; choose from all ports.")
    for i, p in enumerate(pool, 1):
        print(f"  {i}) {p.device}  {p.desc}")
    try:
        sel = input(Ui.c("1", f"Select port for {what} [1-{len(pool)}]: ")).strip()
    except EOFError:
        raise Abort("No input available; pass --port.") from None
    if not sel.isdigit() or not 1 <= int(sel) <= len(pool):
        raise Abort("Invalid selection.")
    return pool[int(sel) - 1].device


# ---------------------------------------------------------------- pio ------
def find_pio() -> str | None:
    venv = REPO / ".venv" / "bin" / "pio"
    if venv.exists():
        return str(venv)
    return shutil.which("pio") or shutil.which("platformio")


def run_pio(args: list[str], cwd: Path, label: str) -> tuple[int, str]:
    """Run pio, streaming output; return (rc, captured text)."""
    pio = find_pio()
    if not pio:
        err("PlatformIO not found. Run: bash tools/setup_mac.sh")
        return 127, ""
    cmd = [pio, *args]
    info(f"{label}: {' '.join(cmd[1:])}   (in {cwd.relative_to(REPO) if cwd != REPO else '.'})")
    buf: list[str] = []
    try:
        proc = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, errors="replace")
        assert proc.stdout is not None
        for line in proc.stdout:
            sys.stdout.write("    " + line)
            buf.append(line)
        rc = proc.wait()
    except KeyboardInterrupt:
        proc.terminate()
        raise
    except OSError as e:
        err(f"Could not run pio: {e}")
        return 127, ""
    out = "".join(buf)
    if rc != 0:
        rosetta_hint(out)
        module_hint(out, pio)
    return rc, out


def rosetta_missing() -> bool:
    """True on Apple Silicon when Rosetta 2 cannot run x86_64 binaries."""
    if sys.platform != "darwin" or platform.machine() != "arm64":
        return False
    try:
        return subprocess.run(["/usr/bin/arch", "-x86_64", "/usr/bin/true"],
                              capture_output=True).returncode != 0
    except OSError:
        return True


def module_hint(output: str, pio: str) -> None:
    m = re.search(r"No module named '([\w.]+)'", output)
    if not m:
        return
    py = Path(pio).with_name("python")
    py_cmd = str(py) if py.exists() else sys.executable
    warn(f"A Python module PlatformIO's tools need is missing ({m.group(1)}). Install esptool's\n"
         f"      dependencies into the Python that runs PlatformIO, then retry:\n"
         f"          {py_cmd} -m pip install intelhex bitstring reedsolo ecdsa pyyaml cryptography")


def rosetta_hint(output: str) -> None:
    low = output.lower()
    if "bad cpu type" in low or "system error -86" in low:
        warn("An Intel-only PlatformIO tool (avr-gcc / ninja) could not run on this Apple Silicon Mac.\n"
             "      Install Rosetta 2, then retry:\n"
             "          softwareupdate --install-rosetta --agree-to-license")


def refresh_sdkconfig() -> None:
    """Delete the generated ESP-IDF sdkconfig when it predates sdkconfig.defaults
    or platformio.ini. ESP-IDF only applies sdkconfig.defaults when it creates
    the file, so an old copy silently keeps stale settings (e.g. an 8 MB flash
    size on this 4 MB board, which boot-loops). It is untracked and regenerated
    on the next build."""
    gen = REPO / f"sdkconfig.{ENV_ESP}"
    if not gen.exists():
        return
    newest = max((REPO / n).stat().st_mtime for n in ("sdkconfig.defaults", "platformio.ini")
                 if (REPO / n).exists())
    if gen.stat().st_mtime < newest:
        info(f"{gen.name} is older than sdkconfig.defaults/platformio.ini - regenerating it")
        gen.unlink()


def build_one(name: str, cwd: Path, env: str) -> bool:
    if env == ENV_ESP:
        refresh_sdkconfig()
    rc, _ = run_pio(["run", "-e", env], cwd, f"build {name}")
    (ok if rc == 0 else err)(f"build {name}: {'PASS' if rc == 0 else 'FAIL'}")
    return rc == 0


def cmd_build(_: argparse.Namespace | None = None) -> int:
    head("Build")
    if rosetta_missing():
        warn("Rosetta 2 is not installed; the AVR toolchain and ninja are Intel-only and will fail.\n"
             "      Run: softwareupdate --install-rosetta --agree-to-license")
    r1 = build_one("ATmega companion", ATMEGA_DIR, ENV_AVR)
    r2 = build_one("ESP32-S3 firmware", REPO, ENV_ESP)
    return EXIT_OK if r1 and r2 else EXIT_FAIL


def upload_hints(output: str, which: str) -> None:
    low = output.lower()
    if "resource busy" in low or "could not exclusively lock" in low or "permission denied" in low:
        warn("Port busy. Close other serial apps (Rayforge, LightBurn, screen, Arduino IDE, "
             "serial monitors) and retry.")
    if which == "avr" and ("not in sync" in low or "stk500" in low):
        warn("avrdude 'stk500_getsync(): not in sync' - typical causes:\n"
             "      - wrong bootloader speed: new-bootloader Nano/Mini clones use 115200. In\n"
             "        ArduinoCompanion/platformio.ini change under [env:arduino_mini]:\n"
             "            upload_speed = 115200\n"
             "        (this tool never edits platformio.ini; revert to 57600 for old bootloaders)\n"
             "      - wrong port (pick the CH340/FTDI one), charge-only cable, board not powered\n"
             "      - serial monitor still open, or the ESP32 TX/RX wired on the Arduino RX/TX\n"
             "        and driving the line: unplug the level-shifter side or disconnect GPIO43/44")
    if which == "esp" and any(k in low for k in ("failed to connect", "no serial data", "wrong boot mode",
                                                   "could not open", "no such file", "device not configured")):
        warn("ESP32-S3 not in download mode / port vanished: hold BOOT, tap RESET, release BOOT, "
             "then re-run (the port may re-enumerate with a new name; run 'detect').")


def cmd_flash_atmega(a: argparse.Namespace) -> int:
    head("Flash ATmega328P companion (do this FIRST)")
    print("  Safety checklist:\n"
          "   - Laser OFF / powered down, and DB25 cable UNPLUGGED (or laser supply off).\n"
          "   - The old companion firmware drives D2/D4 as PWM outputs; the new wiring moves\n"
          "     DB25 pins 19/20 to ESP32 GPIO4/GPIO3. Flash before reconnecting the laser.\n"
          "   - The ESP32 may stay connected.")
    if not getattr(a, "yes", False) and not ask_yes("Laser is OFF and DB25 is unplugged / laser powered down?"):
        raise Abort("Aborted: safety checklist not confirmed.")
    port = pick_port("arduino", a.port, "ATmega (Arduino) USB-serial")
    if not build_one("ATmega companion", ATMEGA_DIR, ENV_AVR):
        return EXIT_FAIL
    rc, out = run_pio(["run", "-e", ENV_AVR, "-t", "upload", "--upload-port", port], ATMEGA_DIR, "upload ATmega")
    if rc != 0:
        err("ATmega upload FAILED")
        upload_hints(out, "avr")
        return EXIT_FAIL
    ok("ATmega companion flashed.")
    warn("Close any serial monitor on the Arduino's USB port: while it is open the ATmega "
         "cannot reach the ESP32 link reliably (CH340 sits in parallel on those lines).")
    return EXIT_OK


def cmd_flash_esp32(a: argparse.Namespace) -> int:
    head("Flash ESP32-S3")
    port = pick_port("esp32", a.port, "ESP32-S3 native USB")
    if a.erase:
        warn("--erase wipes flash incl. NVS: ALL $ settings (calibration, scale, offsets, delays) "
             "return to defaults. Run 'backup' first if the board is working.")
        if not ask_yes("Erase flash?"):
            raise Abort("Aborted: erase not confirmed.")
        rc, out = run_pio(["run", "-e", ENV_ESP, "-t", "erase", "--upload-port", port], REPO, "erase ESP32")
        if rc != 0:
            err("Erase FAILED")
            upload_hints(out, "esp")
            return EXIT_FAIL
    if not build_one("ESP32-S3 firmware", REPO, ENV_ESP):
        return EXIT_FAIL
    refresh_sdkconfig()
    rc, out = run_pio(["run", "-e", ENV_ESP, "-t", "upload", "--upload-port", port], REPO, "upload ESP32")
    if rc != 0:
        err("ESP32 upload FAILED")
        upload_hints(out, "esp")
        return EXIT_FAIL
    ok("ESP32-S3 firmware flashed.")
    return EXIT_OK


# ------------------------------------------------- response parsing -------
# Pure functions: no I/O, covered by `selftest`.
RE_KV = re.compile(r"(\w+)=([^\s\]]+)")


def _kv(line: str) -> dict[str, str]:
    return dict(RE_KV.findall(line))


def _num(s: str) -> float | None:
    m = re.match(r"-?(0x[0-9a-fA-F]+|\d+(\.\d+)?)", s)
    if not m:
        return None
    return float(int(m.group(0), 16)) if m.group(0).lower().lstrip("-").startswith("0x") else float(m.group(0))


def parse_status(line: str) -> bool:
    s = line.strip()
    return s.startswith("<") and s.endswith(">")


def parse_info(lines: list[str]) -> dict:
    res: dict = {"version": None, "rx": None, "queue": None}
    for ln in lines:
        m = re.search(r"\[VER:([^\]]*)\]", ln)
        if m:
            res["version"] = m.group(1)
        m = re.search(r"\[OPT:[^,\]]*,(\d+),(\d+)\]", ln)
        if m:
            res["queue"], res["rx"] = int(m.group(1)), int(m.group(2))
    return res


def parse_atmega(lines: list[str]) -> dict | None:
    for ln in lines:
        if "[MSG:atmega" in ln:
            kv = _kv(ln)
            try:
                return {"link": int(kv["link"]), "armed": int(kv["armed"]), "ready": int(kv["ready"]),
                        "guide": int(kv.get("guide", "0")), "power": int(kv["power"]),
                        "stat": kv.get("stat", "?"), "vdet_mv": int(float(_num(kv["vdet"]) or 0)),
                        "err": int(kv["err"])}
            except (KeyError, ValueError):
                return None
    return None


def parse_galvo(lines: list[str]) -> dict | None:
    for ln in lines:
        if "[MSG:galvo" in ln:
            kv = _kv(ln)
            try:
                return {"chunks": int(kv["chunks"]), "underruns": int(kv["underruns"]),
                        "barriers": int(kv["barriers"]), "ticks": int(kv["ticks"]),
                        "tick_us": float(kv["tick_us"])}
            except (KeyError, ValueError):
                return None
        if "motion hardware fault" in ln:
            return {"fault": True}
    return None


def parse_rb(lines: list[str]) -> dict | None:
    """Return {'x':int,'y':int} on success, {'unavailable':True} when refused, None if absent."""
    for ln in lines:
        m = re.search(r"\[MSG:RB X=0x([0-9A-Fa-f]+) Y=0x([0-9A-Fa-f]+)\]", ln)
        if m:
            return {"x": int(m.group(1), 16), "y": int(m.group(2), 16)}
        if "RB unavailable" in ln:
            return {"unavailable": True}
    return None


def parse_settings(lines: list[str]) -> dict[str, str]:
    out: dict[str, str] = {}
    for ln in lines:
        m = re.match(r"^\s*\$(\d+)=(\S+)\s*$", ln)
        if m:
            out[m.group(1)] = m.group(2)
    return out


def parse_boot_selftest(lines: list[str]) -> bool | None:
    """True if a self-test OK line was seen, False if MISMATCH/error, None if no line."""
    res: bool | None = None
    for ln in lines:
        low = ln.lower()
        if "self-test" in low:
            if "ok" in low and "mismatch" not in low and "error" not in low:
                res = True if res is None else res
            else:
                res = False
    return res


@dataclass
class Check:
    name: str
    status: str  # PASS / FAIL / WARN / SKIP
    detail: str = ""
    fix: str = ""


def evaluate(status_seen: bool, info_d: dict, atm: dict | None, galvo: dict | None,
             rb: dict | None, boot: bool | None) -> list[Check]:
    c: list[Check] = []
    c.append(Check("Status report '?'", "PASS" if status_seen else "FAIL",
                   "" if status_seen else "no <...> within 6 s",
                   "" if status_seen else "Wrong port? Firmware not flashed / crashed (boot loop)? "
                   "Hold BOOT+tap RESET and reflash; make sure no other app has the port."))
    ver = info_d.get("version")
    if ver and "WaveMaster" in ver:
        c.append(Check("Firmware ID ($I)", "PASS", f"VER:{ver}"))
    else:
        c.append(Check("Firmware ID ($I)", "FAIL", f"VER:{ver}", "Expected 1.1h.WaveMaster; reflash ESP32."))
    if info_d.get("rx") == 1024:
        c.append(Check("RX window (OPT)", "PASS", "rx=1024"))
    else:
        c.append(Check("RX window (OPT)", "FAIL", f"rx={info_d.get('rx')}", "Expected 1024; firmware mismatch, reflash."))
    if atm is None:
        c.append(Check("ATmega link ($S)", "FAIL", "no [MSG:atmega ...] line", "$S not answered; reflash ESP32."))
    elif atm["link"] == 1:
        c.append(Check("ATmega link ($S)", "PASS",
                       f"link=1 armed={atm['armed']} ready={atm['ready']} power={atm['power']} "
                       f"vdet={atm['vdet_mv']}mV err={atm['err']}"))
        if atm["err"]:
            c.append(Check("ATmega link errors", "WARN", f"err={atm['err']}",
                           "Frame errors seen: check level shifter, grounds, cable length, baud."))
    else:
        c.append(Check("ATmega link ($S)", "FAIL", "link=0",
                       "ATmega not flashed (flash-atmega) / level shifter unpowered / TX-RX swapped "
                       "(ESP32 GPIO44 TX -> ATmega RX, GPIO43 RX <- ATmega TX) / Arduino USB serial "
                       "monitor still open (close it, or unplug the Arduino USB) / no heartbeat yet "
                       "(wait a few seconds, rerun verify)."))
    if galvo is None:
        c.append(Check("Galvo stats ($S)", "FAIL", "missing", "$S not answered; reflash ESP32."))
    elif galvo.get("fault"):
        c.append(Check("Galvo stats ($S)", "FAIL", "motion hardware fault",
                       "DAC/motion init failed: check AD3552R SPI wiring and power, then reset."))
    else:
        bad = galvo["underruns"] > 0
        c.append(Check("Galvo stats ($S)", "WARN" if bad else "PASS",
                       f"chunks={galvo['chunks']} underruns={galvo['underruns']} "
                       f"barriers={galvo['barriers']} tick_us={galvo['tick_us']:.2f}",
                       "Underruns while idle are unexpected; reset the board and re-run." if bad else ""))
    if rb is None:
        c.append(Check("DAC readback ($RB)", "FAIL", "no reply", "Reflash ESP32."))
    elif rb.get("unavailable"):
        c.append(Check("DAC readback ($RB)", "FAIL", "RB unavailable",
                       "DAC init failed or SPI error/busy: check SPI wiring (SCLK/CS/SDIO, GPIO7 OP1 "
                       "strap), AD3552R supplies and reference per docs/WIRING.md; reset and retry."))
    else:
        c.append(Check("DAC readback ($RB)", "PASS", f"X=0x{rb['x']:04X} Y=0x{rb['y']:04X}"))
    if boot is True:
        c.append(Check("Boot self-test", "PASS", "self-test OK seen"))
    elif boot is False:
        c.append(Check("Boot self-test", "FAIL", "self-test error/mismatch in boot log",
                       "DAC readback mismatch at boot: check SPI wiring/quad-SPI strap (GPIO7)."))
    else:
        c.append(Check("Boot self-test", "SKIP", "no boot log captured (board was already running)"))
    return c


def print_checks(checks: list[Check]) -> bool:
    head("Verification checklist")
    colors = {"PASS": "32", "FAIL": "31", "WARN": "33", "SKIP": "90"}
    for ck in checks:
        print(f"  [{Ui.c(colors[ck.status], ck.status)}] {ck.name}" + (f" - {ck.detail}" if ck.detail else ""))
        if ck.fix and ck.status in ("FAIL", "WARN"):
            print(f"         fix: {ck.fix}")
    failed = [c for c in checks if c.status == "FAIL"]
    (err if failed else ok)(f"{len(failed)} check(s) failed" if failed else "All checks passed")
    return not failed


# ---------------------------------------------------------- serial I/O -----
class Board:
    """Line-oriented GRBL-style session on the ESP32 port."""

    def __init__(self, port: str, baud: int = 115200) -> None:
        self.port = port
        # Open without toggling DTR/RTS (would reset the board on some adapters).
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, baud, 0.1
        self.ser.dtr, self.ser.rts = False, False
        self.buf = b""
        self.ser.open()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def _readline(self, deadline: float) -> str | None:
        while time.monotonic() < deadline:
            if b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                return line.decode("utf-8", "replace").strip()
            self.buf += self.ser.read(256)
        return None

    def drain(self, secs: float = 0.3) -> list[str]:
        lines, end = [], time.monotonic() + secs
        while (ln := self._readline(end)) is not None:
            if ln:
                lines.append(ln)
        return lines

    def command(self, cmd: str, timeout: float = 4.0) -> tuple[list[str], str | None]:
        """Send cmd; return (lines, terminator) where terminator is 'ok', 'error:N' or None."""
        self.ser.write(cmd.encode() + b"\n")
        lines: list[str] = []
        end = time.monotonic() + timeout
        while (ln := self._readline(end)) is not None:
            if ln == "ok" or ln.startswith("error:"):
                return lines, ln
            if ln:
                lines.append(ln)
        return lines, None

    def wait_status(self, timeout: float = 6.0) -> tuple[bool, list[str]]:
        seen: list[str] = []
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.ser.write(b"?")
            for ln in self.drain(0.5):
                seen.append(ln)
                if parse_status(ln):
                    return True, seen
        return False, seen


def open_board(port: str) -> Board | None:
    try:
        return Board(port)
    except serial.SerialException as e:
        msg = str(e)
        if "busy" in msg.lower() or "lock" in msg.lower() or "Errno 16" in msg:
            err(f"{port}: Resource busy. Close other serial apps (Rayforge, LightBurn, screen, "
                "Arduino IDE / serial monitors, a running `pio device monitor`) and retry.")
        elif "No such file" in msg or "Errno 2" in msg or "Errno 6" in msg:
            err(f"{port}: port not found (board reset or unplugged?). Run 'detect'.")
        elif "ermission" in msg:
            err(f"{port}: permission denied ({msg}).")
        else:
            err(f"{port}: {msg}")
        return None


def cmd_verify(a: argparse.Namespace) -> int:
    head("Verify ESP32-S3")
    port = pick_port("esp32", a.port, "ESP32-S3 native USB")
    b = open_board(port)
    if not b:
        return EXIT_FAIL
    try:
        info(f"Connected to {port}; waiting for status report (6 s)...")
        seen, boot_lines = b.wait_status(6.0)
        b.drain(0.3)
        i_lines, _ = b.command("$I")
        s_lines, _ = b.command("$S")
        rb_lines, _ = b.command("$RB")
        st_lines, term = b.command("$$", timeout=8.0)
        settings = parse_settings(st_lines)
        if settings:
            path = write_backup(settings, port)
            ok(f"Settings backup ({len(settings)} entries): {path.relative_to(REPO)}")
        else:
            warn("Could not read $$ settings (no '$N=v' lines).")
        checks = evaluate(seen, parse_info(i_lines), parse_atmega(s_lines), parse_galvo(s_lines),
                          parse_rb(rb_lines), parse_boot_selftest(boot_lines))
        return EXIT_OK if print_checks(checks) else EXIT_FAIL
    except serial.SerialException as e:
        err(f"Serial error: {e}")
        return EXIT_FAIL
    finally:
        b.close()


# ---------------------------------------------------- backup / restore -----
def write_backup(settings: dict[str, str], port: str) -> Path:
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    path = BACKUP_DIR / f"settings-{stamp}.json"
    path.write_text(json.dumps({"created": dt.datetime.now().isoformat(timespec="seconds"),
                                "port": port, "settings": settings}, indent=2) + "\n")
    return path


def cmd_backup(a: argparse.Namespace) -> int:
    head("Backup $$ settings")
    port = pick_port("esp32", a.port, "ESP32-S3 native USB")
    b = open_board(port)
    if not b:
        return EXIT_FAIL
    try:
        seen, _ = b.wait_status(6.0)
        if not seen:
            err("No status report from the board; wrong port?")
            return EXIT_FAIL
        b.drain(0.3)
        lines, _ = b.command("$$", timeout=8.0)
        settings = parse_settings(lines)
        if not settings:
            err("No settings received.")
            return EXIT_FAIL
        ok(f"Saved {len(settings)} settings to {write_backup(settings, port)}")
        return EXIT_OK
    except serial.SerialException as e:
        err(f"Serial error: {e}")
        return EXIT_FAIL
    finally:
        b.close()


def cmd_restore(a: argparse.Namespace) -> int:
    head("Restore settings")
    try:
        data = json.loads(Path(a.file).read_text())
        settings: dict[str, str] = data["settings"]
    except (OSError, ValueError, KeyError) as e:
        err(f"Cannot read backup {a.file}: {e}")
        return EXIT_FAIL
    port = pick_port("esp32", a.port, "ESP32-S3 native USB")
    if not ask_yes(f"Write {len(settings)} settings from {a.file} to {port}?"):
        raise Abort("Aborted.")
    b = open_board(port)
    if not b:
        return EXIT_FAIL
    bad = 0
    try:
        seen, _ = b.wait_status(6.0)
        if not seen:
            err("No status report from the board; wrong port?")
            return EXIT_FAIL
        b.drain(0.3)
        for num, val in sorted(settings.items(), key=lambda kv: int(kv[0])):
            _, term = b.command(f"${num}={val}")
            if term != "ok":
                bad += 1
                warn(f"${num}={val} -> {term or 'timeout'}")
        (ok if not bad else err)(f"Restore done: {len(settings) - bad} ok, {bad} error(s)")
        return EXIT_OK if not bad else EXIT_FAIL
    except serial.SerialException as e:
        err(f"Serial error: {e}")
        return EXIT_FAIL
    finally:
        b.close()


# ------------------------------------------------------------ wizard -------
def wait_for_esp_port(timeout: float = 15.0) -> str | None:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        for p in scan_ports():
            if p.kind == "esp32":
                return p.device
        time.sleep(0.5)
    return None


def cmd_all(a: argparse.Namespace) -> int:
    head("WaveMaster flashing wizard")
    if cmd_detect() != EXIT_OK and not ask_yes("No ports found. Continue anyway?"):
        return EXIT_FAIL
    if not find_pio():
        err("PlatformIO not found. Run: bash tools/setup_mac.sh")
        return EXIT_FAIL
    if cmd_build() != EXIT_OK:
        return EXIT_FAIL
    atm_port = pick_port("arduino", getattr(a, "atmega_port", None), "ATmega (Arduino) USB-serial")
    a.port = atm_port
    if cmd_flash_atmega(a) != EXIT_OK:
        return EXIT_FAIL
    head("Arduino USB")
    print("  The CH340 sits in parallel with the ESP32 link on the ATmega UART lines.\n"
          "  Unplug the Arduino's USB now unless you need USB to power the board.\n"
          "  Either way, close any serial monitor on it.")
    pause("Done?")
    a.port = getattr(a, "esp_port", None)
    a.erase = getattr(a, "erase", False)
    if cmd_flash_esp32(a) != EXIT_OK:
        return EXIT_FAIL
    info("Waiting for the ESP32-S3 to re-enumerate (up to 15 s)...")
    port = wait_for_esp_port(15.0)
    if not port:
        err("ESP32 port did not reappear. Replug USB (or tap RESET) and run: flash_boards.py verify")
        return EXIT_FAIL
    ok(f"ESP32 on {port}")
    time.sleep(1.5)
    a.port = port
    vrc = cmd_verify(a)
    head("Next steps")
    print("  1. Check wiring per docs/WIRING.md (DB25 pins 19/20 -> GPIO4/GPIO3) with the laser OFF.\n"
          "  2. Follow the RF / scope plans in tools/rftest/ (guide plans).\n"
          "  3. Calibrate scale/offset/delays ($100/$101, $110/$111, $130-$134), then `backup`.")
    return vrc


# ------------------------------------------------------------ selftest -----
def cmd_selftest(_: argparse.Namespace | None = None) -> int:
    fails: list[str] = []

    def t(name: str, cond: bool) -> None:
        print(f"  {'ok  ' if cond else 'FAIL'} {name}")
        if not cond:
            fails.append(name)

    atm_ok = ["[MSG:atmega link=1 armed=0 ready=0 guide=0 power=0 stat=0x00 vdet=4980mV err=0]"]
    atm_bad = ["[MSG:atmega link=0 armed=0 ready=0 guide=0 power=0 stat=0x00 vdet=0mV err=3]"]
    galvo = ["[MSG:galvo chunks=0 underruns=0 barriers=0 ticks=0 tick_us=10.00]"]
    rb_ok = ["[MSG:RB X=0x8000 Y=0x8000]"]
    rb_bad = ["[MSG:RB unavailable (busy or SPI error)]"]
    inf = ["[VER:1.1h.WaveMaster:]", "[OPT:V,512,1024]", "[MSG:machine:WaveMaster Galvo]"]

    a = parse_atmega(atm_ok)
    t("atmega ok", a is not None and a["link"] == 1 and a["vdet_mv"] == 4980 and a["err"] == 0)
    a2 = parse_atmega(atm_bad)
    t("atmega link=0", a2 is not None and a2["link"] == 0 and a2["err"] == 3)
    t("atmega absent", parse_atmega(["junk"]) is None)
    g = parse_galvo(galvo)
    t("galvo", g is not None and g["tick_us"] == 10.0 and g["underruns"] == 0)
    t("galvo fault", parse_galvo(["[MSG:motion hardware fault]"]) == {"fault": True})
    t("rb ok", parse_rb(rb_ok) == {"x": 0x8000, "y": 0x8000})
    t("rb unavailable", parse_rb(rb_bad) == {"unavailable": True})
    t("rb absent", parse_rb([]) is None)
    i = parse_info(inf)
    t("info", i["version"] == "1.1h.WaveMaster:" and i["rx"] == 1024 and i["queue"] == 512)
    t("status", parse_status("<Idle|MPos:0.000,0.000,0.000|FS:0,0>") and not parse_status("ok"))
    t("settings", parse_settings(["$100=0.100", "$3=0", "junk", "$12=0.010"]) ==
      {"100": "0.100", "3": "0", "12": "0.010"})
    t("boot ok", parse_boot_selftest(["I (900) dac: self-test OK: CH0=0x8000 CH1=0x8000"]) is True)
    t("boot bad", parse_boot_selftest(["E (900) dac: self-test MISMATCH: CH0=0x1"]) is False)
    t("boot none", parse_boot_selftest(["hello"]) is None)
    t("classify", classify(0x303A, 0x1001) == "esp32" and classify(0x1A86, 0x7523) == "arduino"
      and classify(0x0403, 0x6001) == "arduino" and classify(1, 2) == "unknown")
    good = evaluate(True, i, a, g, parse_rb(rb_ok), None)
    t("evaluate all-pass", all(c.status != "FAIL" for c in good))
    bad = evaluate(True, i, a2, g, parse_rb(rb_bad), None)
    names = {c.name for c in bad if c.status == "FAIL"}
    t("evaluate fails link+RB", names == {"ATmega link ($S)", "DAC readback ($RB)"})
    t("evaluate no status", evaluate(False, {}, None, None, None, None)[0].status == "FAIL")
    print(Ui.c("32", "selftest PASS") if not fails else Ui.c("31", f"selftest FAIL: {fails}"))
    return EXIT_OK if not fails else EXIT_FAIL


# ----------------------------------------------------------------- main ----
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="WaveMaster flashing wizard (ATmega328P + ESP32-S3)")
    p.add_argument("--no-color", action="store_true", help="disable ANSI colors")
    sub = p.add_subparsers(dest="cmd")
    sub.add_parser("detect", help="list serial ports and classify them")
    sub.add_parser("build", help="build both firmwares")
    s = sub.add_parser("flash-atmega", help="flash the ATmega328P companion (do this first)")
    s.add_argument("--port")
    s.add_argument("-y", "--yes", action="store_true", help="skip the safety prompt (not recommended)")
    s = sub.add_parser("flash-esp32", help="flash the ESP32-S3")
    s.add_argument("--port")
    s.add_argument("--erase", action="store_true", help="erase flash first (wipes NVS settings)")
    s = sub.add_parser("verify", help="check the running ESP32 firmware and ATmega link")
    s.add_argument("--port")
    s = sub.add_parser("backup", help="save $$ settings to tools/backups/")
    s.add_argument("--port")
    s = sub.add_parser("restore", help="write settings from a backup JSON")
    s.add_argument("file")
    s.add_argument("--port")
    s = sub.add_parser("all", help="full wizard (default)")
    s.add_argument("--atmega-port")
    s.add_argument("--esp-port")
    s.add_argument("--erase", action="store_true")
    s.add_argument("-y", "--yes", action="store_true")
    sub.add_parser("selftest", help="run built-in parser self-test")
    return p


def main(argv: list[str] | None = None) -> int:
    ap = build_parser()
    a = ap.parse_args(argv)
    Ui.color = not a.no_color and sys.stdout.isatty() and "NO_COLOR" not in os.environ
    cmd = a.cmd or "all"
    if a.cmd is None:
        a = ap.parse_args(["all"] + (["--no-color"] if a.no_color else []))
    table = {"detect": cmd_detect, "build": cmd_build, "flash-atmega": cmd_flash_atmega,
             "flash-esp32": cmd_flash_esp32, "verify": cmd_verify, "backup": cmd_backup,
             "restore": cmd_restore, "all": cmd_all, "selftest": cmd_selftest}
    try:
        return table[cmd](a)
    except Abort as e:
        err(str(e))
        return EXIT_ABORT if "Aborted" in str(e) else EXIT_USAGE
    except (KeyboardInterrupt, EOFError):
        print()
        err("Interrupted.")
        return EXIT_ABORT
    except (OSError, serial.SerialException) as e:
        err(f"I/O error: {e}")
        return EXIT_FAIL


if __name__ == "__main__":
    sys.exit(main())
