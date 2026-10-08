"""
RayforgeClient - a GRBL sender that behaves like Rayforge (raydriver 0.3.0).

Mimicked behaviour:
  * connect: bare "?" every 0.5 s for up to 6 s until a "<...>" status or a
    "Grbl " banner arrives, then "$I\\n" collecting [VER:] [OPT:] [MSG:machine:]
    until "ok".  RX window = third field of [OPT:], default 127.
  * streaming: GRBL character counting.  Lines are comment-stripped (";..." and
    "(...)"), trimmed, empty ones skipped, sent as "line\\n".  A FIFO of line
    lengths (incl. "\\n") is kept; a line is sent only if in-flight + len <= RX
    window; every "ok" / "error:N" pops the oldest.  "error:" or "ALARM:" during
    a job aborts it (cancel sequence).  Stall handling: after `stall_timeout`
    seconds without progress send "?"; keep waiting while the device answers,
    give up after 3 unanswered polls.
  * no "?" polling during jobs by default; optional (poll_ms) for the tests.
  * cancel: raw 0x18, wait 0.2 s, then "M5\\n" and "M9\\n" (each waits for ok).
  * hold / resume: raw "!" / "~".
  * jog: "$J=G91 G21 F<f> X.. Y.." and "$J=G90 G21 F<f> X.. Y..".

Every byte direction is recorded by SessionLogger (session.log, events.jsonl,
firmware_msgs.log, statuses.csv).  Only dependency: pyserial.
"""
from __future__ import annotations

import json
import queue
import re
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional, Sequence

from gcode_gen import fmt_num

REALTIME_NAMES = {0x18: "reset", 0x3F: "status", 0x21: "hold", 0x7E: "resume", 0x85: "jog_cancel"}
STATUS_RE = re.compile(r"<([A-Za-z]+)(?::(\d+))?((?:\|[^|>]*)*)>")
OPT_RE = re.compile(r"\[OPT:([^,\]]*),(\d+)(?:,(\d+))?")


class RfError(Exception):
    pass


class StallError(RfError):
    pass


def strip_comment(line: str) -> str:
    """Rayforge/raydriver comment stripping: ';...' and '(...)', then trim."""
    line = line.split(";", 1)[0]
    line = re.sub(r"\([^)]*\)", "", line)
    return line.strip()


# ------------------------------------------------------------- status ------

@dataclass
class Status:
    t: float
    state: str
    sub: Optional[int] = None
    mpos: Optional[tuple[float, ...]] = None
    wpos: Optional[tuple[float, ...]] = None
    wco: Optional[tuple[float, ...]] = None
    feed: Optional[float] = None
    spindle: Optional[float] = None
    bf: Optional[tuple[int, int]] = None      # (planner blocks free, rx bytes free)
    raw: str = ""

    @property
    def xy(self) -> Optional[tuple[float, float]]:
        p = self.mpos or self.wpos
        return (p[0], p[1]) if p and len(p) >= 2 else None


def _floats(s: str) -> tuple[float, ...]:
    return tuple(float(v) for v in s.split(","))


def parse_status(text: str, t: float = 0.0) -> Optional[Status]:
    m = STATUS_RE.search(text)
    if not m:
        return None
    st = Status(t=t, state=m.group(1), sub=int(m.group(2)) if m.group(2) else None, raw=m.group(0))
    for field_ in filter(None, m.group(3).split("|")):
        key, _, val = field_.partition(":")
        try:
            if key == "MPos":
                st.mpos = _floats(val)
            elif key == "WPos":
                st.wpos = _floats(val)
            elif key == "WCO":
                st.wco = _floats(val)
            elif key == "FS":
                f = _floats(val)
                st.feed, st.spindle = f[0], (f[1] if len(f) > 1 else None)
            elif key == "Bf":
                a, b = val.split(",")[:2]
                st.bf = (int(a), int(b))
        except ValueError:
            pass
    if st.mpos is None and st.wpos is not None and st.wco is not None:
        st.mpos = tuple(w + c for w, c in zip(st.wpos, st.wco))
    return st


# ------------------------------------------------------------- logger ------

class SessionLogger:
    """Records every byte direction with monotonic timestamps.

    session.log        human readable transcript ("TX"/"RX", realtime as <0x18>)
    events.jsonl       structured events
    firmware_msgs.log  every [MSG:...] line
    statuses.csv       t,state,x,y,feed,spindle,bf_blocks,bf_rx
    """

    def __init__(self, out_dir: Optional[Path], verbose: bool = False):
        self.verbose = verbose
        self.t0 = time.monotonic()
        self._lock = threading.Lock()
        self._files: dict[str, object] = {}
        self.statuses: list[Status] = []
        self.out_dir = Path(out_dir) if out_dir else None
        self._last_flush = 0.0
        if self.out_dir:
            self.out_dir.mkdir(parents=True, exist_ok=True)
            for name in ("session.log", "events.jsonl", "firmware_msgs.log", "statuses.csv"):
                self._files[name] = open(self.out_dir / name, "w", encoding="utf-8")
            self._files["statuses.csv"].write("t,state,x,y,feed,spindle,bf_blocks,bf_rx\n")

    def now(self) -> float:
        return time.monotonic() - self.t0

    def _write(self, name: str, text: str) -> None:
        f = self._files.get(name)
        if f:
            f.write(text)

    def _maybe_flush(self, t: float) -> None:
        if t - self._last_flush > 0.25:
            self._last_flush = t
            for f in self._files.values():
                f.flush()

    def _emit(self, direction: str, shown: str, **event) -> float:
        with self._lock:
            t = self.now()
            self._write("session.log", f"[{t:10.4f}] {direction}  {shown}\n")
            if event:
                self._write("events.jsonl", json.dumps({"t": round(t, 6), **event}) + "\n")
            self._maybe_flush(t)
        if self.verbose:
            print(f"[{t:9.3f}] {direction} {shown}")
        return t

    def _event(self, kind: str, **kw) -> None:
        with self._lock:
            t = self.now()
            self._write("events.jsonl", json.dumps({"t": round(t, 6), "kind": kind, **kw}) + "\n")

    # ---- API
    def tx_line(self, text: str) -> float:
        return self._emit("TX", text, kind="tx_line", line=text, bytes=len(text) + 1)

    def tx_realtime(self, byte: int) -> float:
        name = REALTIME_NAMES.get(byte, "?")
        label = chr(byte) if 0x20 < byte < 0x7F else f"<0x{byte:02X}>"
        return self._emit("TX", label, kind="realtime", byte=f"0x{byte:02X}", name=name)

    def rx_line(self, text: str) -> float:
        return self._emit("RX", text, kind="rx_line", line=text)

    def rx_partial(self, text: str) -> None:
        self._emit("RX", f"{text}  (partial, no newline)", kind="rx_partial", line=text)

    def ack(self) -> None:
        self._event("ack")

    def error(self, text: str) -> None:
        self._event("error", line=text)

    def msg(self, text: str) -> None:
        with self._lock:
            self._write("firmware_msgs.log", f"[{self.now():10.4f}] {text}\n")
        self._event("msg", text=text)

    def status(self, st: Status) -> None:
        with self._lock:
            self.statuses.append(st)
            xy = st.xy or ("", "")
            bf = st.bf or ("", "")
            self._write("statuses.csv",
                        f"{st.t:.4f},{st.state},{xy[0]},{xy[1]},{st.feed if st.feed is not None else ''},"
                        f"{st.spindle if st.spindle is not None else ''},{bf[0]},{bf[1]}\n")
        self._event("status", state=st.state, mpos=st.mpos, feed=st.feed, spindle=st.spindle, bf=st.bf)

    def note(self, text: str, **kw) -> None:
        self._emit("##", text, kind="note", text=text, **kw)

    def close(self) -> None:
        with self._lock:
            for f in self._files.values():
                f.flush()
                f.close()
            self._files = {}


# ------------------------------------------------------------- results -----

@dataclass
class DeviceInfo:
    version: str = ""
    options: str = ""
    planner_blocks: Optional[int] = None
    rx_window: int = 127
    machine: str = ""
    raw: list[str] = field(default_factory=list)


@dataclass
class LineError:
    index: int
    line: str
    reply: str
    t: float


@dataclass
class StreamResult:
    lines_total: int = 0
    lines_sent: int = 0
    acks: int = 0
    bytes_sent: int = 0
    max_inflight: int = 0
    duration: float = 0.0
    t_start: float = 0.0
    t_end: float = 0.0
    last_ack_t: Optional[float] = None
    errors: list[LineError] = field(default_factory=list)
    aborted: bool = False
    abort_reason: str = ""
    stall_polls: int = 0

    @property
    def ok(self) -> bool:
        return not self.errors and not self.aborted and self.acks == self.lines_sent == self.lines_total

    @property
    def lines_per_s(self) -> float:
        return self.lines_sent / self.duration if self.duration > 0 else 0.0

    @property
    def bytes_per_s(self) -> float:
        return self.bytes_sent / self.duration if self.duration > 0 else 0.0


@dataclass
class CancelResult:
    t_tx: float = 0.0
    reset_latency: Optional[float] = None       # 0x18 -> banner (s)
    m5: list[str] = field(default_factory=list)
    m9: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return bool(self.m5 and self.m5[-1] == "ok" and self.m9 and self.m9[-1] == "ok")


# -------------------------------------------------------------- client -----

class RayforgeClient:
    def __init__(self, port: Optional[str] = None, logger: Optional[SessionLogger] = None, *,
                 baud: int = 115200, serial_obj=None, rx_window: int = 127,
                 stall_timeout: float = 30.0, poll_ms: float = 0.0):
        self.logger = logger or SessionLogger(None)
        self.rx_window = rx_window
        self.stall_timeout = stall_timeout
        self.poll_ms = poll_ms
        self.info = DeviceInfo(rx_window=rx_window)
        self._port, self._baud = port, baud
        self._ser = serial_obj
        self._wlock = threading.Lock()
        self._q: "queue.Queue[tuple[float, str]]" = queue.Queue()
        self._stop = threading.Event()
        self._reader: Optional[threading.Thread] = None
        self._poller: Optional[threading.Thread] = None
        self._poll_stop = threading.Event()
        self._cancel_flag = threading.Event()
        self._alive = threading.Event()
        self._banner = threading.Event()
        self.banner_t: Optional[float] = None
        self._status_cv = threading.Condition()
        self._status_seq = 0
        self.last_status: Optional[Status] = None
        self.dead: Optional[str] = None
        self.stale: list[str] = []

    # ---- lifecycle
    def open(self) -> "RayforgeClient":
        if self._ser is None:
            import serial
            self._ser = serial.Serial(self._port, self._baud, timeout=0.05, write_timeout=3.0)
        self.logger.note(f"opened {self._port or 'serial object'}")
        self._reader = threading.Thread(target=self._read_loop, name="rf-reader", daemon=True)
        self._reader.start()
        return self

    def close(self) -> None:
        self.stop_polling()
        self._stop.set()
        if self._reader:
            self._reader.join(timeout=1.0)
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self.logger.note("closed")

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # ---- raw IO
    def _write(self, data: bytes) -> None:
        with self._wlock:
            self._ser.write(data)

    def send_realtime(self, byte: int) -> float:
        t = self.logger.tx_realtime(byte)
        self._write(bytes([byte]))
        return t

    def send_line(self, text: str) -> float:
        t = self.logger.tx_line(text)
        self._write((text + "\n").encode())
        return t

    def _read_loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                data = self._ser.read(max(1, getattr(self._ser, "in_waiting", 0) or 1))
            except Exception as e:                 # port vanished
                self.dead = f"{type(e).__name__}: {e}"
                self.logger.note(f"read error: {self.dead}")
                return
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._on_line(raw.decode("utf-8", "replace").strip("\r"))
        if buf:
            self.logger.rx_partial(buf.decode("utf-8", "replace"))

    def _on_line(self, text: str) -> None:
        if not text.strip():
            return
        t = self.logger.rx_line(text)
        rest = text
        for m in STATUS_RE.finditer(text):
            st = parse_status(m.group(0), t)
            if st:
                self.last_status = st
                self.logger.status(st)
                self._alive.set()
                with self._status_cv:
                    self._status_seq += 1
                    self._status_cv.notify_all()
        rest = STATUS_RE.sub("", text).strip()
        if not rest:
            return
        if rest == "ok":
            self.logger.ack()
        elif rest.startswith("error:"):
            self.logger.error(rest)
        elif rest.startswith("ALARM:"):
            self.logger.error(rest)
        elif rest.startswith("[MSG:"):
            self.logger.msg(rest)
        elif rest.startswith("Grbl "):
            self.banner_t = t
            self._alive.set()
            self._banner.set()
        self._q.put((t, rest))

    def read_line(self, timeout: float = 0.05) -> Optional[tuple[float, str]]:
        try:
            return self._q.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain(self) -> list[str]:
        """Discard queued lines (they stay in the logs); returns them."""
        out = []
        while True:
            try:
                out.append(self._q.get_nowait()[1])
            except queue.Empty:
                break
        self.stale.extend(out)
        return out

    # ---- connect
    def handshake(self, timeout: float = 6.0) -> DeviceInfo:
        self._alive.clear()
        t_end = time.monotonic() + timeout
        while not self._alive.is_set():
            if time.monotonic() >= t_end:
                raise RfError(f"no GRBL status/banner within {timeout:.0f} s")
            self.send_realtime(ord("?"))
            self._alive.wait(0.5)
        time.sleep(0.05)
        reply = self.command("$I", timeout=5.0)
        info = DeviceInfo(rx_window=127, raw=reply)
        for line in reply:
            if line.startswith("[VER:"):
                info.version = line
            elif line.startswith("[OPT:"):
                info.options = line
                m = OPT_RE.match(line)
                if m:
                    if m.group(3):
                        info.rx_window = int(m.group(3))
                    info.planner_blocks = int(m.group(2))
            elif line.startswith("[MSG:machine:"):
                info.machine = line[len("[MSG:machine:"):-1]
        self.info = info
        self.rx_window = info.rx_window
        self.logger.note(f"handshake: window={info.rx_window} version={info.version} machine={info.machine}")
        return info

    # ---- commands
    def command(self, line: str, timeout: float = 5.0) -> list[str]:
        """Send one line, wait for ok / error; returns the reply lines (last = ok/error)."""
        self.drain()
        self.send_line(strip_comment(line))
        out: list[str] = []
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            item = self.read_line(min(0.05, max(0.0, end - time.monotonic())))
            if item is None:
                continue
            out.append(item[1])
            if item[1] == "ok" or item[1].startswith("error:"):
                return out
        raise TimeoutError(f"no reply to {line!r} within {timeout} s (got {out!r})")

    def command_ok(self, line: str, timeout: float = 5.0) -> list[str]:
        out = self.command(line, timeout)
        if out[-1] != "ok":
            raise RfError(f"{line!r} -> {out[-1]}")
        return out

    def status(self, timeout: float = 1.0) -> Optional[Status]:
        """Send "?" and wait for the next status report."""
        with self._status_cv:
            seq = self._status_seq
        self.send_realtime(ord("?"))
        end = time.monotonic() + timeout
        with self._status_cv:
            while self._status_seq == seq:
                left = end - time.monotonic()
                if left <= 0:
                    return None
                self._status_cv.wait(left)
        return self.last_status

    def wait_state(self, states: Sequence[str], timeout: float = 30.0,
                   interval: float = 0.05) -> tuple[Optional[Status], float]:
        """Poll "?" until the state is in `states`.  Returns (status or None, seconds waited)."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            st = self.status(0.5)
            if st and st.state in states:
                return st, time.monotonic() - t0
            time.sleep(interval)
        return None, time.monotonic() - t0

    def wait_idle(self, timeout: float = 30.0) -> tuple[Optional[Status], float]:
        return self.wait_state(("Idle",), timeout)

    def stats_s(self) -> dict[str, dict[str, str]]:
        """Run "$S" and parse the [MSG:...] key=value pairs per message kind."""
        out: dict[str, dict[str, str]] = {}
        for line in self.command("$S"):
            m = re.match(r"\[MSG:(\w+)\s+(.*)\]", line)
            if m:
                out[m.group(1)] = dict(re.findall(r"(\w+)=(\S+)", m.group(2)))
        return out

    # ---- realtime helpers
    def hold(self) -> None:
        self.send_realtime(ord("!"))

    def resume(self) -> None:
        self.send_realtime(ord("~"))

    def jog(self, dx: float, dy: float, speed: float) -> list[str]:
        return self.command(f"$J=G91 G21 F{fmt_num(speed)} X{fmt_num(dx)} Y{fmt_num(dy)}")

    def move_to(self, x: float, y: float, speed: float) -> list[str]:
        return self.command(f"$J=G90 G21 F{fmt_num(speed)} X{fmt_num(x)} Y{fmt_num(y)}")

    def cancel(self) -> CancelResult:
        """Rayforge cancel: 0x18, 0.2 s, then M5 and M9 as separate lines."""
        self._cancel_flag.set()
        self._banner.clear()
        res = CancelResult()
        res.t_tx = self.send_realtime(0x18)
        t0 = time.monotonic()
        if self._banner.wait(0.2) and self.banner_t is not None:
            res.reset_latency = self.banner_t - res.t_tx
        time.sleep(max(0.0, 0.2 - (time.monotonic() - t0)))
        for name, attr in (("M5", "m5"), ("M9", "m9")):
            try:
                setattr(res, attr, self.command(name, timeout=3.0))
            except TimeoutError as e:
                self.logger.note(f"cancel: {e}")
        return res

    # ---- polling during jobs
    def start_polling(self, interval_ms: float) -> None:
        if interval_ms <= 0 or self._poller:
            return
        self._poll_stop.clear()

        def run() -> None:
            while not self._poll_stop.wait(interval_ms / 1000.0):
                try:
                    self.send_realtime(ord("?"))
                except Exception:
                    return
        self._poller = threading.Thread(target=run, name="rf-poll", daemon=True)
        self._poller.start()

    def stop_polling(self) -> None:
        self._poll_stop.set()
        if self._poller:
            self._poller.join(timeout=1.0)
            self._poller = None

    # ---- streaming
    def stream(self, lines: Sequence[str], *, abort_on_error: bool = True,
               progress: Optional[Callable[[int, int], None]] = None) -> StreamResult:
        """GRBL character-counting stream (see module docstring)."""
        items = [(i, s) for i, s in enumerate(strip_comment(l) for l in lines) if s]
        res = StreamResult(lines_total=len(items))
        window = self.rx_window
        fifo: deque[tuple[int, int, str]] = deque()      # (item idx, bytes, text)
        inflight = 0
        nxt = 0
        unanswered = 0
        self._cancel_flag.clear()
        self.drain()
        self.logger.note(f"stream start: {len(items)} lines, window {window}, poll {self.poll_ms} ms")
        res.t_start = self.logger.now()
        t0 = time.monotonic()
        last_progress = t0
        self.start_polling(self.poll_ms)
        try:
            while nxt < len(items) or fifo:
                if self._cancel_flag.is_set():
                    res.aborted, res.abort_reason = True, "cancelled"
                    break
                while nxt < len(items):
                    text = items[nxt][1]
                    n = len(text) + 1
                    if n > window:
                        raise RfError(f"line of {n} bytes exceeds the RX window ({window}): {text[:40]!r}")
                    if inflight + n > window:
                        break
                    self.send_line(text)
                    fifo.append((nxt, n, text))
                    inflight += n
                    res.bytes_sent += n
                    res.lines_sent += 1
                    res.max_inflight = max(res.max_inflight, inflight)
                    nxt += 1
                    last_progress = time.monotonic()
                item = self.read_line(0.05)
                if item is None:
                    if fifo and time.monotonic() - last_progress > self.stall_timeout:
                        answered = self._stall_poll()
                        res.stall_polls += 1
                        unanswered = 0 if answered else unanswered + 1
                        last_progress = time.monotonic()
                        if unanswered >= 3:
                            res.aborted, res.abort_reason = True, "stalled: device not answering '?'"
                            break
                    continue
                t, text = item
                if text == "ok" or text.startswith("error:"):
                    if not fifo:
                        self.logger.note(f"unexpected reply with empty FIFO: {text}")
                        continue
                    idx, n, sent = fifo.popleft()
                    inflight -= n
                    res.acks += 1
                    res.last_ack_t = t
                    last_progress = time.monotonic()
                    if text != "ok":
                        res.errors.append(LineError(items[idx][0], sent, text, t))
                        if abort_on_error:
                            res.aborted, res.abort_reason = True, f"{text} on line {items[idx][0]}: {sent}"
                            break
                    if progress:
                        progress(res.acks, len(items))
                elif text.startswith("ALARM:"):
                    res.aborted, res.abort_reason = True, text
                    break
                elif text.startswith("Grbl "):
                    res.aborted, res.abort_reason = True, "device reset during job (banner)"
                    break
        finally:
            self.stop_polling()
            res.t_end = self.logger.now()
            res.duration = time.monotonic() - t0
        if res.aborted and res.abort_reason != "cancelled":
            self.logger.note(f"job aborted: {res.abort_reason}; sending cancel sequence")
            self.cancel()
        self.logger.note(f"stream end: sent {res.lines_sent}/{res.lines_total} acks {res.acks} "
                         f"errors {len(res.errors)} aborted {res.aborted}")
        return res

    def _stall_poll(self) -> bool:
        self.logger.note("stall timeout: polling '?'")
        return self.status(1.0) is not None
