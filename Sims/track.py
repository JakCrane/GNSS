#!/usr/bin/env python3
"""Capture -> acquire -> put the best satellites on the FPGA's channels -> log tracking.

One command for field use. Everything is saved in a timestamped folder so it
can be post-processed later:

    runs/YYYYmmdd_HHMMSS/
        capture.bin            raw 2-bit samples (same format as capture.py)
        acquisition.txt        PRN / ratio / Doppler / code phase table
        search.txt             code-phase search result for each PRN tried
        channels.csv           which PRN ended up on which channel (+ Doppler, search ratio)
        track.csv              one row per status line from the board (10 Hz per channel)
        prompts_ch<n>.bin      every 1 ms dump of channel n while only the DLL runs:
                               19-byte records (6 x int24 LE: ie qe ip qp il ql, then
                               the 8-bit dump counter). Read with load_prompts().
        prompts_pll_ch<n>.bin  same, for the part of the run with the carrier loop on
        timing.txt             the board's "loop:" timing reports

After acquisition it fills channels 0..N-1 in turn: for each, it points the
channel at the next-strongest acquired PRN ("ch"), runs the board's
code-phase search ("srch"), and starts the DLL ("trk") straight away if the
search peak clearly beats the noise; otherwise it tries the next PRN on that
channel. After --dll-secs it closes the carrier loop on every channel
("trk <n> pll").

Usage:
    python3 track.py [port] [--prn N [N ...]] [--doppler HZ] [--nch N] [--secs S]
                     [--out DIR] [--dll-secs S] [--no-pll] [--flip-csign]

    --prn N ...   use these PRNs (in this order) instead of the strongest found
    --doppler HZ  Doppler for any --prn that acquisition didn't find
    --min-ratio R search ratio needed to start tracking (default 8; noise gives ~4-5)
    --nch N       channels to use (default: what the board reports, "nch:")
    --secs S      stop logging after S seconds (default: until Ctrl-C)
    --out DIR     parent folder for runs (default: runs)
    --dll-secs S  DLL-only for S seconds first (recording for pll.py), then switch
                  every channel to DLL+PLL. Default 300; 0 = PLL at once
    --no-pll      DLL only for the whole run (carrier loop never closed)
    --flip-csign  flip the carrier loop's sign first (if the FLL ran away last time)

Default (field run): python3 track.py
    = capture, acquire, up to 4 satellites searched + tracked, 5 min DLL-only
      recording, then live DLL+PLL on all of them until Ctrl-C.
    If the PLL runs away:  python3 track.py --dll-secs 10 --flip-csign
"""
import argparse
import csv
import os
import re
import sys
import time

import numpy as np
import serial  # pip install pyserial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from acquire import (FS, F_IF, SAMPLES_PER_MS, CODE_LEN, DOPPLER_MAX,   # noqa: E402
                     DOPPLER_STEP, THRESHOLD, ca_code, load, acquire)

SEARCH_TIMEOUT = 60      # s; a full search takes ~10 s
SEARCH_MIN_RATIO = 8.0   # noise alone gives ~4-5

# Status line, channel-tagged:
#   "c2 E   123,   -45  P  6789,  -12  L   101,   22  d=17"
#   plus, while the loops are closed: "  dll=-0.012 corr=-0.0240"
#   plus, with the carrier loop on:    " f=+1138.52 lock=+0.97"
STATUS_RE = re.compile(
    r"^c(\d)\s+E\s*(-?\d+),\s*(-?\d+)\s+P\s*(-?\d+),\s*(-?\d+)\s+L\s*(-?\d+),\s*(-?\d+)\s+d=(\d+)"
    r"(?:\s+dll=([-+]?[\d.]+)\s+corr=([-+]?[\d.]+))?"
    r"(?:\s+f=([-+]?[\d.]+)\s+lock=([-+]?[\d.]+))?")
STREAM_RE = re.compile(r"^S(\d) ([0-9a-f]+)$")
TRK_RE = re.compile(r"^trk: n=(\d) on \((dll(?:\+pll)?)\)")


# ---------------------------------------------------------------- capture
def capture(s: serial.Serial) -> bytes:
    """Ask the board for a capture on an already-open port and return the bytes."""
    s.reset_input_buffer()
    s.write(b"cap\r\n")
    data = bytearray()
    started = False
    deadline = time.time() + 20
    while time.time() < deadline:
        line = s.readline().decode(errors="ignore").strip()
        if not line:
            continue
        if line == "capture: failed":
            sys.exit("firmware reported capture: failed")
        if line == "capture: begin":
            started = True
        elif line == "capture: end":
            return bytes(data)
        elif started:
            data += bytes.fromhex(line)
    sys.exit("capture timed out (no 'capture: end')")


# ---------------------------------------------------------------- acquisition
def refine_doppler(x: np.ndarray, prn: int, fd: float, code_phase_chips: float) -> float:
    """Sharpen a coarse Doppler estimate using the carrier phase change from one
    1 ms block to the next. Squaring removes the 180 deg data-bit flips, so the
    correction is unambiguous within +-250 Hz (half a 500 Hz bin)."""
    spc = SAMPLES_PER_MS // CODE_LEN
    n_ms = len(x) // SAMPLES_PER_MS
    if n_ms < 2:
        return fd
    n = n_ms * SAMPLES_PER_MS
    t = np.arange(n) / FS                                   # continuous time, no restart
    carrier = np.exp(-2j * np.pi * (F_IF + fd) * t)
    code = np.repeat(ca_code(prn), spc)
    code = np.roll(code, int(round(code_phase_chips * spc)))  # line up with the signal
    z = (x[:n] * carrier).reshape(n_ms, SAMPLES_PER_MS) @ code  # prompt, one per ms
    z2 = z ** 2
    dphi = np.angle(np.sum(z2[1:] * np.conj(z2[:-1]))) / 2   # rad per ms
    return fd + dphi / (2 * np.pi * 1e-3)


def acquire_all(path: str, report_path: str):
    x = load(path)
    x -= x.mean()
    dopplers = np.arange(-DOPPLER_MAX, DOPPLER_MAX + 1, DOPPLER_STEP)

    rows = []
    for prn in range(1, 33):
        ratio, fd, phase, _ = acquire(x, prn, dopplers)
        rows.append((prn, ratio, fd, phase))

    acquired = [r for r in rows if r[1] > THRESHOLD]
    refined = {prn: refine_doppler(x, prn, fd, ph) for prn, _, fd, ph in acquired}

    lines = [" PRN   ratio   Doppler(Hz)   refined(Hz)   code phase(chips)"]
    for prn, ratio, fd, phase in rows:
        ref = f"{refined[prn]:+11.0f}" if prn in refined else " " * 11
        mark = "  <-- ACQUIRED" if prn in refined else ""
        lines.append(f" {prn:3d}   {ratio:5.2f}   {fd:+9.0f}     {ref}     {phase:8.1f}{mark}")
    text = "\n".join(lines)
    print(text)
    with open(report_path, "w") as f:
        f.write(text + "\n")

    acquired.sort(key=lambda r: r[1], reverse=True)          # strongest first
    return [(prn, refined[prn], ratio) for prn, ratio, _, _ in acquired]


# ---------------------------------------------------------------- 1 ms stream
RECORD_BYTES = 19        # same layout as registers 0x20..0x32: 6 x int24 LE + dump count


class StreamStats:
    """Counts records and dumps the board skipped (from the 8-bit dump counter)."""

    def __init__(self):
        self.records = 0
        self.missed = 0
        self.gaps = 0
        self.bad_lines = 0
        self._last = None

    def add(self, raw: bytes):
        for k in range(0, len(raw), RECORD_BYTES):
            d = raw[k + RECORD_BYTES - 1]
            if self._last is not None:
                step = (d - self._last) % 256
                if step != 1:
                    self.gaps += 1
                    # a step of 0 can't happen for a real gap; treat as a full wrap
                    self.missed += (step - 1) if step else 255
            self._last = d
            self.records += 1

    def summary(self) -> str:
        return (f"{self.records} records ({self.records / 1000:.1f} s), "
                f"{self.missed} dumps missed in {self.gaps} gaps, "
                f"{self.bad_lines} bad lines")


def load_prompts(path: str) -> dict:
    """Read a prompts*.bin into numpy arrays: ie qe ip qp il ql (int32) and dumps (uint8).
    For offline processing (pll.py)."""
    raw = np.fromfile(path, dtype=np.uint8)
    raw = raw[: len(raw) // RECORD_BYTES * RECORD_BYTES].reshape(-1, RECORD_BYTES)

    def i24(col):
        b = raw[:, col:col + 3].astype(np.int32)
        v = b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16)
        return np.where(v & 0x800000, v - (1 << 24), v)

    names = ["ie", "qe", "ip", "qp", "il", "ql"]
    out = {n: i24(3 * k) for k, n in enumerate(names)}
    out["dumps"] = raw[:, 18].copy()
    return out


class Segment:
    """One channel's 1 ms stream for one mode, in its own file."""

    def __init__(self, path: str):
        self.path = path
        self.f = None                      # opened on first data, so unused modes leave no file
        self.stats = StreamStats()

    def add_hex(self, hexpart: str):
        try:
            raw = bytes.fromhex(hexpart)
        except ValueError:
            self.stats.bad_lines += 1
            return
        if len(raw) % RECORD_BYTES:
            self.stats.bad_lines += 1      # truncated line: drop it, the gap shows up
            return
        if self.f is None:
            self.f = open(self.path, "wb")
        self.f.write(raw)
        self.f.flush()                     # survive a yanked cable
        self.stats.add(raw)

    def close(self):
        if self.f:
            self.f.close()


# ---------------------------------------------------------------- the logger
class Logger:
    """Everything the board sends goes through handle(): stream lines, status lines
    and mode changes are recorded whenever they arrive, including while we're
    waiting for a command's reply during channel setup."""

    def __init__(self, run_dir: str):
        self.run_dir = run_dir
        self.t0 = time.time()
        self.chan = {}                     # ch -> dict(prn, doppler, mode, last status)
        self.segs = {}                     # (ch, mode) -> Segment
        self.csv_f = open(os.path.join(run_dir, "track.csv"), "w", newline="")
        self.csv = csv.writer(self.csv_f)
        self.csv.writerow(["t_s", "unix_time", "ch", "prn", "doppler_hz",
                           "ie", "qe", "ip", "qp", "il", "ql", "dumps",
                           "dll_disc", "code_corr_chips_s",          # blank when not tracking
                           "mode", "carrier_doppler_hz", "pll_lock"])  # last two: PLL on
        self.timing_f = open(os.path.join(run_dir, "timing.txt"), "w")
        self.last_print = 0.0

    def set_channel(self, ch: int, prn: int, doppler: float):
        self.chan[ch] = {"prn": prn, "doppler": doppler, "mode": "open", "last": None}

    def _seg(self, ch: int, mode: str) -> Segment:
        key = (ch, mode)
        if key not in self.segs:
            name = f"prompts_ch{ch}.bin" if mode == "dll" else f"prompts_pll_ch{ch}.bin"
            self.segs[key] = Segment(os.path.join(self.run_dir, name))
        return self.segs[key]

    def handle(self, line: str) -> str:
        m = STREAM_RE.match(line)
        if m:
            ch = int(m.group(1))
            mode = self.chan.get(ch, {}).get("mode", "open")
            if mode in ("dll", "pll"):
                self._seg(ch, mode).add_hex(m.group(2))
            return line
        m = STATUS_RE.match(line)
        if m:
            ch = int(m.group(1))
            ie, qe, ip, qp, il, ql, d = map(int, m.groups()[1:8])
            dll, corr, fhz, lock = (g or "" for g in m.groups()[8:12])
            c = self.chan.get(ch, {"prn": "", "doppler": "", "mode": "open", "last": None})
            now = time.time()
            self.csv.writerow([f"{now - self.t0:.3f}", f"{now:.3f}", ch, c["prn"], c["doppler"],
                               ie, qe, ip, qp, il, ql, d, dll, corr, c["mode"], fhz, lock])
            self.csv_f.flush()
            if ch in self.chan:
                self.chan[ch]["last"] = (np.hypot(ie, qe), np.hypot(ip, qp), np.hypot(il, ql),
                                         dll, fhz, lock)
            return line
        m = TRK_RE.match(line)
        if m:
            ch = int(m.group(1))
            if ch in self.chan:
                self.chan[ch]["mode"] = "pll" if m.group(2) == "dll+pll" else "dll"
            print(line)
            return line
        if line.startswith("loop:"):
            self.timing_f.write(f"{time.time() - self.t0:8.1f}  {line}\n")
            self.timing_f.flush()
        return line

    def print_row(self):
        """One line per second: every tracked channel's E/P/L and loop state."""
        now = time.time()
        if now - self.last_print < 1.0:
            return
        self.last_print = now
        parts = []
        for ch in sorted(self.chan):
            c = self.chan[ch]
            if c["last"] is None or c["mode"] == "open":
                continue
            e, p, l, dll, fhz, lock = c["last"]
            s = f"ch{ch} PRN{c['prn']:>2} P{p:6.0f} E{e:5.0f} L{l:5.0f} dll{dll:>7}"
            if c["mode"] == "pll":
                s += f" f{fhz:>8} lock{lock:>6}"
            parts.append(s)
        if parts:
            print(f"{now - self.t0:6.0f}s  " + "  |  ".join(parts))

    def close(self):
        self.csv_f.close()
        self.timing_f.close()
        print()
        for (ch, mode), seg in sorted(self.segs.items()):
            seg.close()
            print(f"1 ms stream ch{ch} {mode:3s} ({os.path.basename(seg.path)}): {seg.stats.summary()}")


# ---------------------------------------------------------------- board commands
def read_line(s: serial.Serial, log: Logger) -> str:
    return log.handle(s.readline().decode(errors="ignore").strip())


def wait_for(s: serial.Serial, log: Logger, prefix: str, timeout: float):
    """Read (and log) lines until one starts with `prefix`; return it, or None on timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = read_line(s, log)
        if line.startswith(prefix):
            return line
    return None


def point(s, log, ch: int, prn: int, doppler: float) -> bool:
    s.write(f"ch {ch} {prn} {doppler:.1f}\r\n".encode())
    line = wait_for(s, log, "ch:", 3)
    if line:
        log.set_channel(ch, prn, doppler)
        print(line)
    return line is not None


def search(s, log, ch: int):
    """Code-phase search on one channel. Returns (ratio, raw line) or (None, None)."""
    s.write(f"srch {ch}\r\n".encode())
    print(f"  searching code phase on ch{ch} (~10 s; other channels coast meanwhile)...")
    line = wait_for(s, log, "search:", SEARCH_TIMEOUT)
    if line is None:
        return None, None
    print("  " + line)
    m = re.search(r"ratio\s+([\d.]+)", line)
    return (float(m.group(1)) if m else None), line


def start_tracking(s, log, ch: int, pll: bool) -> bool:
    s.write(f"trk {ch}{' pll' if pll else ''}\r\n".encode())
    return wait_for(s, log, "trk:", 3) is not None   # Logger prints it and sets the mode


def read_nch(s) -> int:
    """The board prints "nch: N" when the port opens."""
    deadline = time.time() + 2
    while time.time() < deadline:
        line = s.readline().decode(errors="ignore").strip()
        m = re.match(r"nch:\s*(\d+)", line)
        if m:
            return int(m.group(1))
    return 0


# ---------------------------------------------------------------- main
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("--prn", type=int, nargs="+")
    ap.add_argument("--doppler", type=float)
    ap.add_argument("--min-ratio", type=float, default=SEARCH_MIN_RATIO,
                    help="search ratio needed to start tracking (0 = always; for bench tests)")
    ap.add_argument("--nch", type=int)
    ap.add_argument("--secs", type=float)
    ap.add_argument("--out", default="runs")
    ap.add_argument("--pll", action=argparse.BooleanOptionalAction, default=True,
                    help="close the carrier loop after the DLL-only stage (default on; "
                         "--no-pll = DLL only for the whole run)")
    ap.add_argument("--dll-secs", type=float, default=300.0,
                    help="DLL-only recording before switching to DLL+PLL, s (default 300; "
                         "0 = PLL straight away)")
    ap.add_argument("--flip-csign", action="store_true",
                    help="flip the carrier loop sign before tracking (if the FLL runs away)")
    args = ap.parse_args()
    if args.doppler is not None and not args.prn:
        ap.error("--doppler needs --prn")

    run_dir = os.path.join(args.out, time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    cap_path = os.path.join(run_dir, "capture.bin")

    with serial.Serial(args.port, timeout=1) as s:
        s.dtr = True
        board_nch = read_nch(s)
        nch = args.nch or board_nch or 4
        print(f"board reports nch: {board_nch or '?'}; using {nch} channel(s)")

        data = capture(s)
        with open(cap_path, "wb") as f:
            f.write(data)
        print(f"saved {len(data)} bytes to {cap_path}")

        print("acquiring (all 32 PRNs)...")
        found = acquire_all(cap_path, os.path.join(run_dir, "acquisition.txt"))

        # Candidates, best first.
        if args.prn:
            by_prn = {p: d for p, d, _ in found}
            candidates = []
            for p in args.prn:
                if p in by_prn:
                    candidates.append((p, by_prn[p]))
                elif args.doppler is not None:
                    candidates.append((p, args.doppler))
                else:
                    print(f"PRN {p} not acquired; skipping (give --doppler to force it)")
        else:
            candidates = [(p, d) for p, d, _ in found]
        if not candidates:
            sys.exit("no satellites to track - capture saved, try again with a clearer sky view")

        log = Logger(run_dir)
        if args.flip_csign:
            s.write(b"csign\r\n")
            print(wait_for(s, log, "csign:", 3) or "no 'csign:' reply")

        # Fill channels 0..nch-1: point, search, and track straight away on a good peak.
        tracked = []                                           # (ch, prn, doppler, ratio)
        first_pll = args.pll and not args.dll_secs
        with open(os.path.join(run_dir, "search.txt"), "w") as slog:
            queue = list(candidates)
            for ch in range(nch):
                while queue:
                    prn, doppler = queue.pop(0)
                    print(f"\nch{ch}: PRN {prn} at {doppler:+.1f} Hz")
                    if not point(s, log, ch, prn, doppler):
                        sys.exit("board never echoed 'ch:' - is the firmware up to date?")
                    ratio, line = search(s, log, ch)
                    slog.write(f"ch{ch} PRN {prn} doppler {doppler:+.1f}: {line or 'no reply'}\n")
                    slog.flush()
                    if ratio is None:
                        sys.exit("no 'search:' reply - is the firmware up to date?")
                    if ratio < args.min_ratio:
                        print(f"  ratio {ratio:.1f} < {args.min_ratio}: no clear peak, next PRN")
                        continue
                    if not start_tracking(s, log, ch, first_pll):
                        sys.exit("board never echoed 'trk:' - is the firmware up to date?")
                    tracked.append((ch, prn, doppler, ratio))
                    break
                if not queue:
                    break                                      # out of candidates

        with open(os.path.join(run_dir, "channels.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["ch", "prn", "doppler_hz", "search_ratio"])
            for row in tracked:
                w.writerow([row[0], row[1], f"{row[2]:.1f}", f"{row[3]:.1f}"])

        if not tracked:
            log.close()
            sys.exit("no channel got a clear code peak - capture and searches saved")

        print(f"\ntracking {len(tracked)} channel(s): "
              + ", ".join(f"ch{c}=PRN{p}" for c, p, _, _ in tracked))
        switch_at = args.dll_secs if (args.pll and args.dll_secs) else None
        t_log = time.time()
        stop = "Ctrl-C to stop" if args.secs is None else f"{args.secs:.0f} s, or Ctrl-C"
        print(f"logging ({stop})"
              + (f"; switching to DLL+PLL after {switch_at:.0f} s" if switch_at else ""))
        try:
            while args.secs is None or time.time() - t_log < args.secs:
                if switch_at is not None and time.time() - t_log >= switch_at:
                    for ch, *_ in tracked:
                        s.write(f"trk {ch} pll\r\n".encode())   # mode flips when the board confirms
                    switch_at = None
                read_line(s, log)
                log.print_row()
        except KeyboardInterrupt:
            pass
        log.close()

    print(f"\nall data in {run_dir}/")


if __name__ == "__main__":
    main()
