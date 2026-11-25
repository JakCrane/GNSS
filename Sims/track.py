#!/usr/bin/env python3
"""Capture -> acquire -> point the FPGA channel at a satellite -> log tracking.

One command for field use. Everything is saved in a timestamped folder so it
can be post-processed later:

    runs/YYYYmmdd_HHMMSS/
        capture.bin      raw 2-bit samples (same format as capture.py)
        acquisition.txt  PRN / ratio / Doppler / code phase table
        search.txt       code-phase search result for each PRN tried
        track.csv        one row per correlator line from the board (10 Hz)
        prompts_pll.bin  same, for the part of the run with the carrier loop on
        prompts.bin      every 1 ms dump while only the DLL runs: 19-byte
                         records (6 x int24 LE: ie qe ip qp il ql, then the
                         8-bit dump counter). Read with load_prompts().

After acquisition it points the channel at each acquired PRN (strongest
first), runs the board's code-phase search ("srch"), and takes the first one
whose search peak clearly beats the noise. It then closes the tracking loops
("trk") and logs. track.csv gets the DLL discriminator and code correction
while the loops are closed.

Usage:
    python3 track.py [port] [--prn N] [--doppler HZ] [--secs S] [--out DIR]
                     [--no-search] [--open-loop] [--no-pll] [--dll-secs S] [--flip-csign]

    --prn N       track this PRN instead of the strongest one found
    --doppler HZ  use this Doppler instead of the acquired one (needs --prn)
    --secs S      stop logging after S seconds (default: until Ctrl-C)
    --out DIR     parent folder for runs (default: runs)
    --no-search   skip the code-phase search (firmware without "srch")
    --open-loop   don't close the loops (old open-loop behaviour)
    --dll-secs S  DLL-only for S seconds first (recording for pll.py), then
                  switch to DLL+PLL in the same run. Default 300; 0 = PLL at once
    --no-pll      DLL only for the whole run (carrier loop never closed)
    --flip-csign  flip the carrier loop's sign first (if the FLL ran away last time)

Default (field run): python3 track.py
    = capture, acquire, search, 5 min DLL-only recording, then live DLL+PLL
      until Ctrl-C. If the PLL runs away, re-run with:
      python3 track.py --dll-secs 10 --flip-csign
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

# Matches: "E   123,   -45  P  6789,  -12  L   101,   22  d=17"
# plus, while the loops are closed: "  dll=-0.012 corr=-0.0240"
# plus, with the carrier loop on:    " f=+1138.52 lock=+0.97"
CORR_RE = re.compile(
    r"E\s*(-?\d+),\s*(-?\d+)\s+P\s*(-?\d+),\s*(-?\d+)\s+L\s*(-?\d+),\s*(-?\d+)\s+d=(\d+)"
    r"(?:\s+dll=([-+]?[\d.]+)\s+corr=([-+]?[\d.]+))?"
    r"(?:\s+f=([-+]?[\d.]+)\s+lock=([-+]?[\d.]+))?")


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


# ---------------------------------------------------------------- board commands
def wait_for(s: serial.Serial, prefix: str, timeout: float):
    """Read lines until one starts with `prefix`; return it, or None on timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = s.readline().decode(errors="ignore").strip()
        if line.startswith(prefix):
            return line
    return None


def point(s: serial.Serial, prn: int, doppler: float) -> bool:
    """Set the channel's PRN and Doppler. True once the board confirms."""
    s.reset_input_buffer()
    s.write(f"ch {prn} {doppler:.1f}\r\n".encode())
    line = wait_for(s, "ch:", 3)
    if line:
        print(line)
    return line is not None


def search(s: serial.Serial):
    """Run the code-phase search on the board. Returns (ratio, raw line) or (None, None)."""
    s.write(b"srch\r\n")
    print("searching code phase (~10 s)...")
    line = wait_for(s, "search:", SEARCH_TIMEOUT)
    if line is None:
        return None, None
    print(line)
    m = re.search(r"ratio\s+([\d.]+)", line)
    return (float(m.group(1)) if m else None), line


def start_tracking(s: serial.Serial, pll: bool) -> bool:
    """Close the loops on the board (DLL, plus FLL/PLL if pll). True once it confirms."""
    s.write(b"trk pll\r\n" if pll else b"trk\r\n")
    line = wait_for(s, "trk:", 3)
    if line:
        print(line)
    return line is not None


# ---------------------------------------------------------------- tracking log
class Segment:
    """One 1 ms stream file plus its stats. A run has a DLL-only segment
    (prompts.bin: carrier NCO fixed, for pll.py) and/or a PLL segment
    (prompts_pll.bin: carrier NCO steered by the board)."""

    def __init__(self, path: str):
        self.path = path
        self.f = open(path, "wb")
        self.stats = StreamStats()

    def add_line(self, hexpart: str):
        try:
            raw = bytes.fromhex(hexpart)
        except ValueError:
            self.stats.bad_lines += 1
            return
        if len(raw) % RECORD_BYTES:
            self.stats.bad_lines += 1             # truncated line: drop it, the gap shows up
            return
        self.f.write(raw)
        self.f.flush()
        self.stats.add(raw)

    def close(self, name: str):
        self.f.close()
        if self.stats.records == 0:
            os.remove(self.path)                  # nothing streamed in this mode
        else:
            print(f"1 ms stream ({name}, {os.path.basename(self.path)}): {self.stats.summary()}")


def track(s: serial.Serial, prn: int, doppler: float, run_dir: str, secs,
          mode: str, switch_to_pll_at=None):
    """Log until `secs` (or Ctrl-C). mode = "open", "dll" or "pll". If
    switch_to_pll_at is set, send "trk pll" that many seconds in."""
    t0 = time.time()
    segs = {"dll": Segment(os.path.join(run_dir, "prompts.bin")),
            "pll": Segment(os.path.join(run_dir, "prompts_pll.bin"))}
    cur = "pll" if mode == "pll" else "dll"
    switched = switch_to_pll_at is None

    with open(os.path.join(run_dir, "track.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "unix_time", "prn", "doppler_hz",
                    "ie", "qe", "ip", "qp", "il", "ql", "dumps",
                    "dll_disc", "code_corr_chips_s",          # blank when open loop
                    "mode", "carrier_doppler_hz", "pll_lock"])  # last two: carrier loop on
        stop = "Ctrl-C to stop" if secs is None else f"{secs:.0f} s, or Ctrl-C"
        print(f"\nlogging PRN {prn} at {doppler:+.1f} Hz, mode {mode}  ({stop})")
        if not switched:
            print(f"(switching to DLL+PLL after {switch_to_pll_at:.0f} s)")
        print("    t      |E|     |P|     |L|   P phase     dll     corr         f    lock")
        try:
            while secs is None or time.time() - t0 < secs:
                if not switched and time.time() - t0 >= switch_to_pll_at:
                    s.write(b"trk pll\r\n")         # segment changes when the board confirms
                    switched = True

                line = s.readline().decode(errors="ignore").strip()

                if line.startswith("S "):          # 1 ms stream: 10 raw 19-byte records
                    segs[cur].add_line(line[2:])
                    continue
                if line.startswith(("trk:", "csign:")):
                    print(line)
                    if line.startswith("trk:"):
                        cur = "pll" if "pll" in line else "dll"
                        mode = cur
                    continue

                m = CORR_RE.search(line)
                if not m:
                    continue
                ie, qe, ip, qp, il, ql, d = map(int, m.groups()[:7])
                dll, corr, fhz, lock = (g or "" for g in m.groups()[7:11])
                now = time.time()
                w.writerow([f"{now - t0:.3f}", f"{now:.3f}", prn, doppler,
                            ie, qe, ip, qp, il, ql, d, dll, corr, mode, fhz, lock])
                f.flush()                          # survive a yanked cable
                print(f"{now - t0:6.1f}  {np.hypot(ie, qe):7.0f} {np.hypot(ip, qp):7.0f} "
                      f"{np.hypot(il, ql):7.0f}   {np.degrees(np.arctan2(qp, ip)):+6.0f}"
                      f"   {dll:>7} {corr:>8} {fhz:>9} {lock:>7}")
        except KeyboardInterrupt:
            pass

    print()
    for name, seg in segs.items():
        seg.close(name)


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
    """Read prompts.bin into numpy arrays: ie qe ip qp il ql (int32) and dumps (uint8).
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


# ---------------------------------------------------------------- main
SEARCH_TIMEOUT = 60      # s; a full search should take ~10 s
SEARCH_MIN_RATIO = 8.0   # noise alone gives ~4-5


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("--prn", type=int)
    ap.add_argument("--doppler", type=float)
    ap.add_argument("--secs", type=float)
    ap.add_argument("--out", default="runs")
    ap.add_argument("--no-search", action="store_true",
                    help="skip the code-phase search (old firmware)")
    ap.add_argument("--open-loop", action="store_true",
                    help="don't send trk: leave the NCOs fixed")
    ap.add_argument("--pll", action=argparse.BooleanOptionalAction, default=True,
                    help="close the carrier loop after the DLL-only stage (default on; "
                         "--no-pll = DLL only for the whole run)")
    ap.add_argument("--dll-secs", type=float, default=300.0,
                    help="DLL-only recording before switching to DLL+PLL, s (default 300; "
                         "0 = PLL straight away)")
    ap.add_argument("--flip-csign", action="store_true",
                    help="flip the carrier loop sign before tracking (if the FLL runs away)")
    args = ap.parse_args()
    if args.doppler is not None and args.prn is None:
        ap.error("--doppler needs --prn")

    run_dir = os.path.join(args.out, time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    cap_path = os.path.join(run_dir, "capture.bin")

    with serial.Serial(args.port, timeout=1) as s:
        s.dtr = True
        time.sleep(0.3)                            # let the boot messages go by

        data = capture(s)
        with open(cap_path, "wb") as f:
            f.write(data)
        print(f"saved {len(data)} bytes to {cap_path}")

        print("acquiring (all 32 PRNs)...")
        found = acquire_all(cap_path, os.path.join(run_dir, "acquisition.txt"))

        # Candidates to try, best first.
        if args.prn is not None:
            if args.doppler is not None:
                candidates = [(args.prn, args.doppler)]
            else:
                match = [(p, d) for p, d, _ in found if p == args.prn]
                if not match:
                    sys.exit(f"PRN {args.prn} not acquired; give --doppler to force it")
                candidates = match
        else:
            if not found:
                sys.exit("no satellites acquired - capture saved, try again with a clearer sky view")
            candidates = [(p, d) for p, d, _ in found]

        # Point + search each candidate until one gives a clear code peak.
        chosen = None
        with open(os.path.join(run_dir, "search.txt"), "w") as log:
            for prn, doppler in candidates:
                print(f"\nPRN {prn} at {doppler:+.1f} Hz")
                if not point(s, prn, doppler):
                    sys.exit("board never echoed 'ch:' - is the firmware up to date?")
                if args.no_search:
                    chosen = (prn, doppler)
                    break
                ratio, line = search(s)
                log.write(f"PRN {prn} doppler {doppler:+.1f}: {line or 'no reply'}\n")
                if ratio is None:
                    print("no 'search:' reply - firmware without srch? (use --no-search)")
                    continue
                if ratio >= SEARCH_MIN_RATIO:
                    chosen = (prn, doppler)
                    break
                print(f"ratio {ratio:.1f} < {SEARCH_MIN_RATIO}: no clear peak, trying next")

        mode, switch_at = "open", None
        if chosen is None:
            prn, doppler = candidates[0]
            print(f"\nno candidate gave a clear peak; logging PRN {prn} open loop anyway")
            point(s, prn, doppler)
        else:
            prn, doppler = chosen
            # Only close the loops on a real peak: a DLL fed pure noise just wanders.
            if not args.open_loop:
                if args.flip_csign:
                    s.write(b"csign\r\n")
                    print(wait_for(s, "csign:", 3) or "no 'csign:' reply")
                first_pll = args.pll and not args.dll_secs
                if start_tracking(s, first_pll):
                    mode = "pll" if first_pll else "dll"
                    if args.pll and args.dll_secs:
                        switch_at = args.dll_secs
                else:
                    print("board never echoed 'trk:' - firmware without trk? logging open loop")

        track(s, prn, doppler, run_dir, args.secs, mode, switch_at)

    print(f"\nall data in {run_dir}/")


if __name__ == "__main__":
    main()
