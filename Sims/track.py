#!/usr/bin/env python3
"""Capture -> acquire -> point the FPGA channel at a satellite -> log tracking.

One command for field use. Everything is saved in a timestamped folder so it
can be post-processed later:

    runs/YYYYmmdd_HHMMSS/
        capture.bin      raw 2-bit samples (same format as capture.py)
        acquisition.txt  PRN / ratio / Doppler / code phase table
        track.csv        one row per correlator line from the board

Usage:
    python3 track.py [port] [--prn N] [--doppler HZ] [--secs S] [--out DIR]

    --prn N       track this PRN instead of the strongest one found
    --doppler HZ  use this Doppler instead of the acquired one (needs --prn)
    --secs S      stop logging after S seconds (default: until Ctrl-C)
    --out DIR     parent folder for runs (default: runs)
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
CORR_RE = re.compile(
    r"E\s*(-?\d+),\s*(-?\d+)\s+P\s*(-?\d+),\s*(-?\d+)\s+L\s*(-?\d+),\s*(-?\d+)\s+d=(\d+)")


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


# ---------------------------------------------------------------- tracking log
def track(s: serial.Serial, prn: int, doppler: float, csv_path: str, secs):
    s.reset_input_buffer()
    s.write(f"ch {prn} {doppler:.1f}\r\n".encode())

    confirmed = False
    t0 = time.time()
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "unix_time", "prn", "doppler_hz",
                    "ie", "qe", "ip", "qp", "il", "ql", "dumps"])
        print(f"\ntracking PRN {prn} at {doppler:+.1f} Hz  (Ctrl-C to stop)")
        print("    t      |E|     |P|     |L|   P phase")
        try:
            while secs is None or time.time() - t0 < secs:
                line = s.readline().decode(errors="ignore").strip()
                if line.startswith("ch:"):
                    confirmed = True              # rows before this are the old setting
                    print(line)
                    continue
                m = CORR_RE.search(line)
                if not m or not confirmed:
                    continue
                ie, qe, ip, qp, il, ql, d = map(int, m.groups())
                now = time.time()
                w.writerow([f"{now - t0:.3f}", f"{now:.3f}", prn, doppler,
                            ie, qe, ip, qp, il, ql, d])
                f.flush()                         # survive a yanked cable
                print(f"{now - t0:6.1f}  {np.hypot(ie, qe):7.0f} {np.hypot(ip, qp):7.0f} "
                      f"{np.hypot(il, ql):7.0f}   {np.degrees(np.arctan2(qp, ip)):+6.0f}")
        except KeyboardInterrupt:
            pass
    if not confirmed:
        print("warning: board never echoed 'ch:' - is the firmware up to date?")


# ---------------------------------------------------------------- main
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("--prn", type=int)
    ap.add_argument("--doppler", type=float)
    ap.add_argument("--secs", type=float)
    ap.add_argument("--out", default="runs")
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

        if args.prn is not None:
            prn = args.prn
            if args.doppler is not None:
                doppler = args.doppler
            else:
                match = [d for p, d, _ in found if p == prn]
                if not match:
                    sys.exit(f"PRN {prn} not acquired; give --doppler to force it")
                doppler = match[0]
        else:
            if not found:
                sys.exit("no satellites acquired - capture saved, try again with a clearer sky view")
            prn, doppler, ratio = found[0]
            print(f"\nstrongest: PRN {prn} (ratio {ratio:.2f})")

        track(s, prn, doppler, os.path.join(run_dir, "track.csv"), args.secs)

    print(f"\nall data in {run_dir}/")


if __name__ == "__main__":
    main()
