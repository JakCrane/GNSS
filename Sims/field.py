#!/usr/bin/env python3
"""Outdoor test in one command: run it, wait, read the verdict.

    python3 field.py                 # ~10 min: 5 min DLL recording + 3 min live PLL
    python3 field.py --pll-secs 300  # longer live PLL test

What it does:
  1. checks the board is there and running the 4-channel firmware
  2. runs track.py: capture, acquire, search + track up to 4 satellites,
     DLL-only recording for --dll-secs, then live DLL+PLL for --pll-secs
  3. prints a verdict per channel:
       DLL  - did |P| stay up (code held) for the whole DLL stage?
       PLL  - how much of the time was the firmware PLL locked (cos 2phi > 0.8)?
       data - missed dumps in each 1 ms recording
  4. decodes the DLL recordings with pll.py (offline PLL -> bits -> subframes),
     so you know before packing up whether you've got GPS time / subframes 1-3
  5. if the live PLL failed on every channel the way a wrong loop sign does
     (frequency runs away, |P| collapses), it re-runs once with the sign flipped

Everything is saved under runs/; the summary is also written to field_summary.txt
in the run folder.
"""
import argparse
import csv
import glob
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from track import load_prompts   # noqa: E402

LOCK_GOOD = 0.8          # cos(2 phi) above this = phase locked


def newest_run(out_dir, after):
    runs = [d for d in glob.glob(os.path.join(out_dir, "*/")) if os.path.getmtime(d) >= after - 1]
    return max(runs, key=os.path.getmtime).rstrip("/") if runs else None


def check_board(port):
    import serial
    if not os.path.exists(port):
        sys.exit(f"{port} not found - is the board plugged in (and finished booting)?")
    with serial.Serial(port, timeout=0.5) as s:
        s.dtr = True
        t = time.time()
        lines = []
        while time.time() - t < 3:
            l = s.readline().decode(errors="ignore").strip()
            if l.startswith(("flash:", "boot:", "nch:")):
                lines.append(l)
            if l.startswith("nch:"):
                break
    for l in lines:
        print("  " + l)
    if not any(l.startswith("boot: DONE=1") for l in lines):
        print("  WARNING: no 'boot: DONE=1' - the FPGA may not have configured")
    m = [re.match(r"nch:\s*(\d+)", l) for l in lines]
    nch = next((int(x.group(1)) for x in m if x), 0)
    if nch < 1:
        print("  WARNING: no 'nch:' line - old firmware? continuing anyway")
    return nch


def run_track(args, extra):
    cmd = [sys.executable, os.path.join(HERE, "track.py"), args.port, "--out", args.out,
           "--dll-secs", str(args.dll_secs), "--secs", str(args.dll_secs + args.pll_secs)] + extra
    print("\n$ " + " ".join(cmd[1:]) + "\n")
    t0 = time.time()
    try:
        subprocess.run(cmd, cwd=HERE)
    except KeyboardInterrupt:
        print("\n(stopped early)")
    return newest_run(os.path.join(HERE, args.out), t0)


def summarise(run, dll_secs):
    """Per-channel verdicts from channels.csv, track.csv and the stream files."""
    out = []
    chans_csv = os.path.join(run, "channels.csv")
    if not os.path.exists(chans_csv):
        out.append("no channels were tracked (see search.txt / acquisition.txt)")
        return out, []
    with open(chans_csv) as f:
        chans = list(csv.DictReader(f))
    rows = list(csv.DictReader(open(os.path.join(run, "track.csv"))))
    verdicts = []
    out.append(f"{'ch':>2} {'PRN':>3} {'search':>6} | {'DLL: |P| start->end':>21} {'held':>4} | "
               f"{'PLL locked':>10} {'f start->end (Hz)':>19} | data gaps (dll/pll)")
    for c in chans:
        ch = c["ch"]
        r = [x for x in rows if x["ch"] == ch and x["mode"] in ("dll", "pll")]
        dll = [x for x in r if x["mode"] == "dll"]
        pll = [x for x in r if x["mode"] == "pll" and x["pll_lock"]]

        def p_of(x):
            return (float(x["ip"]) ** 2 + float(x["qp"]) ** 2) ** 0.5

        def mean(v):
            return sum(v) / len(v) if v else float("nan")

        p0 = mean([p_of(x) for x in dll[:100]])            # first ~10 s of DLL
        p1 = mean([p_of(x) for x in dll[-100:]])           # last ~10 s of DLL
        held = dll and p1 > 0.5 * p0 and p1 > 1500         # noise |P| is ~800
        if pll:
            late = pll[len(pll) // 3:]                      # skip pull-in
            lock = sum(float(x["pll_lock"]) > LOCK_GOOD for x in late) / len(late)
            f0, f1 = float(pll[0]["carrier_doppler_hz"]), float(pll[-1]["carrier_doppler_hz"])
            p_end = mean([p_of(x) for x in pll[-50:]])
            ran_away = lock < 0.2 and (abs(f1 - f0) > 150 or p_end < 1500)
        else:
            lock, f0, f1, ran_away = None, None, None, False

        gaps = []
        for name in (f"prompts_ch{ch}.bin", f"prompts_pll_ch{ch}.bin"):
            path = os.path.join(run, name)
            if os.path.exists(path):
                d = load_prompts(path)["dumps"].astype(int)
                st = [(b - a) % 256 for a, b in zip(d[:-1], d[1:])]
                gaps.append(f"{sum(s - 1 for s in st if s > 1)}/{len(d)}")
            else:
                gaps.append("-")

        verdicts.append({"ch": ch, "prn": c["prn"], "held": bool(held), "lock": lock,
                         "ran_away": ran_away})
        lock_s = f"{100 * lock:5.0f}%" if lock is not None else "    -"
        f_s = f"{f0:+8.1f}->{f1:+8.1f}" if f0 is not None else "-"
        out.append(f"{ch:>2} {c['prn']:>3} {float(c['search_ratio']):6.1f} | "
                   f"{p0:9.0f} -> {p1:7.0f} {'yes' if held else 'NO':>4} | "
                   f"{lock_s:>10} {f_s:>19} | {gaps[0]} / {gaps[1]}")
    return out, verdicts


def decode(run):
    """Offline PLL + frame sync on every channel's DLL recording."""
    out = []
    if not glob.glob(os.path.join(run, "prompts_ch*.bin")):
        return ["no DLL recordings to decode"]
    print("\ndecoding the DLL recordings (offline PLL -> bits -> subframes), ~20 s per channel...")
    r = subprocess.run([sys.executable, os.path.join(HERE, "pll.py"), run],
                       cwd=HERE, capture_output=True, text=True)
    ch = None
    for line in r.stdout.splitlines():
        m = re.match(r"=+ channel (\d)", line)
        if m:
            ch = m.group(1)
        elif line.startswith(("PLL:", "frames:", "C/N0")) and ch is not None:
            out.append(f"ch{ch}  {line}")
    for f in sorted(glob.glob(os.path.join(run, "subframes_ch*.txt"))):
        ch = re.search(r"_ch(\d)", f).group(1)
        rows = [l.split() for l in open(f).read().splitlines()[1:] if l.strip()]
        ids = [int(x[3]) for x in rows]
        if rows:
            run_123 = any(ids[i:i + 3] == [1, 2, 3] for i in range(len(ids)))
            out.append(f"ch{ch}  subframe IDs {ids}"
                       + ("  <-- 1,2,3 in a row: ephemeris available" if run_123 else ""))
            out.append(f"ch{ch}  GPS time of last subframe: {' '.join(rows[-1][5:])}")
    if r.returncode:
        out.append("pll.py error:\n" + r.stderr[-800:])
    return out


def report(run, title, dll_secs, do_decode):
    lines = [f"======== {title}: {run}"]
    s, verdicts = summarise(run, dll_secs)
    lines += s
    if do_decode:
        lines += [""] + decode(run)
    text = "\n".join(lines)
    print("\n" + text)
    with open(os.path.join(run, "field_summary.txt"), "w") as f:
        f.write(text + "\n")
    return verdicts


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("--dll-secs", type=int, default=300, help="DLL-only recording, s (default 300)")
    ap.add_argument("--pll-secs", type=int, default=180, help="live PLL test, s (default 180)")
    ap.add_argument("--out", default="runs")
    ap.add_argument("--no-decode", action="store_true", help="skip the offline decode")
    ap.add_argument("--no-retry", action="store_true", help="don't re-run with the sign flipped")
    args = ap.parse_args()

    print("checking the board...")
    check_board(args.port)

    run = run_track(args, [])
    if not run:
        sys.exit("track.py made no run folder")
    verdicts = report(run, "RUN 1", args.dll_secs, not args.no_decode)

    tried = [v for v in verdicts if v["lock"] is not None]
    if tried and all(v["ran_away"] for v in tried) and not args.no_retry:
        print("\nThe live PLL ran away on every channel - the classic sign of a wrong loop sign.")
        print("Re-running with the carrier sign flipped (short DLL stage: the long recording is done).")
        time.sleep(2)
        args.dll_secs = 20
        run2 = run_track(args, ["--flip-csign"])
        if run2:
            report(run2, "RUN 2 (carrier sign flipped)", args.dll_secs, False)
            print("\nIf RUN 2 locked, keep --flip-csign for future runs (or flip csign's default in "
                  "the firmware).")

    print("\nDone. Back at the desk:  python3 pll.py " + run + " --plot")


if __name__ == "__main__":
    main()
