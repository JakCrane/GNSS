#!/usr/bin/env python3
"""Play a capture through the SystemVerilog tracking channel in simulation.

1. Acquires the PRN in Python (coarse + fine Doppler) to get the starting values.
2. Converts the capture into a sample file for the testbench.
3. Runs Gateware/sim/tb_channel.sv with iverilog and prints the correlator sums.

Open loop: the fwords are fixed, so the channel starts aligned and then slowly
drifts. Prompt should be clearly larger than early/late for the first dumps.

Usage:  python3 channel_test.py [capture.bin] [prn] [--offset N]
        --offset N  start the code N samples late (to see early/late respond)
"""
import os
import shutil
import subprocess
import sys

import numpy as np

import acquire as acq

HERE = os.path.dirname(os.path.abspath(__file__))
GATEWARE = os.path.join(HERE, "..", "Gateware")
SIM_DIR = os.path.join(GATEWARE, "sim")
WORK = os.path.join(SIM_DIR, "work")
RTL = ["channel.sv", "nco.sv", "ca_gen.sv", "mixer.sv"]
TWO32 = 2 ** 32


def tool(name: str) -> str:
    found = shutil.which(name)
    if found:
        return found
    fallback = os.path.expanduser(f"~/tools/oss-cad-suite/bin/{name}")
    if os.path.exists(fallback):
        return fallback
    sys.exit(f"can't find {name}: put oss-cad-suite on PATH")


def fine_acquire(x: np.ndarray, prn: int):
    _, fd, _, _ = acq.acquire(x, prn, np.arange(-acq.DOPPLER_MAX, acq.DOPPLER_MAX + 1, acq.DOPPLER_STEP))
    best = (0.0, fd, 0)
    for f in np.arange(fd - 500, fd + 501, 25):
        _, _, _, grid = acq.acquire(x, prn, np.array([f]))
        if grid.max() > best[0]:
            best = (grid.max(), f, int(np.argmax(grid[0])))
    return best[1], best[2]          # Doppler (Hz), code start (sample index)


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    path = args[0] if args else os.path.join(HERE, "capture1.bin")
    prn = int(args[1]) if len(args) > 1 else 27
    offset = 0
    if "--offset" in sys.argv:
        offset = int(sys.argv[sys.argv.index("--offset") + 1])

    raw = acq.load(path)
    x = raw - raw.mean()
    fd, code_start = fine_acquire(x, prn)

    carr = round((acq.F_IF + fd) / acq.FS * TWO32)
    code = round((acq.CHIP_RATE + fd / 1540) / acq.FS * TWO32)
    # Channel latency: prompt is 8 clocks behind the generator and i_mix 1 clock behind
    # the sample, which predicts releasing reset 7 samples before the signal's chip 0.
    # In practice early/late balance best at 9 (checked with --offset on capture1.bin):
    # the last ~2 samples come from acquisition's 1-sample resolution and filter delay.
    start = (code_start - 9 + offset) % acq.SAMPLES_PER_MS

    os.makedirs(WORK, exist_ok=True)
    samples_file = os.path.join(WORK, "samples.hex")
    codes = (raw.astype(int) & 0b111)        # ±1/±3 as 3-bit two's complement
    np.savetxt(samples_file, codes, fmt="%x")

    print(f"PRN {prn}: Doppler {fd:+.0f} Hz, code starts at sample {code_start}")
    print(f"carrier fword {carr} (0x{carr:08x}), code fword {code} (0x{code:08x}), "
          f"reset released at sample {start}" + (f" (offset {offset:+d})" if offset else ""))

    sim = os.path.join(WORK, "tb_channel.vvp")
    srcs = [os.path.join(SIM_DIR, "tb_channel.sv")] + [os.path.join(GATEWARE, f) for f in RTL]
    subprocess.run([tool("iverilog"), "-g2012", "-o", sim] + srcs, check=True,
                   stderr=subprocess.DEVNULL)
    out = subprocess.run([tool("vvp"), "-n", sim, f"+SAMPLES={samples_file}", f"+N={len(codes)}",
                          f"+PRN={prn}", f"+CARR={carr}", f"+CODE={code}", f"+START={start}"],
                         check=True, capture_output=True, text=True).stdout

    print(f"\n{'dump':>4}  {'|early|':>8}  {'|prompt|':>8}  {'|late|':>8}   prompt I, Q")
    for line in out.splitlines():
        if not line.startswith("DUMP"):
            continue
        f = line.split()
        k = int(f[1])
        ie, qe, ip, qp, il, ql = (int(f[i]) for i in (3, 5, 7, 9, 11, 13))
        e, p, l = np.hypot(ie, qe), np.hypot(ip, qp), np.hypot(il, ql)
        print(f"{k:>4}  {e:8.0f}  {p:8.0f}  {l:8.0f}   {ip:+6d}, {qp:+6d}")


if __name__ == "__main__":
    main()
