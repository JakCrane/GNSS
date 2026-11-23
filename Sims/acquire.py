#!/usr/bin/env python3
"""GPS L1 C/A acquisition on a capture.bin from the FPGA.

For every PRN (1-32) it searches all code phases (via FFT correlation) and a
range of Doppler offsets, adds up several 1 ms periods non-coherently, and
reports how strongly each satellite stands out from the noise.

Capture format (see capture.py / top.sv):
  - 16.368 MHz real (I-only) samples, 2 bits each, 4 per byte, oldest first
  - each 2-bit sample is {mag, sign} = {I0, I1}

Usage:  python3 acquire.py [capture.bin] [--plot]
"""
import sys

import numpy as np

FS = 16.368e6            # sample rate = MAX2769 CLKOUT
F_IF = 4.092e6           # nominal IF for a 16.368 MHz reference (fs/4)
CHIP_RATE = 1.023e6
CODE_LEN = 1023
SAMPLES_PER_MS = int(FS / 1000)          # 16368 = exactly 16 samples per chip

DOPPLER_MAX = 10e3       # ± search range: satellite motion (~±5 kHz) + TCXO error
DOPPLER_STEP = 500       # Hz; suits 1 ms coherent integration
THRESHOLD = 2.5          # peak / second-best-peak ratio counted as "acquired"

# G2 phase-selector taps for PRN 1..32 (IS-GPS-200, Table 3-Ia)
G2_TAPS = [
    (2, 6), (3, 7), (4, 8), (5, 9), (1, 9), (2, 10), (1, 8), (2, 9),
    (3, 10), (2, 3), (3, 4), (5, 6), (6, 7), (7, 8), (8, 9), (9, 10),
    (1, 4), (2, 5), (3, 6), (4, 7), (5, 8), (6, 9), (1, 3), (4, 6),
    (5, 7), (6, 8), (7, 9), (8, 10), (1, 6), (2, 7), (3, 8), (4, 9),
]


def ca_code(prn: int) -> np.ndarray:
    """One period (1023 chips) of the C/A code for a PRN, as ±1."""
    g1 = [1] * 10
    g2 = [1] * 10
    a, b = G2_TAPS[prn - 1]
    out = np.empty(CODE_LEN, dtype=np.int8)
    for i in range(CODE_LEN):
        out[i] = g1[9] ^ g2[a - 1] ^ g2[b - 1]
        g1 = [g1[2] ^ g1[9]] + g1[:9]
        g2 = [g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]] + g2[:9]
    return 1 - 2 * out.astype(np.float32)    # 0 → +1, 1 → -1


def load(path: str) -> np.ndarray:
    """Unpack the 2-bit sign/magnitude samples into values ±1, ±3."""
    raw = np.fromfile(path, dtype=np.uint8)
    pairs = np.stack([(raw >> s) & 0b11 for s in (6, 4, 2, 0)], axis=1).ravel()
    mag = pairs >> 1
    sign = pairs & 1
    return np.where(sign == 1, 1.0, -1.0) * np.where(mag == 1, 3.0, 1.0)


def acquire(x: np.ndarray, prn: int, dopplers: np.ndarray):
    """Return (peak ratio, doppler Hz, code phase chips, correlation grid)."""
    n_ms = len(x) // SAMPLES_PER_MS
    blocks = x[: n_ms * SAMPLES_PER_MS].reshape(n_ms, SAMPLES_PER_MS)
    t = np.arange(SAMPLES_PER_MS) / FS

    code = np.repeat(ca_code(prn), SAMPLES_PER_MS // CODE_LEN)
    code_fft_conj = np.conj(np.fft.fft(code))

    grid = np.zeros((len(dopplers), SAMPLES_PER_MS))
    for k, fd in enumerate(dopplers):
        carrier = np.exp(-2j * np.pi * (F_IF + fd) * t)
        # carrier phase restarts each block; fine because blocks are summed non-coherently
        corr = np.fft.ifft(np.fft.fft(blocks * carrier, axis=1) * code_fft_conj, axis=1)
        grid[k] = np.sum(np.abs(corr) ** 2, axis=0)

    k_best, s_best = np.unravel_index(np.argmax(grid), grid.shape)
    peak = grid[k_best, s_best]

    # Second-best peak in the same Doppler row, excluding ±1 chip around the best one.
    row = grid[k_best].copy()
    spc = SAMPLES_PER_MS // CODE_LEN
    idx = (np.arange(-spc, spc + 1) + s_best) % SAMPLES_PER_MS
    row[idx] = 0
    ratio = peak / row.max()

    return ratio, dopplers[k_best], s_best / spc, grid


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    path = args[0] if args else "capture.bin"
    plot = "--plot" in sys.argv

    x = load(path)
    x -= x.mean()
    n_ms = len(x) // SAMPLES_PER_MS
    dopplers = np.arange(-DOPPLER_MAX, DOPPLER_MAX + 1, DOPPLER_STEP)
    print(f"{path}: {len(x)} samples = {n_ms} ms, "
          f"{len(dopplers)} Doppler bins of {DOPPLER_STEP} Hz, threshold {THRESHOLD}")
    print(" PRN   ratio   Doppler(Hz)   code phase(chips)")

    results = {}
    for prn in range(1, 33):
        ratio, fd, phase, grid = acquire(x, prn, dopplers)
        results[prn] = (ratio, grid)
        mark = "  <-- ACQUIRED" if ratio > THRESHOLD else ""
        print(f" {prn:3d}   {ratio:5.2f}   {fd:+9.0f}     {phase:8.1f}{mark}")

    found = [p for p, (r, _) in results.items() if r > THRESHOLD]
    print(f"\nacquired: {found if found else 'none'}")

    if plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(13, 4))
        prns = list(results)
        ratios = [results[p][0] for p in prns]
        ax[0].bar(prns, ratios, color=["tab:green" if r > THRESHOLD else "tab:gray" for r in ratios])
        ax[0].axhline(THRESHOLD, color="tab:red", linestyle="--")
        ax[0].set(xlabel="PRN", ylabel="peak / 2nd peak", title="Acquisition metric")
        best = max(prns, key=lambda p: results[p][0])
        grid = results[best][1]
        ax[1].imshow(grid, aspect="auto", origin="lower",
                     extent=[0, CODE_LEN, dopplers[0], dopplers[-1]])
        ax[1].set(xlabel="code phase (chips)", ylabel="Doppler (Hz)",
                  title=f"Search grid, PRN {best}")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
