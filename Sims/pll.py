#!/usr/bin/env python3
"""Offline carrier tracking + navigation bit/frame sync on a recorded 1 ms stream.

Input is a run folder from track.py (prompts_ch<n>.bin + channels.csv, or an
older single-channel run's prompts.bin + track.csv), recorded while
the board's DLL held the code but the carrier NCO stayed fixed. The prompt
then rotates at whatever carrier error is left; this script does the FLL/PLL
in software, then:

  1. FLL pulls in the frequency, then a Costas PLL locks the phase
     (Costas = insensitive to the 180 deg data-bit flips)
  2. bit sync: finds where the 20 ms bit edges fall
  3. bits: sums 20 ms blocks of the locked prompt I
  4. frame sync: finds the 10001011 preamble, checks parity on all
     10 words of each subframe, reads subframe ID and TOW from the HOW word

Outputs go into the run folder:
  pll_out_ch<n>.npz    t_ms, I, Q (after PLL), freq_hz, lock, bits, bit_ms
  subframes_ch<n>.txt  every parity-checked subframe: bit index, ID, TOW, GPS time
  (no _ch<n> suffix for an old single-channel run)

Usage:
  python3 pll.py RUN_DIR [--ch N ...] [--pll-bw HZ (default 40)] [--fll-bw HZ] [--fll-secs S] [--plot]
  python3 pll.py --selftest [--cn0 DBHZ] [--plot]

--selftest generates a synthetic recording (real subframe structure, parity,
frequency offset + drift, missed dumps) and checks the whole chain.
"""
import argparse
import csv
import os
import re
import shutil
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from track import load_prompts, RECORD_BYTES   # noqa: E402

T = 1e-3                       # one dump = 1 ms
MS_PER_BIT = 20
BITS_PER_SUBFRAME = 300
PREAMBLE = np.array([1, 0, 0, 0, 1, 0, 1, 1], dtype=np.uint8)

# GPS LNAV parity (IS-GPS-200 20.3.5.2): (uses D29* or D30*, data bits d1..d24)
PARITY = [
    (29, [1, 2, 3, 5, 6, 10, 11, 12, 13, 14, 17, 18, 20, 23]),        # D25
    (30, [2, 3, 4, 6, 7, 11, 12, 13, 14, 15, 18, 19, 21, 24]),        # D26
    (29, [1, 3, 4, 5, 7, 8, 12, 13, 14, 15, 16, 19, 20, 22]),         # D27
    (30, [2, 4, 5, 6, 8, 9, 13, 14, 15, 16, 17, 20, 21, 23]),         # D28
    (30, [1, 3, 5, 6, 7, 9, 10, 14, 15, 16, 17, 18, 21, 22, 24]),     # D29
    (29, [3, 5, 6, 8, 9, 10, 11, 13, 15, 19, 22, 23, 24]),            # D30
]


# ---------------------------------------------------------------- input
def run_channels(run_dir: str):
    """Channels with a DLL-only recording in this run folder. Multi-channel runs
    have prompts_ch<n>.bin; older single-channel runs have prompts.bin (-> [None])."""
    chans = sorted(int(m.group(1)) for f in os.listdir(run_dir)
                   for m in [re.fullmatch(r"prompts_ch(\d)\.bin", f)] if m)
    if chans:
        return chans
    return [None] if os.path.exists(os.path.join(run_dir, "prompts.bin")) else []


def load_run(run_dir: str, ch=None):
    """Prompts on a gap-free 1 ms grid (missed dumps = 0), plus PRN and Doppler.
    ch=None reads an old single-channel run (prompts.bin)."""
    name = "prompts.bin" if ch is None else f"prompts_ch{ch}.bin"
    p = load_prompts(os.path.join(run_dir, name))
    d = p["dumps"].astype(np.int64)
    step = np.diff(d) % 256
    step[step == 0] = 256
    idx = np.concatenate([[0], np.cumsum(step)])        # ms index of each record

    z = np.zeros(idx[-1] + 1, dtype=complex)
    z[idx] = p["ip"] + 1j * p["qp"]
    have = np.zeros(len(z), dtype=bool)
    have[idx] = True

    prn, doppler = None, None
    chan_csv = os.path.join(run_dir, "channels.csv")
    csv_path = os.path.join(run_dir, "track.csv")
    if ch is not None and os.path.exists(chan_csv):
        with open(chan_csv) as f:
            for row in csv.DictReader(f):
                if int(row["ch"]) == ch:
                    prn, doppler = int(row["prn"]), float(row["doppler_hz"])
    elif os.path.exists(csv_path):
        with open(csv_path) as f:
            row = next(csv.DictReader(f), None)
            if row and row.get("prn"):
                prn, doppler = int(row["prn"]), float(row["doppler_hz"])
    return z, have, prn, doppler


# ---------------------------------------------------------------- carrier loops
def initial_freq(z: np.ndarray, have: np.ndarray, secs: float = 2.0) -> float:
    """Coarse frequency of the prompt from an FFT of z^2 (squaring removes the
    data bits). Unambiguous for |f| < 250 Hz at a 1 kHz dump rate."""
    n = min(len(z), int(secs / T))
    zz = (z[:n] ** 2) * have[:n]
    nfft = 1 << max(14, int(np.ceil(np.log2(n))))
    spec = np.abs(np.fft.fft(zz, nfft))
    f2 = np.fft.fftfreq(nfft, T)[np.argmax(spec)]
    return f2 / 2


def track_carrier(z, have, f0, pll_bw=40.0, fll_bw=10.0, fll_secs=1.0, fll_assist_bw=5.0):
    """FLL for the first `fll_secs`, then 2nd-order Costas PLL with a weak FLL
    assist. Returns wiped-off prompt (I + jQ), loop frequency (Hz) per ms."""
    n = len(z)
    w0 = pll_bw / 0.53                       # 2nd-order loop natural frequency
    a2 = np.sqrt(2.0)
    theta = 0.0                              # NCO phase, cycles
    f_int = f0                               # loop frequency integrator, Hz
    prev = None
    out = np.zeros(n, dtype=complex)
    freq = np.zeros(n)
    n_fll = int(fll_secs / T)

    for k in range(n):
        zr = z[k] * np.exp(-2j * np.pi * theta)
        out[k] = zr
        f = f_int
        if have[k]:
            # FLL: phase change between consecutive dumps, atan (not atan2) so a
            # bit flip between them doesn't matter. +-250 Hz pull-in.
            f_err = 0.0
            if prev is not None and (prev.real or prev.imag):
                cross = prev.real * zr.imag - zr.real * prev.imag
                dot = prev.real * zr.real + prev.imag * zr.imag
                if dot != 0.0:
                    f_err = np.arctan(cross / dot) / (2 * np.pi * T)
            prev = zr

            if k < n_fll:
                f_int += 4.0 * fll_bw * T * f_err
                f = f_int
            else:
                # Costas discriminator: atan(Q/I), cycles, ignores bit sign
                e = np.arctan(zr.imag / zr.real) / (2 * np.pi) if zr.real != 0.0 else 0.0
                f_int += w0 * w0 * T * e + 4.0 * fll_assist_bw * T * f_err
                f = f_int + a2 * w0 * e
        else:
            prev = None                          # gap: no FLL pair across it
        freq[k] = f
        theta += f * T                           # coast through gaps
    return out, freq


def lock_indicator(zr: np.ndarray, have: np.ndarray, win_ms: int = 100) -> np.ndarray:
    """cos(2 * phase error), averaged: ~1 when phase-locked, ~0 when not."""
    i2, q2 = zr.real ** 2, zr.imag ** 2
    k = np.ones(win_ms)
    num = np.convolve((i2 - q2) * have, k, "same")
    den = np.convolve((i2 + q2) * have, k, "same")
    return np.divide(num, den, out=np.zeros_like(num), where=den > 0)


# ---------------------------------------------------------------- bits
def bit_sync(I: np.ndarray, good: np.ndarray) -> tuple:
    """Bit edges: sign changes of I between consecutive locked dumps, histogrammed
    mod 20 ms. Returns (edge offset 0..19, fraction of transitions at that offset)."""
    s = np.sign(I)
    flip = (s[1:] * s[:-1] < 0) & good[1:] & good[:-1]
    pos = (np.nonzero(flip)[0] + 1) % MS_PER_BIT
    if len(pos) == 0:
        return 0, 0.0
    hist = np.bincount(pos, minlength=MS_PER_BIT)
    return int(np.argmax(hist)), float(hist.max() / hist.sum())


def extract_bits(I: np.ndarray, offset: int):
    n_bits = (len(I) - offset) // MS_PER_BIT
    blocks = I[offset: offset + n_bits * MS_PER_BIT].reshape(n_bits, MS_PER_BIT)
    sums = blocks.sum(axis=1)
    bit_ms = offset + MS_PER_BIT * np.arange(n_bits)
    return sums, bit_ms


# ---------------------------------------------------------------- frames
def parity_ok(word: np.ndarray, d29s: int, d30s: int) -> bool:
    """word: 30 transmitted bits D1..D30 (0/1)."""
    d = word[:24] ^ d30s                         # undo the D30* data inversion
    star = {29: d29s, 30: d30s}
    for j, (s, idxs) in enumerate(PARITY):
        p = star[s]
        for i in idxs:
            p ^= d[i - 1]
        if p != word[24 + j]:
            return False
    return True


def word_data(word: np.ndarray, d30s: int) -> np.ndarray:
    return word[:24] ^ d30s


def bits_to_int(b) -> int:
    v = 0
    for x in b:
        v = (v << 1) | int(x)
    return v


def find_subframes(soft: np.ndarray):
    """Search both polarities for preambles, keep subframes whose 10 words all
    pass parity. Returns list of dicts sorted by bit index."""
    hard = (soft > 0).astype(np.uint8)
    found = []
    for inv in (0, 1):
        b = hard ^ inv
        for i in range(0, len(b) - BITS_PER_SUBFRAME + 1):
            if not np.array_equal(b[i:i + 8], PREAMBLE):
                continue
            sf = b[i:i + BITS_PER_SUBFRAME]
            d29s, d30s = 0, 0                    # word 10 always ends in 00 (spec)
            ok = True
            for w in range(10):
                word = sf[30 * w: 30 * w + 30]
                if not parity_ok(word, d29s, d30s):
                    ok = False
                    break
                d29s, d30s = int(word[28]), int(word[29])
            if not ok:
                continue
            how = word_data(sf[30:60], int(sf[29]))
            tow = bits_to_int(how[0:17])          # x 6 s = GPS time of NEXT subframe start
            sfid = bits_to_int(how[19:22])
            found.append({"bit": i, "inverted": inv, "id": sfid, "tow": tow})
    found.sort(key=lambda s: s["bit"])
    return found


def gps_time_str(tow_count: int) -> str:
    s = tow_count * 6                            # seconds into the GPS week
    day = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"][(s // 86400) % 7]
    s %= 86400
    return f"{day} {s // 3600:02d}:{s % 3600 // 60:02d}:{s % 60:02d}"


# ---------------------------------------------------------------- run
def analyse(run_dir, pll_bw, fll_bw, fll_secs, plot, quiet=False, ch=None):
    say = (lambda *a: None) if quiet else print
    z, have, prn, doppler = load_run(run_dir, ch)
    sfx = "" if ch is None else f"_ch{ch}"                   # output file suffix
    if ch is not None:
        say(f"\n======== channel {ch}")
    n_missing = int((~have).sum())
    say(f"{run_dir}: {len(z)} ms ({len(z) / 1000:.1f} s), {n_missing} missed dumps"
        + (f", PRN {prn}, NCO Doppler {doppler:+.1f} Hz" if prn is not None else ""))

    f0 = initial_freq(z, have)
    say(f"coarse prompt frequency (FFT of z^2): {f0:+.2f} Hz")
    zr, freq = track_carrier(z, have, f0, pll_bw, fll_bw, fll_secs)
    lock = lock_indicator(zr, have)
    I, Q = zr.real, zr.imag

    after = int(fll_secs / T)
    locked = (lock > 0.6) & have
    frac = locked[after:].mean() if len(z) > after else 0.0
    def avg_f(a, b):                         # 200 ms mean: one sample is noisy (PLL proportional term)
        seg = freq[max(a, 0):max(b, 1)]
        return float(seg.mean()) if len(seg) else float(freq[-1])
    f_start = avg_f(after, after + 200)
    f_end = avg_f(len(freq) - 200, len(freq))
    say(f"PLL: locked (cos2phi > 0.6) {100 * frac:.0f}% of the time after the FLL stage; "
        f"loop frequency {f_start:+.2f} -> {f_end:+.2f} Hz (200 ms averages)")
    if doppler is not None:
        # mixer gives z ~ exp(j(theta_nco - theta_sig)), so f_sig = f_nco - f_prompt
        say(f"implied carrier Doppler: {doppler - f_end:+.2f} Hz at end "
            f"(set {doppler:+.1f}; sign from the mixer maths, confirm when the FLL goes in firmware)")
    if locked.any():
        q_var = np.var(Q[locked])
        i_mean = np.mean(np.abs(I[locked]))
        cn0 = 10 * np.log10(max(i_mean ** 2 / (2 * q_var * T), 1e-9))
        say(f"C/N0 estimate while locked: {cn0:.1f} dB-Hz")

    offset, conf = bit_sync(I, locked)
    say(f"bit sync: edges at ms offset {offset} (mod 20), {100 * conf:.0f}% of transitions there")
    soft, bit_ms = extract_bits(I, offset)
    subframes = find_subframes(soft)
    say(f"frames: {len(subframes)} subframes passed parity")

    lines = ["bit_index  ms_index  inverted  subframe_id  tow_count  gps_time_of_next_subframe"]
    for s in subframes:
        lines.append(f"{s['bit']:9d}  {bit_ms[s['bit']]:8d}  {s['inverted']:8d}  "
                     f"{s['id']:11d}  {s['tow']:9d}  {gps_time_str(s['tow'])}")
    for ln in lines[1:]:
        say("  " + ln)
    with open(os.path.join(run_dir, f"subframes{sfx}.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    np.savez(os.path.join(run_dir, f"pll_out{sfx}.npz"), t_ms=np.arange(len(z)), I=I, Q=Q,
             freq_hz=freq, lock=lock, have=have, bits=soft, bit_ms=bit_ms, bit_offset=offset)

    if plot:
        make_plot(I, Q, freq, lock, soft, bit_ms, subframes, prn)
    return {"f0": f0, "freq": freq, "f_end": f_end, "lock_frac": frac, "offset": offset,
            "subframes": subframes}


def make_plot(I, Q, freq, lock, soft, bit_ms, subframes, prn):
    import matplotlib.pyplot as plt
    t = np.arange(len(I)) * T
    fig, ax = plt.subplots(4, 1, figsize=(12, 9), sharex=False)
    ax[0].plot(t, I, ".", ms=1, label="I")
    ax[0].plot(t, Q, ".", ms=1, label="Q")
    ax[0].set(ylabel="prompt", title=f"PRN {prn}: prompt after software PLL")
    ax[0].legend(loc="upper right", markerscale=8)
    ax[1].plot(t, freq)
    ax[1].set(ylabel="loop freq (Hz)")
    ax[2].plot(t, lock)
    ax[2].set(ylabel="cos 2phi", ylim=(-1.05, 1.05))
    # zoom: 400 ms of I once locked, with bit boundaries
    k0 = int(np.argmax(lock > 0.8)) if (lock > 0.8).any() else 0
    sl = slice(k0, min(k0 + 400, len(I)))
    ax[3].plot(t[sl], I[sl], ".-", ms=2)
    for b in bit_ms:
        if sl.start <= b < sl.stop:
            ax[3].axvline(b * T, color="gray", lw=0.5)
    ax[3].set(xlabel="time (s)", ylabel="I (zoom)", title="20 ms bits")
    for s in subframes:
        ax[0].axvline(bit_ms[s["bit"]] * T, color="tab:red", lw=0.5)
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------- self-test
def encode_word(d24: np.ndarray, d29s: int, d30s: int) -> np.ndarray:
    star = {29: d29s, 30: d30s}
    par = []
    for s, idxs in PARITY:
        p = star[s]
        for i in idxs:
            p ^= d24[i - 1]
        par.append(p)
    return np.concatenate([d24 ^ d30s, np.array(par, dtype=np.uint8)])


def make_subframe(tow: int, sfid: int, rng, d29s: int, d30s: int) -> np.ndarray:
    words = []
    for w in range(10):
        d = rng.integers(0, 2, 24).astype(np.uint8)
        if w == 0:
            d[:8] = PREAMBLE
        if w == 1:
            d[0:17] = [(tow >> (16 - k)) & 1 for k in range(17)]
            d[19:22] = [(sfid >> (2 - k)) & 1 for k in range(3)]
        if w in (1, 9):                               # t bits: force D29 = D30 = 0
            for t1 in (0, 1):
                for t2 in (0, 1):
                    d[22], d[23] = t1, t2
                    word = encode_word(d, d29s, d30s)
                    if word[28] == 0 and word[29] == 0:
                        break
                else:
                    continue
                break
        else:
            word = encode_word(d, d29s, d30s)
        words.append(word)
        d29s, d30s = int(word[28]), int(word[29])
    return np.concatenate(words)


def selftest(cn0: float, plot: bool, pll_bw: float, fll_bw: float, fll_secs: float) -> None:
    rng = np.random.default_rng(1)
    secs = 70
    n = secs * 1000
    sigma = 630.0                                     # noise per component, as measured
    amp = np.sqrt(2 * sigma ** 2 * 10 ** (cn0 / 10) * T)
    f_start, f_rate = 37.0, 0.6                       # Hz, Hz/s: leftover error + drift
    edge = 7                                          # ms offset of bit edges
    tow0 = 100800 // 6 + 3                            # arbitrary start

    # nav bits: some random lead-in, then real subframes
    lead = rng.integers(0, 2, 137).astype(np.uint8)
    sfs, d29s, d30s, tow = [], 0, 0, tow0
    for k in range(12):
        sf = make_subframe(tow + 1, (k % 5) + 1, rng, d29s, d30s)
        sfs.append(sf)
        d29s, d30s, tow = int(sf[-2]), int(sf[-1]), tow + 1
    nav = np.concatenate([lead] + sfs)
    first_sf_bit = len(lead)

    t = np.arange(n) * T
    phase = 2 * np.pi * (f_start * t + 0.5 * f_rate * t ** 2) + rng.uniform(0, 2 * np.pi)
    bit_idx = (np.arange(n) - edge) // MS_PER_BIT + 1           # bit 0 = partial bit
    sym = 1 - 2 * nav[np.clip(bit_idx, 0, len(nav) - 1)].astype(float)
    z = amp * sym * np.exp(1j * phase) + sigma * (rng.standard_normal(n) + 1j * rng.standard_normal(n))

    keep = rng.random(n) > 0.002                      # 0.2% random missed dumps
    keep[30000:30025] = False                         # plus one 25 ms USB stall
    idx = np.nonzero(keep)[0]

    def i24(v):
        return int(v) & 0xFFFFFF

    run = tempfile.mkdtemp(prefix="pll_selftest_")
    with open(os.path.join(run, "prompts.bin"), "wb") as f:
        for k in idx:
            noise = rng.normal(0, sigma, 4)
            vals = [noise[0], noise[1], z[k].real, z[k].imag, noise[2], noise[3]]
            rec = b"".join(i24(round(v)).to_bytes(3, "little") for v in vals) + bytes([k & 0xFF])
            assert len(rec) == RECORD_BYTES
            f.write(rec)
    with open(os.path.join(run, "track.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "unix_time", "prn", "doppler_hz"])
        w.writerow([0, 0, 10, 2721.0])

    print(f"self-test: {secs} s at {cn0} dB-Hz, f = {f_start} Hz + {f_rate} Hz/s, "
          f"bit edges at {edge} ms, TOW from {tow0 + 1}  ->  {run}\n")
    r = analyse(run, pll_bw, fll_bw, fll_secs, plot)

    f_true_end = f_start + f_rate * (t[-1] - 0.1)          # centre of the last 200 ms
    tows = [s["tow"] for s in r["subframes"]]
    expect = list(range(tow0 + 1, tow0 + 1 + 12))
    checks = [
        ("frequency at end within 1 Hz", abs(r["f_end"] - f_true_end) < 1.0,
         f"{r['f_end']:+.2f} vs {f_true_end:+.2f}"),
        ("bit edge offset", r["offset"] == edge, f"{r['offset']} vs {edge}"),
        ("subframes decoded", len(tows) >= 10, f"{len(tows)} of 12"),
        ("TOW values all correct", set(tows) <= set(expect) and tows == sorted(tows), str(tows)),
        # extracted bit k starts at ms edge + 20k, which is nav bit k + 1
        ("subframes at the right bit positions", bool(r["subframes"]) and all(
            s["bit"] == first_sf_bit - 1 + BITS_PER_SUBFRAME * (s["tow"] - (tow0 + 1))
            for s in r["subframes"]), ", ".join(str(s["bit"]) for s in r["subframes"])),
    ]
    print()
    for name, ok, info in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {info}")
    shutil.rmtree(run, ignore_errors=True)            # synthetic data: don't leave it lying around
    sys.exit(0 if all(ok for _, ok, _ in checks) else 1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run_dir", nargs="?")
    ap.add_argument("--ch", type=int, nargs="+", help="channel(s) to process (default: all)")
    # 40 Hz: the first real recording (PRN 32, 2026-11-25) had carrier wander that a
    # 15 Hz loop kept slipping on (44% lock, 0 subframes); 40 Hz held 90-100% and
    # decoded 6 subframes. Synthetic self-tests pass at 38-45 dB-Hz either way.
    ap.add_argument("--pll-bw", type=float, default=40.0, help="PLL noise bandwidth, Hz")
    ap.add_argument("--fll-bw", type=float, default=10.0, help="FLL bandwidth during pull-in, Hz")
    ap.add_argument("--fll-secs", type=float, default=1.0, help="FLL-only pull-in time, s")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--cn0", type=float, default=45.0, help="self-test signal strength, dB-Hz")
    a = ap.parse_args()
    if a.selftest:
        selftest(a.cn0, a.plot, a.pll_bw, a.fll_bw, a.fll_secs)
    elif a.run_dir:
        chans = a.ch if a.ch else run_channels(a.run_dir)
        if not chans:
            sys.exit(f"no prompts*.bin in {a.run_dir}")
        for ch in chans:
            analyse(a.run_dir, a.pll_bw, a.fll_bw, a.fll_secs, a.plot, ch=ch)
    else:
        ap.error("give a run folder, or --selftest")


if __name__ == "__main__":
    main()
