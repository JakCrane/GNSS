#!/usr/bin/env python3
"""Decode GPS LNAV ephemeris (subframes 1-3) from a track.py run folder, and
compute satellite positions from it.

Bits come from, per channel, in order of preference:
  prompts_pll_ch<n>.bin  - the board's own PLL was locked: sign(I_P) is the data
  pll_out_ch<n>.npz      - pll.py's offline PLL on the DLL-only recording

Per satellite it needs one subframe 1, 2 and 3 with matching IODE/IODC (same
ephemeris issue). Writes ephemeris.json in the run folder and prints the
decoded orbit + sanity checks (orbit radius, inclination, all satellites above
one common horizon).

Usage:
  python3 ephemeris.py RUN_DIR [--source pll|dll|auto] [-v]
  python3 ephemeris.py --selftest

Field layout and orbit equations: IS-GPS-200 20.3.3.3 / 20.3.3.4,
Tables 20-I, 20-III, 20-IV.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pll import (MS_PER_BIT, BITS_PER_SUBFRAME, PREAMBLE, bit_sync, extract_bits,  # noqa: E402
                 find_subframes, gps_time_str, parity_ok, encode_word, load_run, run_channels)

# ---------------------------------------------------------------- constants (IS-GPS-200)
MU = 3.986005e14               # m^3/s^2, WGS-84 value used by GPS
OMEGA_E = 7.2921151467e-5      # rad/s, Earth rotation
F_REL = -4.442807633e-10       # s/m^0.5, relativistic clock term
PI = 3.1415926535898           # the spec's own value of pi (semicircles -> rad)
HALF_WEEK = 302400

# ---------------------------------------------------------------- field table
# name: (subframe, [(first bit, n bits), ...] MSB first, signed, scale)
# Bit numbers are 1..300 within the subframe; each word is 30 bits, data in its
# first 24. Split fields (e.g. M0 = 8 MSBs in one word + 24 LSBs in the next)
# list both pieces.
SC = PI          # semicircle fields -> radians
FIELDS = {
    # subframe 1: clock, health, week
    "wn":       (1, [(61, 10)], False, 1),
    "ura":      (1, [(73, 4)], False, 1),
    "health":   (1, [(77, 6)], False, 1),
    "iodc":     (1, [(83, 2), (211, 8)], False, 1),
    "tgd":      (1, [(197, 8)], True, 2**-31),
    "toc":      (1, [(219, 16)], False, 2**4),
    "af2":      (1, [(241, 8)], True, 2**-55),
    "af1":      (1, [(249, 16)], True, 2**-43),
    "af0":      (1, [(271, 22)], True, 2**-31),
    # subframe 2: orbit part 1
    "iode2":    (2, [(61, 8)], False, 1),
    "crs":      (2, [(69, 16)], True, 2**-5),
    "dn":       (2, [(91, 16)], True, 2**-43 * SC),
    "m0":       (2, [(107, 8), (121, 24)], True, 2**-31 * SC),
    "cuc":      (2, [(151, 16)], True, 2**-29),
    "e":        (2, [(167, 8), (181, 24)], False, 2**-33),
    "cus":      (2, [(211, 16)], True, 2**-29),
    "sqrt_a":   (2, [(227, 8), (241, 24)], False, 2**-19),
    "toe":      (2, [(271, 16)], False, 2**4),
    "fit":      (2, [(287, 1)], False, 1),
    # subframe 3: orbit part 2
    "cic":      (3, [(61, 16)], True, 2**-29),
    "omega0":   (3, [(77, 8), (91, 24)], True, 2**-31 * SC),
    "cis":      (3, [(121, 16)], True, 2**-29),
    "i0":       (3, [(137, 8), (151, 24)], True, 2**-31 * SC),
    "crc":      (3, [(181, 16)], True, 2**-5),
    "omega":    (3, [(197, 8), (211, 24)], True, 2**-31 * SC),
    "omegadot": (3, [(241, 24)], True, 2**-43 * SC),
    "iode3":    (3, [(271, 8)], False, 1),
    "idot":     (3, [(279, 14)], True, 2**-43 * SC),
}


def get_field(sf_bits: np.ndarray, pieces, signed: bool) -> int:
    """Raw integer from 1-based bit pieces of a parity-corrected subframe."""
    v, n = 0, 0
    for first, length in pieces:
        for b in sf_bits[first - 1:first - 1 + length]:
            v = (v << 1) | int(b)
        n += length
    if signed and v & (1 << (n - 1)):
        v -= 1 << n
    return v


def corrected_subframe(hard: np.ndarray, start: int, inv: int) -> np.ndarray:
    """300 bits with each word's data un-inverted by the previous word's D30
    (parity bits kept as sent), so field bit numbers can be used directly."""
    sf = (hard[start:start + BITS_PER_SUBFRAME] ^ inv).copy()
    d30s = 0                                       # word 10 of the previous subframe ends in 00
    for w in range(10):
        word = sf[30 * w:30 * w + 30]
        word[:24] ^= d30s
        d30s = int(hard[start + 30 * w + 29] ^ inv)
    return sf


# ---------------------------------------------------------------- bits from a run
def soft_bits_fw_pll(run: str, ch: int):
    """20 ms bit sums of I_P from the board's PLL stage. Missed dumps (from the
    dump counter) become zeros, so a gap doesn't shift the bit boundaries."""
    from track import load_prompts
    p = load_prompts(os.path.join(run, f"prompts_pll_ch{ch}.bin"))
    step = np.diff(p["dumps"].astype(np.int64)) % 256
    step[step == 0] = 256
    idx = np.concatenate([[0], np.cumsum(step)])
    I = np.zeros(idx[-1] + 1)
    I[idx] = p["ip"]
    have = np.zeros(len(I), bool)
    have[idx] = True
    off, frac = bit_sync(I, have)
    soft, _ = extract_bits(I, off)
    return soft, f"board PLL, bit sync {frac:.0%}, {len(I) - len(idx)} missed dumps filled"


def soft_bits_dll(run: str, ch: int):
    d = np.load(os.path.join(run, f"pll_out_ch{ch}.npz"))
    return d["bits"], "offline PLL (pll_out npz)"


def channel_prns(run: str) -> dict:
    out = {}
    for ch in run_channels(run):
        _, _, prn, _ = load_run(run, ch)
        out[ch] = prn
    return out


# ---------------------------------------------------------------- decode
def decode_soft(soft: np.ndarray, verbose=False) -> dict:
    """Newest complete ephemeris (subframes 1, 2, 3 with matching issue) in a
    stream of soft bits. Returns {} if none."""
    hard = (np.asarray(soft) > 0).astype(np.uint8)
    sfs = find_subframes(soft)
    by_id = {1: [], 2: [], 3: []}
    for s in sfs:
        if s["id"] in by_id:
            bits = corrected_subframe(hard, s["bit"], s["inverted"])
            by_id[s["id"]].append((s, bits))
    # walk back from the newest subframe 1 to find a matching 2 and 3
    for s1, b1 in reversed(by_id[1]):
        raw = {n: get_field(b1, f[1], f[2]) for n, f in FIELDS.items() if f[0] == 1}
        for s2, b2 in reversed(by_id[2]):
            r2 = {n: get_field(b2, f[1], f[2]) for n, f in FIELDS.items() if f[0] == 2}
            if r2["iode2"] != raw["iodc"] & 0xFF:
                continue
            for s3, b3 in reversed(by_id[3]):
                r3 = {n: get_field(b3, f[1], f[2]) for n, f in FIELDS.items() if f[0] == 3}
                if r3["iode3"] != r2["iode2"]:
                    continue
                raw.update(r2)
                raw.update(r3)
                eph = {n: raw[n] * FIELDS[n][3] for n in FIELDS}
                for n in ("wn", "ura", "health", "iodc", "iode2", "iode3", "fit"):
                    eph[n] = int(raw[n])
                eph["tow_sf1"] = s1["tow"] * 6 - 6        # transmit time of subframe 1 start
                if verbose:
                    print(f"    using subframes at bits {s1['bit']}, {s2['bit']}, {s3['bit']}")
                return eph
    return {}


# ---------------------------------------------------------------- orbit
def wrap_week(dt: float) -> float:
    if dt > HALF_WEEK:
        dt -= 2 * HALF_WEEK
    elif dt < -HALF_WEEK:
        dt += 2 * HALF_WEEK
    return dt


def sat_clock(eph: dict, t: float, ek: float = None) -> float:
    """Satellite clock offset (s) at GPS time-of-week t, incl. relativity (no TGD)."""
    dt = wrap_week(t - eph["toc"])
    corr = eph["af0"] + eph["af1"] * dt + eph["af2"] * dt * dt
    if ek is not None:
        corr += F_REL * eph["e"] * eph["sqrt_a"] * math.sin(ek)
    return corr


def sat_position(eph: dict, t: float):
    """ECEF position (m) at GPS time-of-week t (transmit time), and eccentric
    anomaly E (for the clock's relativistic term). IS-GPS-200 Table 20-IV."""
    a = eph["sqrt_a"] ** 2
    n = math.sqrt(MU / a ** 3) + eph["dn"]
    tk = wrap_week(t - eph["toe"])
    mk = eph["m0"] + n * tk
    ek = mk
    for _ in range(30):                              # Kepler: E - e sin E = M
        ek_new = mk + eph["e"] * math.sin(ek)
        if abs(ek_new - ek) < 1e-13:
            ek = ek_new
            break
        ek = ek_new
    e = eph["e"]
    vk = math.atan2(math.sqrt(1 - e * e) * math.sin(ek), math.cos(ek) - e)
    phi = vk + eph["omega"]
    s2, c2 = math.sin(2 * phi), math.cos(2 * phi)
    uk = phi + eph["cus"] * s2 + eph["cuc"] * c2
    rk = a * (1 - e * math.cos(ek)) + eph["crs"] * s2 + eph["crc"] * c2
    ik = eph["i0"] + eph["idot"] * tk + eph["cis"] * s2 + eph["cic"] * c2
    xp, yp = rk * math.cos(uk), rk * math.sin(uk)
    om = eph["omega0"] + (eph["omegadot"] - OMEGA_E) * tk - OMEGA_E * eph["toe"]
    x = xp * math.cos(om) - yp * math.cos(ik) * math.sin(om)
    y = xp * math.sin(om) + yp * math.cos(ik) * math.cos(om)
    z = yp * math.sin(ik)
    return np.array([x, y, z]), ek


def ecef_to_latlon(r):
    """Spherical lat/lon (deg) - only for the rough sanity check."""
    x, y, z = r
    return math.degrees(math.atan2(z, math.hypot(x, y))), math.degrees(math.atan2(y, x))


# ---------------------------------------------------------------- report
def describe(prn, eph, src):
    print(f"PRN {prn:2d}  ({src})")
    print(f"    week {eph['wn']} (mod 1024)  health {eph['health']}  URA {eph['ura']}  "
          f"IODC {eph['iodc']}  IODE {eph['iode2']}")
    print(f"    toc {eph['toc']:.0f} s  af0 {eph['af0'] * 1e6:+.3f} us  af1 {eph['af1']:+.3e}  "
          f"af2 {eph['af2']:+.1e}  TGD {eph['tgd'] * 1e9:+.2f} ns")
    print(f"    toe {eph['toe']:.0f} s ({gps_time_str(int(eph['toe'] // 6))})  "
          f"sqrtA {eph['sqrt_a']:.4f}  e {eph['e']:.6f}  i0 {math.degrees(eph['i0']):.3f} deg")


def sanity(ephs: dict, t: float):
    print(f"\nsatellite positions at GPS time {t:.0f} s of week ({gps_time_str(int(t // 6))}):")
    ok = True
    units = []
    for prn, eph in ephs.items():
        r, ek = sat_position(eph, t)
        dt = sat_clock(eph, t, ek)
        rad = np.linalg.norm(r)
        lat, lon = ecef_to_latlon(r)
        units.append(r / rad)
        checks = []
        if not 26.0e6 < rad < 27.2e6:
            checks.append("radius off")
        if not 0 <= eph["e"] < 0.03:
            checks.append("eccentricity off")
        if not 50 < math.degrees(eph["i0"]) < 60:
            checks.append("inclination off")
        if abs(eph["af0"]) > 1e-3:
            checks.append("clock offset > 1 ms")
        if eph["toe"] % 16:
            checks.append("toe not a multiple of 16 s")
        if eph["health"]:
            checks.append("UNHEALTHY flag set")
        ok &= not checks
        print(f"  PRN {prn:2d}  |r| {rad / 1e3:8.1f} km  ECEF ({r[0] / 1e3:9.1f}, {r[1] / 1e3:9.1f}, "
              f"{r[2] / 1e3:9.1f}) km  sub-sat {lat:+6.1f}, {lon:+7.1f}  clk {dt * 1e6:+8.3f} us  "
              + ("ok" if not checks else "<-- " + ", ".join(checks)))
    if len(units) >= 2:
        # All tracked satellites were in view at once, so there must be a spot on Earth
        # that sees all of them above the horizon. Use the mean direction as that spot.
        u = np.mean(units, axis=0)
        u /= np.linalg.norm(u)
        site = u * 6371e3
        print("  common-horizon check (from the mean direction - expect every elevation > 0):")
        for (prn, eph), _ in zip(ephs.items(), units):
            r, _ = sat_position(eph, t)
            los = r - site
            el = math.degrees(math.asin(np.dot(los, u) / np.linalg.norm(los)))
            ok &= el > 0
            print(f"    PRN {prn:2d}  elevation {el:+5.1f} deg")
    print("\nall checks passed" if ok else "\nSOME CHECKS FAILED")
    return ok


def analyse(run: str, source: str, verbose: bool):
    prns = channel_prns(run)
    if not prns:
        sys.exit(f"no channels in {run}")
    ephs, srcs = {}, {}
    for ch, prn in prns.items():
        tries = {"pll": [soft_bits_fw_pll], "dll": [soft_bits_dll],
                 "auto": [soft_bits_fw_pll, soft_bits_dll]}[source]
        for fn in tries:
            try:
                soft, src = fn(run, ch)
            except FileNotFoundError:
                continue
            eph = decode_soft(soft, verbose)
            if eph:
                ephs[prn], srcs[prn] = eph, f"ch{ch}, {src}"
                break
            print(f"ch{ch} PRN {prn}: no complete 1-2-3 set from {src}")
        else:
            if prn not in ephs:
                print(f"ch{ch} PRN {prn}: no ephemeris")
    for prn, eph in ephs.items():
        describe(prn, eph, srcs[prn])
    if not ephs:
        return
    t = max(e["tow_sf1"] for e in ephs.values())
    sanity(ephs, t)
    path = os.path.join(run, "ephemeris.json")
    with open(path, "w") as f:
        json.dump({str(p): e for p, e in ephs.items()}, f, indent=1)
    print(f"wrote {path}")


# ---------------------------------------------------------------- selftest
def encode_subframes(eph: dict, tow_sf1: int, rng) -> np.ndarray:
    """Subframes 1, 2, 3 (900 bits) carrying `eph`, with real parity."""
    data = [rng.integers(0, 2, 300).astype(np.uint8) for _ in range(3)]
    for n, (sf, pieces, signed, scale) in FIELDS.items():
        v = int(round(eph[n] / scale))
        nb = sum(p[1] for p in pieces)
        v &= (1 << nb) - 1
        bits = [(v >> (nb - 1 - k)) & 1 for k in range(nb)]
        for first, length in pieces:
            data[sf - 1][first - 1:first - 1 + length] = bits[:length]
            bits = bits[length:]
    out, d29s, d30s = [], 0, 0
    for k, d in enumerate(data):
        d[0:8] = PREAMBLE
        tow = (tow_sf1 // 6) + k + 1
        d[30:47] = [(tow >> (16 - j)) & 1 for j in range(17)]
        d[49:52] = [((k + 1) >> (2 - j)) & 1 for j in range(3)]
        for w in range(10):
            d24 = d[30 * w:30 * w + 24].copy()
            if w in (1, 9):                           # pick t bits so D29 = D30 = 0
                for t in range(4):
                    d24[22], d24[23] = t >> 1, t & 1
                    word = encode_word(d24, d29s, d30s)
                    if word[28] == 0 and word[29] == 0:
                        break
            else:
                word = encode_word(d24, d29s, d30s)
            assert parity_ok(word, d29s, d30s)
            out.append(word)
            d29s, d30s = int(word[28]), int(word[29])
    return np.concatenate(out)


def selftest():
    rng = np.random.default_rng(1)
    ref = {"wn": 391, "ura": 0, "health": 0, "iodc": 0x123, "tgd": -1.1641532e-08,
           "toc": 7200 * 3, "af2": 0.0, "af1": -2.6147972e-12, "af0": 3.2456312e-4,
           "iode2": 0x23, "crs": -54.15625, "dn": 4.3e-9, "m0": 2.1, "cuc": -2.8e-6,
           "e": 0.0123, "cus": 7.6e-6, "sqrt_a": 5153.65, "toe": 7200 * 3, "fit": 0,
           "cic": 1.1e-7, "omega0": -1.3, "cis": -5.2e-8, "i0": 0.962, "crc": 250.3,
           "omega": -1.7, "omegadot": -8.1e-9, "iode3": 0x23, "idot": 3.0e-10}
    # quantise the reference the way the encoder will
    q = {n: round(ref[n] / FIELDS[n][3]) * FIELDS[n][3] for n in FIELDS}
    bits = np.concatenate([rng.integers(0, 2, 137).astype(np.uint8),
                           encode_subframes(q, 21000, rng),
                           rng.integers(0, 2, 50).astype(np.uint8)])
    bits = np.concatenate([np.zeros(0, np.uint8), bits])
    ok = True
    for inv in (0, 1):
        soft = np.where((bits ^ inv) > 0, 1.0, -1.0)
        eph = decode_soft(soft)
        if not eph:
            print(f"inv={inv}: decode FAILED")
            ok = False
            continue
        bad = [n for n in FIELDS if not math.isclose(eph[n], q[n], rel_tol=1e-12, abs_tol=1e-30)]
        print(f"inv={inv}: {len(FIELDS) - len(bad)}/{len(FIELDS)} fields round-trip"
              + (f", wrong: {bad}" if bad else "") + f", tow_sf1 {eph['tow_sf1']}")
        ok &= not bad and eph["tow_sf1"] == 21000
    r, ek = sat_position(q, q["toe"] + 100)
    rad = np.linalg.norm(r)
    print(f"orbit radius at toe+100 s: {rad / 1e3:.1f} km (a = {q['sqrt_a'] ** 2 / 1e3:.1f} km)")
    ok &= abs(rad - q["sqrt_a"] ** 2) < 0.02 * q["sqrt_a"] ** 2
    # an orbit takes half a sidereal day: position repeats in ECI, and the speed is ~3.9 km/s
    r2, _ = sat_position(q, q["toe"] + 101)
    v_ecef = np.linalg.norm(r2 - r)
    print(f"ECEF speed: {v_ecef / 1e3:.2f} km/s (expect ~3 to 4)")
    ok &= 2.5e3 < v_ecef < 4.5e3
    print("SELFTEST PASSED" if ok else "SELFTEST FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", nargs="?")
    ap.add_argument("--source", choices=["auto", "pll", "dll"], default="auto",
                    help="bits from the board PLL recording, the offline PLL, or whichever works")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(0 if selftest() else 1)
    if not a.run:
        ap.error("give a run folder (or --selftest)")
    analyse(a.run, a.source, a.verbose)


if __name__ == "__main__":
    main()
