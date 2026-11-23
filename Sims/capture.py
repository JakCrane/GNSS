#!/usr/bin/env python3
"""Grab one FPGA sample capture over USB serial and save it as raw bytes.

Opening the port (DTR high) makes the firmware run a capture and print it as
hex lines between "capture: begin" and "capture: end".

Each byte holds 4 samples, oldest in the top two bits. Each 2-bit sample is
{mag, sign} = {I0, I1} (the order set in top.sv).

Usage:  python3 capture.py [port] [output file]
        defaults: /dev/ttyACM0, capture.bin
"""
import sys

import serial  # pip install pyserial

EXPECTED_BYTES = 32768


def grab(port: str) -> bytes:
    data = bytearray()
    started = False
    with serial.Serial(port, timeout=5) as s:
        s.dtr = True
        for raw in s:
            line = raw.decode(errors="ignore").strip()
            if line == "capture: failed":
                sys.exit("firmware reported capture: failed")
            if line == "capture: begin":
                started = True
                continue
            if line == "capture: end":
                break
            if started:
                data += bytes.fromhex(line)
    if not started:
        sys.exit("never saw 'capture: begin' (timed out)")
    return bytes(data)


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    out = sys.argv[2] if len(sys.argv) > 2 else "capture.bin"

    data = grab(port)
    with open(out, "wb") as f:
        f.write(data)
    print(f"saved {len(data)} bytes to {out}")
    if len(data) != EXPECTED_BYTES:
        print(f"warning: expected {EXPECTED_BYTES} bytes")

    # Quick sanity check: sign should be ~50% ones, magnitude ~33% (AGC target).
    sign = mag = 0
    for b in data:
        for shift in (6, 4, 2, 0):
            pair = (b >> shift) & 0b11
            mag += pair >> 1
            sign += pair & 1
    n = len(data) * 4
    if n:
        print(f"sign ones: {100 * sign / n:.1f}%   mag ones: {100 * mag / n:.1f}%")


if __name__ == "__main__":
    main()
