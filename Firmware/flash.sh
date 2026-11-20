#!/usr/bin/env bash
# Build and flash over USB DFU (no SWD on this board).
#
# If the board is running our firmware, it is told to reboot into DFU over the
# serial port (sends "dfu"), so no BOOT button is needed. If it is already in
# DFU mode, it is flashed directly. Manual fallback: hold BOOT (SW302), tap
# RESET (SW301), release BOOT.
set -euo pipefail
cd "$(dirname "$0")"
cargo build --release
OBJCOPY=$(ls ~/.rustup/toolchains/*/lib/rustlib/*/bin/llvm-objcopy | head -1)
"$OBJCOPY" -O binary target/thumbv7em-none-eabihf/release/gnss-fw target/gnss-fw.bin

in_dfu() {
    for d in /sys/bus/usb/devices/*/idVendor; do
        dir=$(dirname "$d")
        [ "$(cat "$d")" = 0483 ] && [ "$(cat "$dir/idProduct")" = df11 ] && return 0
    done
    return 1
}

if ! in_dfu; then
    port=${PORT:-/dev/ttyACM0}
    # Ask the running firmware to reboot into DFU; retry, since right after a
    # previous flash the app may not be listening yet.
    for attempt in 1 2 3 4 5; do
        if [ -e "$port" ]; then
            echo "Asking the firmware on $port to reboot into DFU (attempt $attempt)..."
            # pyserial raises DTR and waits for the write to go out; a plain shell
            # redirect to the tty did not reliably deliver the command.
            python3 - "$port" <<'EOF2' || true
import sys, time, serial
try:
    s = serial.Serial(sys.argv[1], 115200, timeout=0.2)
    s.dtr = True
    time.sleep(0.3)
    s.write(b"dfu\r")
    s.flush()
    time.sleep(0.3)
    s.close()
except Exception:
    pass  # the board disconnects as it reboots
EOF2
        fi
        for _ in $(seq 1 15); do in_dfu && break 2; sleep 0.2; done
    done
    in_dfu || { echo "DFU device did not appear: put the board in DFU mode manually (BOOT + RESET)."; exit 1; }
    sleep 0.5
fi

dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D target/gnss-fw.bin
