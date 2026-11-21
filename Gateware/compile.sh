#!/usr/bin/env bash
# Build the FPGA bitstream: synth (yosys) -> place & route (nextpnr) -> pack (ecppack).
# Output: build/top.bit
set -euo pipefail

# Run from the directory this script lives in, wherever it's called from.
cd "$(dirname "$0")"

# Put oss-cad-suite on PATH if it isn't already.
if ! command -v yosys >/dev/null; then
    source "$HOME/tools/oss-cad-suite/environment"
fi

mkdir -p build

# `check -assert` turns problems like multiple drivers on one signal into a hard error.
# It has to run before synth_ecp5, which would otherwise silently delete the broken logic.
yosys -p "read_verilog -sv top.sv; hierarchy -check -top top; proc; check -assert; synth_ecp5 -top top -json build/top.json" \
    -l build/yosys.log

nextpnr-ecp5 --45k --package TQFP144 --speed 6 \
    --lpf board.lpf \
    --json build/top.json \
    --textcfg build/top.config \
    --log build/nextpnr.log

ecppack --compress build/top.config build/top.bit

echo "Built build/top.bit"
