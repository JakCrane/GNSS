#!/usr/bin/env bash
# Outdoor test: keeps the laptop awake for the whole run, then runs field.py.
#   ./field.sh                    # default ~10 min
#   ./field.sh --pll-secs 300     # any field.py options pass straight through
set -euo pipefail
cd "$(dirname "$0")"
if command -v systemd-inhibit >/dev/null; then
    exec systemd-inhibit --what=sleep:idle:handle-lid-switch --why="GNSS field test" \
        python3 field.py "$@"
else
    echo "(systemd-inhibit not found: make sure the laptop won't sleep during the run)"
    exec python3 field.py "$@"
fi
