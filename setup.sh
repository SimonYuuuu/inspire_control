#!/usr/bin/env bash
# Create ./.venv and install mediapipe + opencv for hand tracking.
set -euo pipefail
cd "$(dirname "$0")"

PY=${PY:-python3}
if [[ ! -d .venv ]]; then
    "$PY" -m venv .venv
fi
./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install -r requirements.txt
echo
echo "done. run:  ./.venv/bin/python run_hand_tracking.py --list"
