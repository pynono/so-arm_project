#!/usr/bin/env bash
# First-time install for SO-ARM101 Web Controller (Linux / macOS).
#
#   ./scripts/install.sh
#
# Creates a local venv/ and installs Python deps. Nothing is installed
# globally.
set -euo pipefail

cd "$(dirname "$0")/.."

PY="${PYTHON:-python3}"

if ! command -v "$PY" >/dev/null 2>&1; then
  echo "python3 is required but not found in PATH."
  exit 1
fi

echo "[install] creating venv/"
"$PY" -m venv venv

echo "[install] upgrading pip"
./venv/bin/python -m pip install --upgrade pip

echo "[install] installing requirements"
./venv/bin/python -m pip install -r requirements.txt

# Linux: help students avoid sudo when talking to the servos
if [ "$(uname)" = "Linux" ]; then
  if ! groups | grep -qE '(^|\s)dialout(\s|$)'; then
    echo
    echo "[install] NOTE: your user is not in the 'dialout' group."
    echo "          To access /dev/ttyACM* without sudo, run:"
    echo "            sudo usermod -aG dialout \$USER"
    echo "          then log out and back in."
  fi
fi

echo
echo "[install] done. Run ./run.sh to start the controller."
