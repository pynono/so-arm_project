#!/usr/bin/env bash
# SO-ARM101 Controller launcher (Linux / macOS).
#
# Usage:
#   ./run.sh                  # default, opens http://127.0.0.1:8000
#   ./run.sh --port 9000      # different port
#   ./run.sh --serial /dev/ttyUSB0
set -euo pipefail

cd "$(dirname "$0")"

if [ ! -d "venv" ]; then
  echo "venv/ not found — running ./scripts/install.sh first"
  ./scripts/install.sh
fi

# shellcheck disable=SC1091
source venv/bin/activate
exec python3 app.py "$@"
