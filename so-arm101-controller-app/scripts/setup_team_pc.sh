#!/usr/bin/env bash
#
# One-shot setup for a classroom team PC running Ubuntu/Debian.
#
# What it does (idempotent — re-running is safe):
#   1. Sets the host name to teamN so students can reach it as teamN.local
#   2. Installs avahi-daemon (mDNS broadcaster)
#   3. Adds the current user to `dialout` (USB serial access)
#   4. Runs the project's install.sh (creates venv, installs Python deps)
#   5. Installs a systemd service so the controller starts on boot
#
# Usage (run on the team PC, NOT a student laptop):
#   sudo ./scripts/setup_team_pc.sh team1
#
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "Usage: sudo $0 <team-name>     # e.g. sudo $0 team1"
  exit 1
fi

TEAM="$1"
if [[ ! "$TEAM" =~ ^[a-z0-9-]{2,16}$ ]]; then
  echo "Team name must be 2-16 chars [a-z0-9-]. Got: $TEAM"
  exit 1
fi

if [ "$EUID" -ne 0 ]; then
  echo "This script must run with sudo. Re-run as: sudo $0 $TEAM"
  exit 1
fi

# Resolve the project root (parent of this scripts/ directory).
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Real (non-root) user that invoked sudo — we want the venv owned by them
# and dialout added to them, not root.
OWNER="${SUDO_USER:-$(logname)}"
OWNER_HOME="$(getent passwd "$OWNER" | cut -d: -f6)"

echo "──────────────────────────────────────────"
echo " SO-ARM101 team PC setup"
echo " Team:    $TEAM"
echo " Project: $ROOT"
echo " Owner:   $OWNER"
echo "──────────────────────────────────────────"

# ── 1. Hostname ───────────────────────────────────────────────────────────
CUR_HOST="$(hostname)"
if [ "$CUR_HOST" != "$TEAM" ]; then
  echo "[1/5] Renaming host: $CUR_HOST → $TEAM"
  hostnamectl set-hostname "$TEAM"
  # Keep /etc/hosts consistent so sudo and tools don't whine.
  if grep -qE "^\s*127\.0\.1\.1\s" /etc/hosts; then
    sed -i -E "s|^(\s*127\.0\.1\.1\s+).*|\1$TEAM|" /etc/hosts
  else
    echo -e "127.0.1.1\t$TEAM" >> /etc/hosts
  fi
else
  echo "[1/5] Hostname already $TEAM ✓"
fi

# ── 2. mDNS (avahi) ───────────────────────────────────────────────────────
echo "[2/5] Installing avahi-daemon for mDNS"
DEBIAN_FRONTEND=noninteractive apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq avahi-daemon python3-venv >/dev/null
systemctl enable --now avahi-daemon >/dev/null

# ── 3. Serial port permission ─────────────────────────────────────────────
if id -nG "$OWNER" | tr ' ' '\n' | grep -qx dialout; then
  echo "[3/5] $OWNER already in dialout ✓"
else
  echo "[3/5] Adding $OWNER to dialout (log out / in for it to take effect)"
  usermod -aG dialout "$OWNER"
fi

# ── 4. Python venv + deps (run as the owner, not root) ────────────────────
echo "[4/5] Installing Python deps"
sudo -u "$OWNER" bash -c "cd '$ROOT' && ./scripts/install.sh"
sudo -u "$OWNER" bash -c "cd '$ROOT' && ./venv/bin/pip install -q requests httpx"

# ── 5. systemd service ────────────────────────────────────────────────────
SERVICE=/etc/systemd/system/soarm-controller.service
echo "[5/5] Writing $SERVICE"
cat > "$SERVICE" <<EOF
[Unit]
Description=SO-ARM101 web controller (team $TEAM)
After=network-online.target avahi-daemon.service
Wants=network-online.target

[Service]
Type=simple
User=$OWNER
WorkingDirectory=$ROOT
Environment=SOARM_TEAM=$TEAM
ExecStart=$ROOT/venv/bin/python $ROOT/app.py --host 0.0.0.0 --port 8000
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable soarm-controller.service
systemctl restart soarm-controller.service

echo
echo "──────────────────────────────────────────"
echo " Done. Controller will start on boot."
echo
echo " Students on the same Wi-Fi can reach it at:"
echo "   http://$TEAM.local:8000"
echo
echo " Useful commands:"
echo "   sudo systemctl status  soarm-controller    # is it running?"
echo "   sudo systemctl restart soarm-controller    # after a git pull"
echo "   sudo journalctl -fu    soarm-controller    # tail the log"
echo "──────────────────────────────────────────"
