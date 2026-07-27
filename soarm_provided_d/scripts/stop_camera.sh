#!/bin/bash
pkill -f '/shm_bridge ' 2>/dev/null || true
pkill -f 'start_bridge.sh' 2>/dev/null || true
sleep 0.3
pgrep -a shm_bridge || echo "camera bridge stopped"
