"""
04 — Use poses you saved in the web UI.

In the browser at http://teamN.local:8000 → "Teach & Poses" tab, position
the arm and save a pose with a name like "home" or "pick". This script
goes through whatever poses currently exist on the team's controller.
"""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from soarm import Robot

TEAM = "team1"

with Robot(TEAM, name="me") as bot:
    poses = bot.poses()
    if not poses:
        print("No poses saved yet. Open the web UI and save one first.")
        sys.exit(0)

    print("Saved poses:", list(poses))
    for name in poses:
        print(f"→ {name}")
        bot.go_pose(name, speed=300)
        bot.wait_until_settled(timeout=5.0)
        time.sleep(0.5)
