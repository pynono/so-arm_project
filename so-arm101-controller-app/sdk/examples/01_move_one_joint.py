"""
01 — Move one joint.

Connects, claims the soft lock, swings the elbow ±20° a few times, releases.
"""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from soarm import Robot

TEAM = "team1"

with Robot(TEAM, name="me") as bot:        # __enter__: auto-connect
    for angle in (-20, 0, 20, 0):
        print(f"elbow → {angle:+d}°")
        bot.move("elbow", angle, speed=300)
        time.sleep(1.0)
    bot.center()
