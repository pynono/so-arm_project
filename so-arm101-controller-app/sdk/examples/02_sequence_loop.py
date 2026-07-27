"""
02 — A small sequence with a loop.

Demonstrates move_all() (multiple joints in one round-trip) and waiting for
motion to settle before the next step.
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from soarm import Robot

TEAM = "team1"

WAYPOINTS = [
    {"shoulder":  20, "elbow": -20},
    {"shoulder": -10, "elbow":  30},
    {"shoulder":   0, "elbow":   0},
]

with Robot(TEAM, name="me") as bot:
    bot.center()
    bot.wait_until_settled()

    for i, wp in enumerate(WAYPOINTS, 1):
        print(f"Step {i}: {wp}")
        bot.move_all(wp, speed=350)
        bot.wait_until_settled(tolerance_deg=2.0, timeout=4.0)

    bot.center()
