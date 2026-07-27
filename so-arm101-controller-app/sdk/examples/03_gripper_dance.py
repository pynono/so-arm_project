"""
03 — Gripper open / close.

The gripper (joint 6) has its own physical end stops. Sending the joint
toward the negative side closes it; positive opens it. The server clamps
to safe limits, so feel free to overshoot — nothing crashes.
"""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from soarm import Robot

TEAM = "team1"

with Robot(TEAM, name="me") as bot:
    for _ in range(3):
        print("open")
        bot.move("gripper",  60, speed=400)
        time.sleep(0.7)
        print("close")
        bot.move("gripper", -60, speed=400)
        time.sleep(0.7)
    bot.move("gripper", 0)
