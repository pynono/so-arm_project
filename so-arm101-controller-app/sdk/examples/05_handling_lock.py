"""
05 — When the instructor revoked your access.

By default any client on the team can drive. The instructor (the main
PC operator) can revoke a specific client from the admin panel. While
revoked, every write call raises PermissionDeniedError.

This script shows the polite pattern: catch the error, wait, retry.
"""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from soarm import Robot, PermissionDeniedError

TEAM = "team1"

bot = Robot(TEAM, name="me")
bot.connect()

while True:
    try:
        bot.move("base", 10)
        print("Moved.")
        break
    except PermissionDeniedError as e:
        print(f"{e} — waiting 5 s for the instructor to restore access…")
        time.sleep(5)

bot.center()
