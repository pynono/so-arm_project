"""
00 — Hello, robot.

The smallest possible script. Connects to your team's controller, prints
who's currently driving, and reads the joint angles. No motion.

Run:
    python3 00_hello_robot.py
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from soarm import Robot

# ⬇️ Change this to your team's controller URL.
#   Same PC as the server     →  "http://localhost:8000"
#   Different PC, IP known    →  "http://192.168.0.42:8000"
#   Different PC, mDNS works  →  "team1"   (= http://team1.local:8000)
TEAM = "http://localhost:8000"

bot = Robot(TEAM, name="me")

print("URL          :", bot.base_url)
print("Server says  :", bot.info().get("team") or "(no team label)")
print("Me           :", bot.me())
print("Connected?   :", bot.status().connected)
print("Joint angles :", bot.angles())
