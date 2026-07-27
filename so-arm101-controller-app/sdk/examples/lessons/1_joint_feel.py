"""
차시 1 — 관절 6개 감 잡기.

각 줄 한 번씩 실행해보면서 어느 관절이 어디로 움직이는지 관찰.
한 줄 돌리고 → 로봇 어디로 가는지 보고 → 다음 줄.
"""
import time
from soarm import Robot

TEAM = "http://10.0.0.24:8000"   # ← 강사가 알려준 IP 로 바꾸기

bot = Robot(TEAM, name="me")
bot.torque(True)


# ── 베이스 회전 ────────────────────────────────────────────
bot.move("base", 30)
time.sleep(2)
bot.move("base", 0)
time.sleep(1)


# ── TODO: 아래 주석 풀고 다른 관절도 시도 ────────────────────
# bot.move("shoulder",    -30); time.sleep(2); bot.move("shoulder",    0)
# bot.move("elbow",        30); time.sleep(2); bot.move("elbow",       0)
# bot.move("wrist_pitch",  20); time.sleep(2); bot.move("wrist_pitch", 0)
# bot.move("wrist_roll",   45); time.sleep(2); bot.move("wrist_roll",  0)


# ── 관찰 결과를 종이에 적어두기 ───────────────────────────
# base        + 면 어디로?  - 면 어디로?
# shoulder    ...
# elbow       ...
# wrist_pitch ...
# wrist_roll  ...
