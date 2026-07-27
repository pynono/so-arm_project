"""
차시 3 — 첫 자세를 코드로 정의하고 가게 시키기.

차시 2 에서 종이에 적어둔 각도를 HOME dict 에 박고 실행.
로봇이 그 자세로 자동으로 가야 함.
"""
from soarm import Robot

TEAM = "http://10.0.0.24:8000"


# ★ 본인이 차시 2 에서 적은 각도로 바꿔주세요 ★
HOME = {
    "base":         0.0,
    "shoulder":     0.0,
    "elbow":        0.0,
    "wrist_pitch":  0.0,
    "wrist_roll":   0.0,
}


bot = Robot(TEAM, name="me")
bot.torque(True)

bot.move_all(HOME, speed=150)
bot.wait_until_settled(tolerance_deg=2.0, timeout=5.0)
print("✓ HOME 도착")
