"""
차시 3 (그리퍼) — 그리퍼 열고 닫기 + 자기 큐브에 맞는 CLOSE 값 찾기.

   984 ━━━━━━━━━━━━━━━━━━━━━━━ 2318
      (꽉 닫힘)            (활짝 열림)

자기 큐브 사이에 두고 닫아보면서 잡히는 CLOSE 값을 찾으세요.
찾은 값은 종이에 적어두기 (차시 5 에 쓸 거).
"""
import time
from soarm import Robot

TEAM = "http://10.0.0.24:8000"

bot = Robot(TEAM, name="me")
bot.torque(True)

# 활짝 열기
print("→ 활짝 열기")
bot.move_raw("gripper", 2318, speed=300)
time.sleep(0.8)

# 꽉 닫기
print("→ 꽉 닫기")
bot.move_raw("gripper", 984, speed=300)
time.sleep(0.8)


# ── TODO: 자기 큐브에 맞는 CLOSE 값 찾기 ──────────────────
#
#   1) 그리퍼를 활짝 연 상태에서, 사이에 큐브 끼우기
#   2) 아래 값을 바꿔가며 실행해보면서 적당히 잡히는 값 찾기
#
#       값 작게 (예: 950) = 더 꽉 닫힘  (작은 큐브에 OK)
#       값 크게 (예: 1050) = 덜 닫힘    (큰 큐브에 OK)
#
# bot.move_raw("gripper", 1000, speed=300)
# time.sleep(1.0)
#
# 자기 큐브 맞는 값:  GRIPPER_CLOSE = ____
