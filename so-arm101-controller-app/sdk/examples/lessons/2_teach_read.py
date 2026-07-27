"""
차시 2 — 자세 알아내기 (Teach + Read).

토크 풀고 → 로봇을 손으로 원하는 자세로 만들고 → 그 자세의 각도 출력.
출력된 6개 숫자를 종이에 적어두세요. 다음 차시에 코드에 박을 거예요.
"""
from soarm import Robot

TEAM = "http://10.0.0.24:8000"   # ← 강사가 알려준 IP

bot = Robot(TEAM, name="me")

# 1) 토크 풀기 — 손으로 자유롭게 움직일 수 있게.
#    ⚠ 풀리는 순간 중력으로 처질 수 있어요. 한 손으로 받치기.
print("토크 풀림. 로봇을 원하는 자세로 손으로 옮기세요.")
print("준비되면 Enter →")
bot.torque(False)
input()

# 2) 현재 각도 출력
print()
print("현재 각도:", bot.angles())
print()
print("→ 이 숫자들을 종이에 적어두세요.")

# 3) 다시 잠그기 (안 그러면 중력으로 처짐)
bot.torque(True)
