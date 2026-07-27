"""
차시 5 — 픽앤플레이스 시퀀스 완성.

전제: 같은 폴더에 poses.py (= lessons/4_poses.py) 가 있어야 함.
    학생 작업폴더 권장 배치:
        ~/soarm-lab/
            soarm.py
            poses.py        ← 차시 4 의 4_poses.py 를 이 이름으로
            pick_place.py   ← 이 파일을 이 이름으로

TODO 4개 채우면 완성. 막힐 때 docs/PICK_AND_PLACE.md 차시 5 참고.
"""
import time
from soarm import Robot
from poses import (
    HOME, ABOVE_PICK, PICK, ABOVE_PLACE, PLACE,
    GRIPPER_OPEN, GRIPPER_CLOSE,
)

TEAM = "http://10.0.0.24:8000"


# ── 도우미 (바꾸지 않아도 됨) ─────────────────────────────
def goto(bot, pose, speed=180):
    """포즈로 이동 + 도착할 때까지 대기."""
    bot.move_all(pose, speed=speed)
    bot.wait_until_settled(tolerance_deg=2.0, timeout=5.0)


def grip(bot, raw, dwell=0.4):
    """그리퍼 열거나 닫고 잠시 대기."""
    bot.move_raw("gripper", raw, speed=300)
    time.sleep(dwell)


# ── 시퀀스 — 학생이 채울 영역 ─────────────────────────────
def main():
    bot = Robot(TEAM, name="me")
    bot.torque(True)

    # TODO 1) 시작 — HOME 에서 그리퍼 열고 출발
    # goto(bot, HOME)
    # grip(bot, GRIPPER_OPEN)

    # TODO 2) 픽업 — above_pick → pick (천천히) → 그리퍼 닫기
    # goto(bot, ABOVE_PICK)
    # goto(bot, PICK, speed=100)
    # grip(bot, GRIPPER_CLOSE, dwell=0.6)

    # TODO 3) 들어올리기 — 같은 ABOVE_PICK 으로 수직 상승
    # ...

    # TODO 4) 이동 + 놓기 — above_place → place → 그리퍼 열기
    # ...

    # 복귀 — above_place 거쳐서 home
    # ...

    print("✓ done.")


if __name__ == "__main__":
    main()
