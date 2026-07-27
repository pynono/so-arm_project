"""
차시 4 — 5개 자세 모두 정의.

이 파일은 라이브러리처럼 5_pick_place.py 가 import 해서 쓸 거예요:

    from poses import HOME, ABOVE_PICK, PICK, ABOVE_PLACE, PLACE
    from poses import GRIPPER_OPEN, GRIPPER_CLOSE

차시 2 의 방법으로 4 자세를 더 측정해서 채우세요.
실행할 일은 없습니다 — 데이터 파일이에요.
"""


# ═══════════════════════════════════════════════════════════════════
#  자세 5개
# ═══════════════════════════════════════════════════════════════════

HOME = {
    "base":         0.0,
    "shoulder":     0.0,
    "elbow":        0.0,
    "wrist_pitch":  0.0,
    "wrist_roll":   0.0,
}

ABOVE_PICK = {
    # 잡을 큐브 바로 위 ~5cm, 그리퍼가 큐브를 좌우로 감쌀 위치
    "base":         0.0,
    "shoulder":     0.0,
    "elbow":        0.0,
    "wrist_pitch":  0.0,
    "wrist_roll":   0.0,
}

PICK = {
    # ⚠ base/shoulder/elbow 는 ABOVE_PICK 과 거의 같아야 함 (수직 강하)
    "base":         0.0,
    "shoulder":     0.0,
    "elbow":        0.0,
    "wrist_pitch":  0.0,
    "wrist_roll":   0.0,
}

ABOVE_PLACE = {
    # 놓을 자리 바로 위 ~5cm
    "base":         0.0,
    "shoulder":     0.0,
    "elbow":        0.0,
    "wrist_pitch":  0.0,
    "wrist_roll":   0.0,
}

PLACE = {
    # 놓을 자리, 바닥 근처
    "base":         0.0,
    "shoulder":     0.0,
    "elbow":        0.0,
    "wrist_pitch":  0.0,
    "wrist_roll":   0.0,
}


# ═══════════════════════════════════════════════════════════════════
#  그리퍼 raw 위치 (차시 3 에서 찾은 값)
# ═══════════════════════════════════════════════════════════════════

GRIPPER_OPEN  = 2318
GRIPPER_CLOSE = 984    # ← 자기 큐브에 맞춰 차시 3 에서 정한 값으로
