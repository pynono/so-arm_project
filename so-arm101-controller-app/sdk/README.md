# SO-ARM101 학생용 Python SDK

자기 노트북에서 팀 로봇을 파이썬으로 조작하기 위한 작은 클라이언트입니다.

## 1. 요구사항

```bash
pip install requests
```

## 2. 첫 연결

팀 PC가 같은 와이파이에 켜져 있고 컨트롤러가 떠 있어야 합니다 (강사가 미리
세팅). 그 다음 자기 노트북에서:

```python
from soarm import Robot

bot = Robot("team1", name="홍길동")    # → http://team1.local:8000
bot.connect()                          # 메인PC가 USB 시리얼 연결
print(bot.angles())                    # 현재 각 관절 각도(°)
```

`team1.local`이 안 풀리면 IP 직접 입력:

```python
bot = Robot("http://192.168.0.42:8000", name="홍길동")
```

## 3. 핵심 명령

```python
bot.move("elbow", 30)                  # 관절 하나
bot.move(2, 30)                        # 같은 명령 (id=2 = shoulder)
bot.move_all({"shoulder": 0, "elbow": 0})
bot.center()                           # 모두 가운데
bot.go_pose("home")                    # 웹 UI에서 저장한 포즈로

bot.lock()                             # 모든 관절 토크 ON
bot.unlock()                           # 토크 OFF (손으로 움직일 수 있음)

print(bot.status())                    # 상세 상태
```

각도는 **0°가 기계적 중앙**, 양수/음수가 양방향. 그리퍼는 정/부가 열림/닫힘.

## 4. 권한 모델 (4명 같이 쓸 때)

기본은 **누구나 운전 가능**. 같은 팀 4명이 동시에 명령을 보낼 수도 있고,
사회적으로 "한 명만 잡고 하기" 정도로 합의하면 충분해요.

문제가 생기면 강사(메인PC 운영자)가 admin 패널에서 특정 학생의 권한을
회수할 수 있습니다. 그 학생의 `bot.move(...)`는 `PermissionDeniedError`로
실패해요.

```python
from soarm import Robot, PermissionDeniedError

bot = Robot("team1", name="me")
bot.connect()

try:
    bot.move("elbow", 0)
except PermissionDeniedError:
    print("강사한테 권한 풀어달라고 부탁하세요.")
```

`with` 구문도 됩니다 — connect 자동, 별도 release는 필요 없음:

```python
with Robot("team1", name="me") as bot:
    bot.move("elbow", 0)
```

## 5. 예제

[`examples/`](examples/) 안에 5개 스크립트가 있어요. 순서대로 실행해보세요:

| 파일 | 설명 |
|----|----|
| `00_hello_robot.py` | 연결 테스트 (모터는 안 움직임) |
| `01_move_one_joint.py` | 한 관절 흔들기 |
| `02_sequence_loop.py` | 여러 관절 동시 + 정착 대기 |
| `03_gripper_dance.py` | 그리퍼 여닫기 |
| `04_play_saved_pose.py` | 저장된 포즈 순회 |
| `05_handling_lock.py` | 권한 박탈 시 폴링 패턴 |

각 파일 맨 위의 `TEAM = "team1"` 한 줄만 자기 팀으로 바꾸면 됩니다.

## 6. 자주 마주치는 에러

| 에러 | 원인 / 해결 |
|----|----|
| `ControllerError: Cannot reach ...` | 팀 PC가 안 켜졌거나, 와이파이가 다른 망. `ping team1.local` 로 먼저 확인. |
| `PermissionDeniedError: ...` | 강사가 권한을 회수한 상태. 4번 참고. |
| `move()` 직후 위치가 안 바뀜 | `bot.connect()` 호출 안 했거나, 해당 관절 토크가 OFF. `bot.lock()` 한 번. |
| `bot.move(7, ...)` 같은 잘못된 id | 1~6만 유효. 또는 `"base"`, `"elbow"` 등 이름. |
