# 픽앤플레이스 실습 — 차시별 시작 파일

학생이 자기 작업폴더 (`~/soarm-lab/`) 에 `soarm.py` 와 함께 놓고
실행하는 차시별 스타터 파일들.

| 파일 | 차시 | 내용 |
|---|---|---|
| `1_joint_feel.py`  | 1 | 관절 6개 감 잡기 — 어느 관절이 어디 움직이는지 |
| `2_teach_read.py`  | 2 | 손으로 자세 만들고 각도 출력 |
| `3_first_pose.py`  | 3 | 측정한 각도를 dict 로 만들어 가게 시키기 |
| `3b_gripper.py`    | 3 | 그리퍼 raw 위치 + 자기 큐브 맞는 CLOSE 찾기 |
| `4_poses.py`       | 4 | 5자세 모두 정의 (template) |
| `5_pick_place.py`  | 5 | 시퀀스 조립 (template, TODO 채우기) |

자세한 설명: [../../../docs/PICK_AND_PLACE.md](../../../docs/PICK_AND_PLACE.md)

## 학생 작업폴더 권장 배치

강사가 알려준 방법으로 `soarm.py` + 이 차시 파일들 받아오면:

```
~/soarm-lab/
├── soarm.py            ← HTTP 클라이언트 (수정 X)
├── 1_joint_feel.py
├── 2_teach_read.py
├── 3_first_pose.py
├── 3b_gripper.py
├── poses.py            ← 4_poses.py 를 이 이름으로 두면 5_pick_place 가 import
└── pick_place.py       ← 5_pick_place.py 를 이 이름으로 두면 명령어 짧아짐
```

## 실행

각 파일 안의 `TEAM` 값을 자기 팀 컨트롤러 URL 로 바꾼 다음:

```bash
python3 1_joint_feel.py
python3 2_teach_read.py
python3 3_first_pose.py
python3 3b_gripper.py
python3 pick_place.py     # 4_poses.py 가 같은 폴더에 있어야 함
```

각 차시 파일 안의 **TODO 주석** 을 채우면서 진행. 차시 1~3 은 거의
바로 돌아가고, 4~5 는 본인이 측정/조립해야 완성됩니다.
