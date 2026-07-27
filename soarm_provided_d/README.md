# soarm-team-sort

SO-ARM101 + HP60C 비전으로 **빨강/파랑 공 분류** 미니 과제용 프로젝트.

원본 handout: `../soarm_provided_d` 에서 필요한 부분만 옮김.
`hp60c-camera` 는 용량 때문에 원본으로 **심볼릭 링크**.

## 환경

```bash
source /home/rookie/D053/.venv/bin/activate
export PYTHONPATH=/home/rookie/D053/soarm-team-sort:/home/rookie/D053/soarm-team-sort/hp60c-camera
```

시리얼 포트 기본: `/dev/ttyACM0` (`soarm_lab/real.py`)

## 카메라

```bash
./scripts/start_camera.sh          # 브리지
python vision/00_view.py           # 화면
./scripts/stop_camera.sh           # 종료 시 브리지도 끌 것
```

## 비전 파이프라인

| 스크립트 | 역할 |
|---------|------|
| `vision/00_view.py` | 카메라 프리뷰 |
| `vision/01_capture.py` | 샷 저장 |
| `vision/02_detect.py` | HSV 공 검출 |
| `vision/03_map.py` | 8점 캘리브 → `data/H.npy` |
| `vision/04_click_move.py` | 클릭 이동 |
| `vision/05_track.py` | 공 추적 |
| `vision/06_sort.py` | **분류 메인 (구현 예정)** |

## 데이터

- `data/H.npy` — 픽셀↔로봇 XY
- `data/calibration.json` — 서보 캘리브
