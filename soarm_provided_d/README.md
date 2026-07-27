# SO-ARM101 비전 실습 (학생 배포본)

카메라(HP60C)로 공을 찾아 로봇 좌표로 바꾸고, 로봇이 그 자리로 가게 만드는 실습입니다.
**핵심은 직접 만듭니다.** 카메라·로봇 라이브러리와 어려운 배관(4점 캘리브)은 제공되고,
검출·좌표활용·트래킹은 여러분이 (필요하면 AI를 활용해) 만들되 **각 부분을 설명할 수 있어야** 합니다.

## 제공 vs 직접
| | 내용 |
|---|---|
| **제공** | `hp60c-camera/`(카메라) · `soarm_lab/`(로봇) · `vision/03_map.py`(4점 호모그래피) |
| **직접** | `vision/01_capture` · `02_detect` · `04_click_move` · `05_track` (스캐폴드에 요구사항·힌트) |

## 셋업 (최초 1회)
```bash
python3 -m venv .venv
source .venv/bin/activate

cd hp60c-camera && pip install -e ".[examples]"   # ① 카메라 리더 + opencv
cd ..            && pip install -e .              # ② soarm_lab (+ numpy·pyserial·mujoco)
```
- 새 컴퓨터면 ②에서 mujoco 다운로드로 몇 분 걸릴 수 있습니다(한 번만).
- 브리지 빌드가 필요하면: `cd hp60c-camera/bridge && rm -rf build && ./build.sh`

## 매번 실습할 때
```bash
source .venv/bin/activate
./hp60c-camera/scripts/start_bridge.sh     # 카메라 브리지('Streaming started')
# ↑ 켜두고, 새 터미널(venv 다시 activate)에서 아래 실행
cd vision
python 01_capture.py     # 공 사진        (카메라만)
python 02_detect.py      # HSV 검출        (카메라만 / 사진만도 가능)
python 03_map.py         # 4점 → data/H.npy (카메라 + 로봇, 제공됨)
python 04_click_move.py  # 클릭 → 이동      (카메라 + 로봇 + H.npy)
python 05_track.py       # 트래킹          (카메라 + 로봇 + H.npy)
```



## 흐름 한눈에
`공 사진(01)` → `HSV로 공의 픽셀(02)` → `4점 호모그래피로 H(03)` → `픽셀→로봇좌표로 이동(04)` → `트래킹(05)`


## 안전
로봇이 움직이는 03·04·05 는 **작업면에 손을 넣지 말 것.** 처음엔 `REAL=False`(시뮬)로 확인한 뒤 실물로.
