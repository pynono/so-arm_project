# Examples

| 파일 | 설명 |
| --- | --- |
| `01_open_camera.py` | 가장 작은 예제. RGB 한 창만 띄운다. shm_bridge 연결과 프레임 읽기를 보여준다. |
| `02_rgb_and_depth.py` | RGB와 Depth를 동시에 표시. Depth는 JET 컬러맵, 마우스 포인터 픽셀의 mm 값 오버레이. |

실행 전, 다른 터미널에서 bridge 가 떠 있어야 합니다.

```bash
# 터미널 A
export HP60C_SDK_DIR=$HOME/hp60c_sdk/linux_ros/linux
./scripts/start_bridge.sh

# 터미널 B
python examples/01_open_camera.py
```
