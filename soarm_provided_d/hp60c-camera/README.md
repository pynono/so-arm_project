# hp60c-camera

Hiwonder가 유통하고 **Angstrong 이 제조한 HP60C / ASC60C 뎁스카메라**를 Linux에서
**RGB + Depth 둘 다** 받아 쓰기 위한 독립 패키지입니다. 단독으로도 동작하며,
다른 프로젝트에는 `git clone` 또는 폴더 복사 후 `pip install -e .` 로 가져다 씁니다.

이 카메라는 USB로 잡히고 `/dev/video*` 노드도 만들지만, **UVC 표준을 따르지 않아 OpenCV
`VideoCapture` 같은 일반 도구로는 검은 화면만 나옵니다.** 제조사 C++ SDK를 거쳐야 하는데,
SDK가 `std::list`/콜백을 그대로 노출하는 C++ API라 Python에서 `ctypes`만으로는 깔끔히
못 쓰입니다.

이 패키지는 그 사이를 이렇게 메웁니다:

```
   [HP60C USB]
        │
        ▼
  ┌───────────────────────┐
  │ shm_bridge (C++ 데몬) │   Angstrong SDK Listener 패턴으로 카메라 핸들 획득
  │   - SDK 콜백 수신      │   매 프레임 RGB(BGR uint8) + Depth(uint16 mm) 를
  │   - mmap에 프레임 쓰기 │   /dev/shm/hp60c_frames 에 동기화
  └─────────┬─────────────┘
            │  POSIX shared memory (/dev/shm/hp60c_frames)
            ▼
  ┌───────────────────────┐
  │ hp60c_camera (Python) │   numpy + mmap 으로 zero-copy 읽기
  │   CameraReader.read() │   다른 언어에서도 같은 SHM 레이아웃으로 접근 가능
  └───────────────────────┘
```

설계 의도:

- 카메라 SDK 의존성을 **C++ 바이너리 하나에만 가둔다.** Python 쪽은 numpy 만 있으면 됨.
- 여러 프로세스가 동시에 `read()` 가능 (read-only mmap).
- shm_bridge 가 죽어도 다음 실행에서 같은 경로로 다시 붙기만 하면 됨.
- 다른 언어/노드(예: ROS2, Rust)에서도 같은 `/dev/shm/hp60c_frames` 를 그대로 읽을 수 있다.

## 디렉토리 구조

```
hp60c-camera/                # 저장소 루트
├── bridge/                  # C++ shm_bridge (SDK -> SHM)
│   ├── CMakeLists.txt
│   ├── shm_bridge.cpp
│   └── build.sh
├── sdk/                     # 번들된 Angstrong HP60C SDK (libs + configurationfiles)
├── hp60c_camera/            # Python 패키지 (SHM -> numpy) — import 이름은 hp60c_camera
│   ├── __init__.py
│   └── reader.py
├── examples/
│   ├── 01_open_camera.py
│   ├── 02_rgb_and_depth.py
│   └── 03_rgb_depth_view.py
├── scripts/
│   └── start_bridge.sh
├── docs/
│   ├── architecture.md
│   └── troubleshooting.md
├── LICENSE
└── pyproject.toml
```

## 사전 준비

### 1. SDK

저장소의 `sdk/` 디렉토리에 Angstrong HP60C SDK가 함께 들어있습니다 (헤더 +
x86_64 / aarch64 / armv7 .so + configurationfiles). **별도 다운로드 불필요.**

다른 SDK 버전을 쓰고 싶다면 환경변수로 덮어쓸 수 있습니다:

```bash
export HP60C_SDK_DIR=/path/to/hp60c_sdk/linux_ros/linux
```

### 2. shm_bridge 빌드 (C++)

```bash
cd bridge
./build.sh
# -> bridge/build/shm_bridge 생성
```

호스트 아키텍처(x86_64 / aarch64 / armv7)는 자동 인식됩니다.

### 3. Python 의존성

```bash
pip install -e .                     # numpy 만 사용
pip install -e ".[examples]"         # OpenCV 예제까지
```

## 실행

```bash
# 터미널 A: 카메라를 열고 SHM 에 프레임을 흘린다 (포그라운드 실행)
./scripts/start_bridge.sh

# 터미널 B: 프레임을 읽어서 화면에 표시
python examples/01_open_camera.py
python examples/02_rgb_and_depth.py
python examples/03_rgb_depth_view.py
```

`shm_bridge` 가 카메라를 인식하면 콘솔에 다음과 같이 찍힙니다:

```
[shm_bridge] Camera attached, model type: 9
[shm_bridge] Using config: .../configurationfiles/vega_xxxx.cfg
[shm_bridge] IR : fx=423.1 fy=422.7 cx=320.8 cy=235.8
[shm_bridge] RGB: fx=568.8 fy=568.3 cx=331.6 cy=230.0
[shm_bridge] Streaming started.
```

`/tmp/hp60c_params.txt` 에 IR/RGB intrinsic 도 같이 기록됩니다.

## Python API

```python
from hp60c_camera import CameraReader

with CameraReader() as cam:
    rgb, depth, frame_id = cam.read()           # 즉시
    rgb, depth, frame_id = cam.read_blocking()  # 새 프레임 도착까지 대기
    # rgb   : np.uint8  (H, W, 3)  BGR
    # depth : np.uint16 (H, W)     mm; 0 = invalid
```

자세한 SHM 레이아웃과 동작 원리는 [docs/architecture.md](docs/architecture.md) 참고.
문제가 생기면 [docs/troubleshooting.md](docs/troubleshooting.md) 부터 보세요.

## 라이선스

이 저장소에는 출처가 다른 두 종류의 파일이 섞여 있습니다.

| 대상 | 라이선스 |
|------|----------|
| `bridge/`, `hp60c_camera/`, `examples/`, `scripts/`, `docs/` (직접 작성한 코드) | **MIT** (루트 `LICENSE`) |
| `sdk/` 이하 전체 (헤더 · `.so` · configurationfiles) | **Angstrong 독점 SDK** — 제조사 약관 적용 |

`sdk/` 는 빌드·실행 편의를 위해 번들했을 뿐이며, 본 저장소의 MIT 라이선스가
적용되지 않습니다. 따라서 이 저장소는 **private 상태에서 HP60C 하드웨어 보유자에게
배포**하는 것을 전제로 합니다. 저장소를 public 으로 전환하거나 외부에 fork · 재배포할
경우 `sdk/` 를 제외해야 할 수 있으니, 그 전에 Angstrong/Hiwonder 의 SDK 사용 약관을
확인하세요.
