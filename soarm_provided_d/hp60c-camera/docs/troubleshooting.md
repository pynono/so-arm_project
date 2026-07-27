# Troubleshooting

## `/dev/shm/hp60c_frames` 가 없다고 나온다
- `scripts/start_bridge.sh` 가 떠있는지 확인.
- bridge 콘솔에 `Streaming started.` 가 찍혔는지 확인.

## bridge 가 `Camera attached` 까지만 찍히고 멈춤
대부분 config 파일 매칭 실패.
- `HP60C_CONFIG_DIR` 가 `configurationfiles/` 를 가리키는지 확인.
- 디렉토리 안에 `hp60c_*.cfg` 또는 `vega_*.cfg` 가 실제로 있는지.
- bridge 는 `hp60c_` → `hp60cn_` → 첫 파일 순으로 fallback 하므로, 잘못된 config
  로 OpenCamera 가 실패하면 stream 이 시작 안 됨.

## bridge 빌드 시 `as_camera_sdk_api.h` 못 찾음
- 기본값은 저장소의 `sdk/` 디렉토리. 그 아래 `libs/include/as_camera_sdk_api.h` 가
  실제로 있는지 확인 (`ls hp60c_camera/sdk/libs/include/`).
- 별도 SDK 경로를 쓴다면 `HP60C_SDK_DIR` 이 `linux_ros/linux` 까지 들어가 있어야 한다
  (그 아래 `libs/include/`).

## bridge 실행 시 `libAngstrongCameraSdk.so: cannot open shared object file`
- `start_bridge.sh` 는 `LD_LIBRARY_PATH` 에 SDK lib 디렉토리를 추가해 실행한다. 직접 실행
  중이라면 같은 변수를 export 했는지 확인.

## OpenCV 로 직접 열어보고 싶다
열리지 않습니다. `cv2.VideoCapture(2)` 는 검은 프레임만 돌려준다 — 이게 이 저장소가 존재하는
이유. UVC depth 스트림이 표준화되어 있지 않다.

## USB 권한 (`Permission denied`)
udev rule 또는 `sudo` 로 실행. 대부분 카메라가 `/dev/video*` 에 만드는 노드는 `video` 그룹
소유라 `sudo usermod -aG video $USER` 로 충분.

## Python에서 `read()` 가 항상 (None, None, 0)
- `header()` 의 magic 이 0 이면 → bridge 가 아직 첫 프레임을 안 썼다.
- magic 은 정상인데 ready 플래그가 둘 다 0 이라면 → 카메라가 attach 됐지만 stream 이
  죽은 상태. bridge 콘솔의 detached 메시지 확인.

## 카메라가 USB 2.0 모드로 잡혀 프레임 레이트가 낮다
- 동봉 케이블이 USB 2.0 인 경우가 흔함. 짧은 USB 3.0 데이터 케이블로 교체 시 개선.
- 시리얼/모델은 `lsusb -v -d 3482:6723` 로 확인.

## 같은 컴퓨터에서 두 번째 reader 가 떠도 되는가
- Read-only mmap 이므로 동시 다수 reader OK. producer 는 bridge 한 개.
