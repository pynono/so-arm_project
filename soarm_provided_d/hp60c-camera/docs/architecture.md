# Architecture

## 왜 brige가 필요한가

HP60C / ASC60C(Angstrong 제조, Hiwonder 유통)는 USB로 꽂으면 NOVATEK USB camera
(VID:PID `3482:6723`)로 인식되고 `/dev/video2`(메인), `/dev/video3`(메타데이터) 노드를
만든다. 그러나:

- `/dev/video2` 를 OpenCV `cv2.VideoCapture(2)` 로 열면 **검은 화면**만 나온다 — UVC
  포맷 협상이 정상적으로 끝나지 않는다.
- depth 스트림은 표준 UVC 포맷에 매핑되지 않으므로 `v4l2-ctl` 로도 받을 수 없다.

제조사 C++ SDK (`libAngstrongCameraSdk.so`) 만이 RGB + Depth 양쪽을 정상적으로 동기화해
넘겨준다. SDK는 다음과 같은 비대칭이 있어 **그대로 ctypes 로 부르기는 어렵다**:

- 카메라 발견은 `AS_SDK_GetCameraList()` 가 아니라 **Listener 패턴** (`AS_SDK_StartListener`
  + `onAttached` 콜백) 으로만 동작.
- 데이터 콜백 시그니처가 `std::list<...>` 등 C++ 타입을 노출.
- Open 시 `configurationfiles/` 디렉토리에서 모델별 `.cfg` 파일을 자동 매칭해야 한다.

따라서 SDK 호출은 C++ 작은 바이너리(`shm_bridge`) 하나에 가두고, 다른 모든 코드는 OS 의
공유메모리만 읽는다.

## 데이터 흐름

```
HP60C ── USB ──▶ libAngstrongCameraSdk.so
                       │  (Listener → Stream callback)
                       ▼
                shm_bridge.cpp
                       │  memcpy
                       ▼
        /dev/shm/hp60c_frames  (POSIX shm, mmap)
                       │
            ┌──────────┼─────────────┐
            ▼          ▼             ▼
     Python        ROS2 노드      또 다른 프로세스
   (mmap+numpy)   (직접 mmap)    (read-only)
```

## SHM 메모리 레이아웃

| Offset | Type     | 의미                                    |
|------- |--------- |---------------------------------------- |
| 0      | uint32   | magic = `0x48503630` ("HP60")          |
| 4      | uint32   | rgb_w                                   |
| 8      | uint32   | rgb_h                                   |
| 12     | uint32   | rgb_size (bytes)                        |
| 16     | uint32   | depth_w                                 |
| 20     | uint32   | depth_h                                 |
| 24     | uint32   | depth_size (bytes)                      |
| 28     | uint32   | _pad                                    |
| 32     | uint64   | frame_id (단조증가)                     |
| 40     | uint64   | timestamp_us (steady_clock)             |
| 48     | uint32   | rgb_ready (0/1)                         |
| 52     | uint32   | depth_ready (0/1)                       |
| 64     | bytes    | RGB 데이터 (BGR uint8, 최대 1920×1080×3) |
| 64+MAX_RGB | bytes | Depth 데이터 (uint16 mm, 최대 640×480×2) |

전체 크기 = 64 + 1920·1080·3 + 640·480·2 ≈ 6.85 MB.

전형적인 출력 해상도는 **RGB 640×480 BGR**, **Depth 640×480 uint16(mm)**.
컨피그(`vega_*.cfg`)에 따라 RGB는 1920×1080까지 가능.

## 동기화 / 동시성

- **단일 producer**: shm_bridge 만 쓴다.
- 별도 락은 없다. 헤더의 `frame_id` 가 단조증가하므로 reader 는 변경 감지가 가능.
- 일반적인 사용에서 부분쓰기를 본 적은 없으나, 절대적인 atomicity 가 필요하면 더블버퍼
  (RGB/Depth 슬롯 2개 + slot 인덱스)를 추가하는 것이 정공.
- 헤더의 마지막 갱신은 `magic` 이 가장 마지막에 쓰여지지 않으므로, reader 는 magic 검증
  + frame_id 비교의 두 단계로 무효 프레임을 거른다.

## 캘리브레이션 / 좌표

bridge 는 `AS_SDK_GetCamParameter` 결과를 `/tmp/hp60c_params.txt` 에 저장한다 (IR / RGB
intrinsic + IR→RGB extrinsic translation `T1,T2,T3`). 이 패키지는 거기까지만 책임지고,
3D 변환이나 IR↔RGB 정합은 사용 측에서 한다.

기록된 전형적인 값(개체차 있음):

- IR  : fx=423.1, fy=422.7, cx=320.8, cy=235.8
- RGB : fx=568.8, fy=568.3, cx=331.6, cy=230.0
