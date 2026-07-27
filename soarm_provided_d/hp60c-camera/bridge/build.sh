#!/usr/bin/env bash
# HP60C shm_bridge 빌드 스크립트
#
# SDK 위치 우선순위:
#   1) $HP60C_SDK_DIR 환경변수
#   2) 저장소에 번들된 ../sdk (기본)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEFAULT_SDK="$(cd "$SCRIPT_DIR/.." && pwd)/sdk"
SDK_DIR="${HP60C_SDK_DIR:-$DEFAULT_SDK}"

if [[ ! -f "$SDK_DIR/libs/include/as_camera_sdk_api.h" ]]; then
    cat >&2 <<EOF
SDK를 찾을 수 없습니다: $SDK_DIR

번들된 sdk/ 디렉토리가 비어있다면(LFS 미적용 클론 등) 별도 경로를 지정하세요:
  export HP60C_SDK_DIR=/path/to/hp60c_sdk/linux_ros/linux
  $0
EOF
    exit 1
fi

mkdir -p "$SCRIPT_DIR/build"
cd "$SCRIPT_DIR/build"
cmake -DHP60C_SDK_DIR="$SDK_DIR" ..
make -j"$(nproc)"
echo
echo "빌드 완료: $SCRIPT_DIR/build/shm_bridge"
