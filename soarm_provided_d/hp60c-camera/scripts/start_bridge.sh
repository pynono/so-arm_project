#!/usr/bin/env bash
# HP60C shm_bridge 실행
#
# SDK 위치 우선순위:
#   1) $HP60C_SDK_DIR
#   2) 저장소에 번들된 ../sdk
# 환경변수:
#   HP60C_CONFIG_DIR    카메라 설정 파일 디렉토리 (기본: $SDK_DIR/configurationfiles)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BRIDGE="$PKG_ROOT/bridge/build/shm_bridge"
DEFAULT_SDK="$PKG_ROOT/sdk"
SDK_DIR="${HP60C_SDK_DIR:-$DEFAULT_SDK}"

# 호스트 아키텍처에 맞는 lib 디렉토리 자동 선택
case "$(uname -m)" in
    x86_64|amd64)         SDK_ARCH="x86_64-linux-gnu" ;;
    aarch64|arm64)        SDK_ARCH="aarch64-linux-gnu" ;;
    armv7l|armv6l|arm)    SDK_ARCH="arm-linux-gnueabihf" ;;
    *) echo "지원하지 않는 아키텍처: $(uname -m)" >&2; exit 1 ;;
esac

SDK_LIB="$SDK_DIR/libs/lib/$SDK_ARCH"
CONFIG_DIR="${HP60C_CONFIG_DIR:-$SDK_DIR/configurationfiles}"

if [[ ! -x "$BRIDGE" ]]; then
    echo "shm_bridge 바이너리가 없습니다. 먼저 빌드하세요:" >&2
    echo "  cd $PKG_ROOT/bridge && ./build.sh" >&2
    exit 1
fi

if [[ ! -f "$SDK_LIB/libAngstrongCameraSdk.so" ]]; then
    echo "SDK 라이브러리를 찾을 수 없습니다: $SDK_LIB/libAngstrongCameraSdk.so" >&2
    exit 1
fi

echo "[start_bridge] SDK lib   : $SDK_LIB"
echo "[start_bridge] Config dir: $CONFIG_DIR"
echo "[start_bridge] Ctrl+C 로 종료"
echo

exec env LD_LIBRARY_PATH="$SDK_LIB:${LD_LIBRARY_PATH:-}" \
        "$BRIDGE" "$CONFIG_DIR"
