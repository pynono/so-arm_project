cmake_minimum_required(VERSION 3.10)
project(hp60c_shm_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# HP60C SDK 경로
#   1순위: -DHP60C_SDK_DIR=... 옵션
#   2순위: $HP60C_SDK_DIR 환경변수
#   3순위: 저장소 안에 번들된 ../sdk 디렉토리 (기본)
if(NOT DEFINED HP60C_SDK_DIR)
    set(HP60C_SDK_DIR "$ENV{HP60C_SDK_DIR}")
endif()
if(NOT HP60C_SDK_DIR)
    set(HP60C_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../sdk")
endif()
get_filename_component(HP60C_SDK_DIR "${HP60C_SDK_DIR}" ABSOLUTE)
message(STATUS "HP60C_SDK_DIR = ${HP60C_SDK_DIR}")

# 호스트 아키텍처에 맞는 lib 디렉토리 선택
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
    set(SDK_ARCH "x86_64-linux-gnu")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(SDK_ARCH "aarch64-linux-gnu")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|armv7)")
    set(SDK_ARCH "arm-linux-gnueabihf")
else()
    message(FATAL_ERROR "지원하지 않는 아키텍처: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
message(STATUS "SDK arch = ${SDK_ARCH}")

set(SDK_INC "${HP60C_SDK_DIR}/libs/include")
set(SDK_LIB "${HP60C_SDK_DIR}/libs/lib/${SDK_ARCH}")

if(NOT EXISTS "${SDK_INC}/as_camera_sdk_api.h")
    message(FATAL_ERROR "SDK 헤더를 찾을 수 없습니다: ${SDK_INC}/as_camera_sdk_api.h")
endif()
if(NOT EXISTS "${SDK_LIB}/libAngstrongCameraSdk.so")
    message(FATAL_ERROR "SDK 라이브러리를 찾을 수 없습니다: ${SDK_LIB}/libAngstrongCameraSdk.so")
endif()

add_executable(shm_bridge shm_bridge.cpp)
target_include_directories(shm_bridge PRIVATE ${SDK_INC})
target_link_directories(shm_bridge PRIVATE ${SDK_LIB})
target_link_libraries(shm_bridge
    AngstrongCameraSdk
    pthread
    rt
)

# 런타임에 libAngstrongCameraSdk.so 를 찾도록 RPATH 설정
set_target_properties(shm_bridge PROPERTIES
    BUILD_RPATH   "${SDK_LIB}"
    INSTALL_RPATH "${SDK_LIB}"
)
