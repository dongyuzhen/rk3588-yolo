cmake_minimum_required(VERSION 3.16)

project(streamer_codev4 LANGUAGES C CXX)

# ── 性能基准测试开关 ──────────────────────────────────────────────
# cmake -DBENCH_MODE=ON  ..  → 启用逐帧计时 + 定期报告 + CSV 输出
# cmake -DBENCH_MODE=OFF ..  → 零开销，无任何计时代码
option(BENCH_MODE "Enable pipeline performance benchmarking" OFF)

set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-std=c++17) # 强制给编译器传递 c++17 参数，解决结构化绑定警告

message(STATUS "System: ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION}")

# ------------------------- 第三方依赖 -------------------------
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# FFmpeg（RTMP 推流所需）
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
    libavformat
    libavcodec
    libavutil
)

# MPP
find_library(MPP_LIBRARY NAMES rockchip_mpp mpp REQUIRED)
message(STATUS "find MPP library: ${MPP_LIBRARY}")

# ------------------------- 芯片 / 第三方路径 -------------------------
set(LIB_ARCH "aarch64")
set(DEVICE_NAME "RK3588")
set(THIRDPARTY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty")

# RKNN — 优先查找系统安装的，找不到再 fallback 到 3rdparty
set(RKNN_API_PATH "${THIRDPARTY_PATH}/librknn_api")
set(RKNN_INCLUDE_DIRS "${RKNN_API_PATH}/include")
find_library(RKNN_LIBS NAMES rknnrt
    PATHS "${RKNN_API_PATH}/${LIB_ARCH}"
    NO_DEFAULT_PATH
)
if(NOT RKNN_LIBS)
    message(FATAL_ERROR "librknnrt.so not found! \
        Place it at 3rdparty/librknn_api/aarch64/ or install system-wide.")
endif()
message(STATUS "find RKNN library: ${RKNN_LIBS}")

# RGA — 优先查找系统安装的，找不到再 fallback 到 3rdparty
set(RGA_PATH "${THIRDPARTY_PATH}/rga/${DEVICE_NAME}")
set(RGA_INCLUDE_DIRS "${RGA_PATH}/include")
find_library(RGA_LIBS NAMES rga
    PATHS "${RGA_PATH}/lib/Linux/${LIB_ARCH}"
    NO_DEFAULT_PATH
)
if(NOT RGA_LIBS)
    message(FATAL_ERROR "librga.so not found! \
        Place it at 3rdparty/rga/RK3588/lib/Linux/aarch64/ or install system-wide.")
endif()
message(STATUS "find RGA library: ${RGA_LIBS}")

# ------------------------- 子模块 -------------------------
add_subdirectory(mpp)
add_subdirectory(yolo)
add_subdirectory(pool)
add_subdirectory(streamer)

# ------------------------- 主可执行程序 -------------------------
add_executable(app
    main.cpp
    camera/v4l2_camera.cpp
    camera/cma_buffer.cpp
)

target_include_directories(app
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${RKNN_INCLUDE_DIRS}
        ${RGA_INCLUDE_DIRS}
)

target_link_libraries(app
    PRIVATE
        pool_module
        streamer_module
        ${OpenCV_LIBS}
        ${RKNN_LIBS}
        ${RGA_LIBS}
        PkgConfig::FFMPEG
        Threads::Threads
)

# ── 性能基准测试编译开关 ──────────────────────────────────────────
if(BENCH_MODE)
    target_compile_definitions(app PRIVATE BENCH_MODE)
    target_compile_definitions(yolo_module PRIVATE BENCH_MODE)
    message(STATUS ">>> BENCH_MODE = ON  (pipeline performance profiling enabled)")
else()
    message(STATUS ">>> BENCH_MODE = OFF (zero-overhead production build)")
endif()
