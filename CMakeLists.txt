set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
cmake_minimum_required(VERSION 3.16)

project(streamer_codev4 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

# RKNN
set(RKNN_API_PATH "${THIRDPARTY_PATH}/librknn_api")
set(RKNN_INCLUDE_DIRS "${RKNN_API_PATH}/include")
set(RKNN_LIBS "${RKNN_API_PATH}/${LIB_ARCH}/librknnrt.so")

# RGA（Rockchip Graphic Accelerator）
set(RGA_PATH "${THIRDPARTY_PATH}/rga/${DEVICE_NAME}")
set(RGA_INCLUDE_DIRS "${RGA_PATH}/include")
set(RGA_LIBS "${RGA_PATH}/lib/Linux/${LIB_ARCH}/librga.so")

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
