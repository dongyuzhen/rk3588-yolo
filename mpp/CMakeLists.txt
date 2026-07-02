add_library(mpp_module STATIC
    mpp.c
    mpp_encoder.cpp
)

target_include_directories(mpp_module
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mpp_module
    PUBLIC
        ${MPP_LIBRARY}
)

# mpp.h 依赖 <rockchip/rk_mpi.h>，通常位于系统 include 路径
# 若你的环境不在默认路径，可在这里补充 include 目录
