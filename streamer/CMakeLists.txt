add_library(streamer_module STATIC
    rtsp.cpp
)

target_include_directories(streamer_module
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}
)

target_link_libraries(streamer_module
    PUBLIC
        mpp_module
        PkgConfig::FFMPEG
)
