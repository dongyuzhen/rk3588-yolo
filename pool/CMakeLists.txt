add_library(pool_module STATIC
    thread_poll.cpp
)

target_include_directories(pool_module
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(pool_module
    PUBLIC
        yolo_module
        Threads::Threads
)
