add_library(yolo_module STATIC
    yolov5s.cpp
    post_process.cpp
)

target_include_directories(yolo_module
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${RKNN_INCLUDE_DIRS}
        ${RGA_INCLUDE_DIRS}
)

target_link_libraries(yolo_module
    PUBLIC
        ${OpenCV_LIBS}
        ${RKNN_LIBS}
        ${RGA_LIBS}
)
