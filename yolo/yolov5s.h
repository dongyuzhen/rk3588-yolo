#ifndef YOLOV5S_H
#define YOLOV5S_H

#include <iostream>
#include <string.h>
#include <vector>
#include <cstddef>

#include "3rdparty/librknn_api/include/rknn_api.h"

#include "RgaUtils.h"
#include "im2d.h"
#include "rga.h"

#include "post_process.h"



class Yolov5s
{
private:
    rknn_context context;
    unsigned int model_size;

    rknn_input_output_num num_tensors;
    std::vector<rknn_tensor_attr> input_attrs;
    std::vector<rknn_tensor_attr> output_attrs;

    unsigned char *model_data;
    unsigned char *load_model(const char *model_path, unsigned int &model_size);

    // 中间缓冲：YUYV→RGB 色彩转换用，由 rknn_create_mem 分配，fd 可直接被 RGA 使用
    rknn_tensor_mem *mid_mem_{nullptr};

    // 持久化 RKNN IO 缓冲：构造时分配一次，整个生命周期复用，避免每帧 alloc/free 引起 cache miss
    rknn_tensor_mem *input_mem_{nullptr};
    std::vector<rknn_tensor_mem *> output_mems_;

public:
    Yolov5s(const char *model_path, int npu_index, int img_w = 640, int img_h = 480);
    ~Yolov5s();

    // 模型要求的输入尺寸（由 rknn_query 填充）
    int model_height;
    int model_width;
    int model_channel;

    // 摄像头原始尺寸（与采集分辨率一致，由构造函数传入）
    int img_height;
    int img_width;

    //  转换后的尺寸
    int resized_height;
    int resized_width;

    // 转换因子
    float scale;

    // x与y的偏移量
    int pad_left, pad_right, pad_top, pad_bottom;

    // 缓存 RGA handle：mid_mem_ 和 input_mem_ 的 fd 从不变化，构造时 import 一次
    rga_buffer_handle_t mid_handle_{0};
    rga_buffer_handle_t model_handle_{0};

    // 推理：接收 V4L2 DMABUF fd（YUYV422，img_width×img_height）
    // RGA 将其转换+缩放为 RGB888（model_width×model_height）后送 NPU
    int inference_image(int dmabuf_fd, detect_result_group_t &result_group);

    // 在 Mat 上绘制检测框（调用方负责提供原始分辨率的 BGR Mat）
    int draw_result(cv::Mat &orig_img, detect_result_group_t &result_group);

    // 对比 RGA 与 OpenCV 预处理速度，iterations 为重复次数
    void benchmark_preprocess(int dmabuf_fd, int iterations = 100);
};

#endif
