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

using namespace std;

class Yolov5s
{
private:
    rknn_context context;
    unsigned int model_size;

    rknn_input_output_num num_tensors;
    vector<rknn_tensor_attr> input_attrs;
    vector<rknn_tensor_attr> output_attrs;

    unsigned char *model_data;
    unsigned char *load_model(const char *model_path, unsigned int &model_size);

    // 中间缓冲：YUYV→RGB 色彩转换用，由 rknn_create_mem 分配，fd 可直接被 RGA 使用
    rknn_tensor_mem *mid_mem_{nullptr};

public:
    Yolov5s(const char *model_path, int npu_index);
    ~Yolov5s();

    // 模型要求的输入尺寸（由 rknn_query 填充）
    int model_height;
    int model_width;
    int model_channel;

    // 摄像头原始尺寸（与采集分辨率一致）
    int img_height{480};
    int img_width{640};

    // 推理：接收 V4L2 DMABUF fd（YUYV422，img_width×img_height）
    // RGA 将其转换+缩放为 RGB888（model_width×model_height）后送 NPU
    int inference_image(int dmabuf_fd, detect_result_group_t &result_group);

    // 在 Mat 上绘制检测框（调用方负责提供原始分辨率的 BGR Mat）
    int draw_result(cv::Mat &orig_img, detect_result_group_t &result_group);
};

#endif
