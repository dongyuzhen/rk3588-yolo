#ifndef YOLOV5S_H
#define YOLOV5S_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <string.h>
#include <vector>
#include <cstddef>
#include "3rdparty/librknn_api/include/rknn_api.h"

#include "RgaUtils.h"
#include "im2d.h"
#include "rga.h"

#include "post_process.h"

// #include "3rdparty/rga/RK3588/include/im2d_version.h"
// #include "3rdparty/rga/RV110X/include/im2d_type.h"
// #include "3rdparty/rga/RK3588/include/im2d_buffer.h"

using namespace std;
using namespace cv;
class Yolov5s
{
private:
    rknn_context context;  // 关键点：此处必须与 rknn_api.h 中的定义一致
    unsigned int model_size;

    rknn_tensor_attr input_tensor;
    rknn_tensor_attr output_tensor;
    rknn_input_output_num num_tensors;

    vector<rknn_tensor_attr> input_attrs;
    vector<rknn_tensor_attr> output_attrs;

    // 预处理缓冲（复用，避免每帧 malloc/free）
    std::vector<unsigned char> src_buf_;
    std::vector<unsigned char> src_cvt_buf_;
    std::vector<unsigned char> dst_buf_;
    int cached_src_w_{0};
    int cached_src_h_{0};
    int cached_src_c_{0};

    unsigned char *model_data;
    unsigned char * load_model(const char* model_path, unsigned int &model_size);
    bool ensure_preprocess_buffers(int src_w, int src_h, int src_c, int dst_w, int dst_h, int dst_c);

public:

    Yolov5s(const char* model_path, int npu_index);
    ~Yolov5s();

    
    // 模型的高、宽和通道数
    int model_height;
    int model_width;
    int model_channel;

    // 输入图像的高、宽和通道数
    int img_height;
    int img_width;
    int img_channel;

    //模型推理函数
    int inference_image(const Mat &origin_img, detect_result_group_t &result_group);
    int draw_result(const cv::Mat &orig_img, detect_result_group_t &group);

};



#endif