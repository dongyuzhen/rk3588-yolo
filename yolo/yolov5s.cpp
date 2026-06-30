#include "yolov5s.h"
#include "post_process.h"

#include <chrono>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

// 打印 tensor 属性（调试用）
static void print_tensor_attr(rknn_tensor_attr *attr)
{
    string shape_str = attr->n_dims < 1 ? "" : to_string(attr->dims[0]);
    for (int i = 1; i < attr->n_dims; i++)
        shape_str += "," + to_string(attr->dims[i]);
    printf("  index=%d name=%s dims=[%s] size=%d fmt=%d type=%d\n",
           attr->index, attr->name, shape_str.c_str(), attr->size, attr->fmt, attr->type);
}

// -------------------- 加载模型文件 --------------------
unsigned char *Yolov5s::load_model(const char *model_path, unsigned int &model_size)
{
    FILE *fp = fopen(model_path, "rb");
    if (!fp)
    {
        printf("open model failed: %s\n", model_path);
        return nullptr;
    }
    fseek(fp, 0, SEEK_END);
    model_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    unsigned char *data = (unsigned char *)malloc(model_size);
    if (!data)
    {
        printf("malloc model buffer failed\n");
        fclose(fp);
        return nullptr;
    }
    if (fread(data, 1, model_size, fp) != model_size)
    {
        printf("read model failed\n");
        free(data);
        fclose(fp);
        return nullptr;
    }
    fclose(fp);
    return data;
}

Yolov5s::Yolov5s(const char *model_path, int npu_index, int img_w, int img_h)
    : context(0), model_size(0), model_data(nullptr),
      model_height(0), model_width(0), model_channel(0),
      img_width(img_w), img_height(img_h)
{
    int ret;

    model_data = load_model(model_path, model_size);
    if (!model_data)
        return;

    // 初始化 RKNN context
    ret = rknn_init(&context, model_data, model_size, RKNN_FLAG_PRIOR_HIGH, NULL);
    if (ret != RKNN_SUCC)
    {
        printf("rknn_init failed, ret=%d\n", ret);
        return;
    }
    printf("Yolov5s[%d] init OK\n", npu_index);

    // 绑定 NPU core（三核轮转，充分利用 RK3588 三个 NPU 核心）
    rknn_core_mask core_mask;
    switch (npu_index % 3)
    {
    case 0:
        core_mask = RKNN_NPU_CORE_0;
        break;
    case 1:
        core_mask = RKNN_NPU_CORE_1;
        break;
    default:
        core_mask = RKNN_NPU_CORE_2;
        break;
    }

    // [BENCH] 单核测试：解除下面两行注释，所有实例都强制跑 Core0，测单核吞吐上限
    // 注意：同时要把 ThreadPoll 的线程数改为 1，否则多个线程竞争同一个核会相互阻塞
    // core_mask = RKNN_NPU_CORE_0;

    ret = rknn_set_core_mask(context, core_mask);
    if (ret != RKNN_SUCC)
        printf("rknn_set_core_mask failed, ret=%d\n", ret);

    // 查询输入输出 tensor 数量
    ret = rknn_query(context, RKNN_QUERY_IN_OUT_NUM, &num_tensors, sizeof(num_tensors));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query IN_OUT_NUM failed, ret=%d\n", ret);
        return;
    }
    printf("  n_input=%d n_output=%d\n", num_tensors.n_input, num_tensors.n_output);

    // 查询输入 tensor 属性
    input_attrs.resize(num_tensors.n_input);
    for (int i = 0; i < (int)num_tensors.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(context, RKNN_QUERY_INPUT_ATTR, &input_attrs[i], sizeof(input_attrs[i]));
        if (ret != RKNN_SUCC)
            printf("rknn_query INPUT_ATTR[%d] failed, ret=%d\n", i, ret);
        printf("input[%d]: ", i);
        print_tensor_attr(&input_attrs[i]);
    }

    // 查询输出 tensor 属性
    output_attrs.resize(num_tensors.n_output);
    for (int i = 0; i < (int)num_tensors.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(context, RKNN_QUERY_OUTPUT_ATTR, &output_attrs[i], sizeof(output_attrs[i]));
        if (ret != RKNN_SUCC)
            printf("rknn_query OUTPUT_ATTR[%d] failed, ret=%d\n", i, ret);
    }

    // 解析模型输入尺寸（支持 NCHW / NHWC）
    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        model_channel = input_attrs[0].dims[1];
        model_height = input_attrs[0].dims[2];
        model_width = input_attrs[0].dims[3];
    }
    else
    { // NHWC
        model_height = input_attrs[0].dims[1];
        model_width = input_attrs[0].dims[2];
        model_channel = input_attrs[0].dims[3];
    }
    printf("  model input: %dx%dx%d\n", model_width, model_height, model_channel);

    // 计算 letterbox 参数
    scale = std::min((float)model_width / img_width, (float)model_height / img_height);
    
    // 保证 4 字节对齐（RGA硬件对齐最严要求）
    resized_width = ((int)(img_width * scale) / 4) * 4;
    resized_height = ((int)(img_height * scale) / 4) * 4;
    
    pad_left = (model_width - resized_width) / 2;
    pad_top = (model_height - resized_height) / 2;
    // 保证 pad 也是 4 字节对齐
    pad_left = (pad_left / 4) * 4;
    pad_top = (pad_top / 4) * 4;
    
    pad_right = model_width - resized_width - pad_left;
    pad_bottom = model_height - resized_height - pad_top;

    // 分配中间 RGB888 缓冲（img_width × img_height × 3），用于 YUYV→RGB 色彩转换
    // rknn_create_mem 内部使用 DMA 堆，fd 可直接被 RGA importbuffer_fd 使用
    size_t mid_size = (size_t)img_width * img_height * 3;
    mid_mem_ = rknn_create_mem(context, mid_size);
    if (!mid_mem_)
    {
        printf("[yolo] rknn_create_mem mid_mem failed\n");
        return;
    }

    // 持久化分配 input/output 内存，避免每帧 create/destroy 引起 cache cold miss
    input_attrs[0].type = RKNN_TENSOR_UINT8;
    input_attrs[0].fmt = RKNN_TENSOR_NHWC;
    input_mem_ = rknn_create_mem(context, input_attrs[0].size);
    if (!input_mem_)
    {
        printf("[yolo] rknn_create_mem input_mem failed\n");
        return;
    }
    ret = rknn_set_io_mem(context, input_mem_, &input_attrs[0]);
    if (ret != RKNN_SUCC)
    {
        printf("[yolo] rknn_set_io_mem input failed, ret=%d\n", ret);
        return;
    }

    // 缓存 RGA handle：mid_mem_ fd 和 input_mem_ fd 从不变化，import 一次全生命周期复用
    mid_handle_ = importbuffer_fd(mid_mem_->fd, img_width, img_height, RK_FORMAT_RGB_888);
    model_handle_ = importbuffer_fd(input_mem_->fd, model_width, model_height, RK_FORMAT_RGB_888);
    if (!mid_handle_ || !model_handle_)
    {
        printf("[yolo] RGA handle cache failed: mid=%d model=%d\n", (int)mid_handle_, (int)model_handle_);
        return;
    }

    output_mems_.resize(num_tensors.n_output, nullptr);
    for (int i = 0; i < (int)num_tensors.n_output; i++)
    {
        output_mems_[i] = rknn_create_mem(context, output_attrs[i].size);
        if (!output_mems_[i])
        {
            printf("[yolo] rknn_create_mem output[%d] failed\n", i);
            return;
        }
        ret = rknn_set_io_mem(context, output_mems_[i], &output_attrs[i]);
        if (ret != RKNN_SUCC)
        {
            printf("[yolo] rknn_set_io_mem output[%d] failed, ret=%d\n", i, ret);
            return;
        }
    }
}

// -------------------- 析构函数 --------------------
Yolov5s::~Yolov5s()
{
    if (mid_handle_)
        releasebuffer_handle(mid_handle_);
    if (model_handle_)
        releasebuffer_handle(model_handle_);

    for (int i = 0; i < (int)output_mems_.size(); i++)
    {
        if (output_mems_[i])
            rknn_destroy_mem(context, output_mems_[i]);
    }
    if (input_mem_)
    {
        rknn_destroy_mem(context, input_mem_);
        input_mem_ = nullptr;
    }
    if (mid_mem_)
    {
        rknn_destroy_mem(context, mid_mem_);
        mid_mem_ = nullptr;
    }
    if (context)
        rknn_destroy(context);
    free(model_data);
}

// -------------------- 推理主函数 --------------------
// dmabuf_fd: V4L2 DMA buffer fd，格式 YUYV422，尺寸 img_width × img_height
// 使用 RGA 将其转换并缩放为 RGB888（model_width × model_height），
// 直接写入 RKNN input_mem，实现零拷贝。
int Yolov5s::inference_image(int dmabuf_fd, detect_result_group_t &result_group)
{
    int ret = 0;

    // ---------- 1. 检查持久化 IO 缓冲是否就绪 ----------
    if (!input_mem_ || output_mems_.empty())
    {
        printf("[yolo] IO buffers not initialized\n");
        return -1;
    }

    // ---------- 2. RGA 硬件 Letterbox 预处理 ----------
    {
        // import V4L2 dmabuf（每帧变化，必须 per-frame import）
        rga_buffer_handle_t h_yuyv = importbuffer_fd(dmabuf_fd, img_width, img_height, RK_FORMAT_YUYV_422);
        if (!h_yuyv)
        {
            printf("[yolo] RGA import yuyv fd failed\n");
            return -1;
        }

        rga_buffer_t src_yuyv = wrapbuffer_handle(h_yuyv, img_width, img_height, RK_FORMAT_YUYV_422);
        rga_buffer_t dst_model = wrapbuffer_handle(model_handle_, model_width, model_height, RK_FORMAT_RGB_888);

        // 1. 填充目标内存为灰色 (114, 114, 114)
        // imfill 在 RGA3 上不支持 RGB888 目标格式（Invalid argument），
        // 改用 CPU memset 写入同一块 DMA 内存的虚拟地址，开销更低。
        memset(input_mem_->virt_addr, 114, (size_t)model_width * model_height * 3);

        // 2. 利用 improcess 一步完成 YUYV->RGB888 色彩转换、缩放并放置在居中区域
        im_rect srect = {0, 0, img_width, img_height};
        im_rect drect = {pad_left, pad_top, resized_width, resized_height};
        im_rect prect = {0, 0, 0, 0};
        rga_buffer_t empty_pat;
        memset(&empty_pat, 0, sizeof(rga_buffer_t));
        
        IM_STATUS rga_ret = improcess(src_yuyv, dst_model, empty_pat, srect, drect, prect, IM_SYNC);
        if (rga_ret != IM_STATUS_SUCCESS)
        {
            printf("[yolo] RGA improcess Letterbox failed: %s\n", imStrError((IM_STATUS)rga_ret));
            releasebuffer_handle(h_yuyv);
            return -1;
        }

        releasebuffer_handle(h_yuyv);
    }

    // [BENCH] OpenCV 等效路径（仅用于对比，正常运行时保持注释）
    // 解除下面整块注释后，需同时注释上方 RGA 块，避免重复写入 input_mem_
    /*
    {
        auto _t0 = std::chrono::high_resolution_clock::now();

        size_t _yuyv_size = (size_t)img_width * img_height * 2;
        void *_yuyv_ptr = mmap(nullptr, _yuyv_size, PROT_READ, MAP_SHARED, dmabuf_fd, 0);
        cv::Mat _yuyv_mat(img_height, img_width, CV_8UC2, _yuyv_ptr);
        cv::Mat _rgb_mat, _resized_mat;
        cv::cvtColor(_yuyv_mat, _rgb_mat, cv::COLOR_YUV2RGB_YUYV);   // YUYV→RGB（与 RGA 输出色序一致）
        cv::resize(_rgb_mat, _resized_mat, cv::Size(model_width, model_height));
        // 将结果写入 input_mem_（与 RGA 路径写入同一块内存，保证后续推理可用）
        memcpy(input_mem_->virt_addr, _resized_mat.data, (size_t)model_width * model_height * 3);
        munmap(_yuyv_ptr, _yuyv_size);

        double _ocv_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - _t0).count();
        printf("[bench] OpenCV preprocess: %.3f ms\n", _ocv_ms);
    }
    */

    // ---------- 3. NPU 推理 ----------
    ret = rknn_run(context, NULL);
    if (ret != RKNN_SUCC)
    {
        printf("[yolo] rknn_run failed, ret=%d\n", ret);
        return ret;
    }

    // ---------- 4. 后处理 ----------
    {
        vector<int32_t> qnt_zps;
        vector<float> qnt_scales;
        for (int i = 0; i < (int)num_tensors.n_output; i++)
        {
            qnt_zps.emplace_back(output_attrs[i].zp);
            qnt_scales.emplace_back(output_attrs[i].scale);
        }

        post_process(
            (int8_t *)output_mems_[0]->virt_addr,
            (int8_t *)output_mems_[1]->virt_addr,
            (int8_t *)output_mems_[2]->virt_addr,
            model_height, model_width,
            BOX_THRESHOLD, NMS_THRESHOLD,
            scale, scale,
            qnt_zps, qnt_scales,
            result_group);
    }

    return 0;
}

// -------------------- 绘制检测框 --------------------
// orig_img：原始分辨率的 BGR Mat（img_width × img_height）
int Yolov5s::draw_result(cv::Mat &orig_img, detect_result_group_t &result_group)
{
    for (int i = 0; i < result_group.box_count; i++)
    {
        int xmin = result_group.result[i].box.xmin;
        int ymin = result_group.result[i].box.ymin;
        int xmax = result_group.result[i].box.xmax;
        int ymax = result_group.result[i].box.ymax;

        cv::rectangle(orig_img, cv::Point(xmin, ymin), cv::Point(xmax, ymax),
                      cv::Scalar(255, 0, 0, 255), 3);

        std::stringstream ss;
        ss << std::fixed << std::setprecision(2)
           << result_group.result[i].label << ":"
           << result_group.result[i].box_conf * 100 << " %";
        std::string img_label = ss.str();

        const double font_scale = 1.1;
        cv::Point text_org(xmin, std::max(20, ymin - 12));

        // 黑色描边
        cv::putText(orig_img, img_label, text_org, cv::FONT_HERSHEY_SIMPLEX,
                    font_scale, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
        // 黄色正文
        cv::putText(orig_img, img_label, text_org, cv::FONT_HERSHEY_SIMPLEX,
                    font_scale, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }
    return 0;
}
