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
    if (!fp) {
        printf("open model failed: %s\n", model_path);
        return nullptr;
    }
    fseek(fp, 0, SEEK_END);
    model_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    unsigned char *data = (unsigned char *)malloc(model_size);
    if (!data) {
        printf("malloc model buffer failed\n");
        fclose(fp);
        return nullptr;
    }
    if (fread(data, 1, model_size, fp) != model_size) {
        printf("read model failed\n");
        free(data);
        fclose(fp);
        return nullptr;
    }
    fclose(fp);
    return data;
}

// -------------------- 构造函数 --------------------
Yolov5s::Yolov5s(const char *model_path, int npu_index)
    : context(0), model_size(0), model_data(nullptr),
      model_height(0), model_width(0), model_channel(0)
{
    int ret;

    model_data = load_model(model_path, model_size);
    if (!model_data) return;

    // 初始化 RKNN context
    ret = rknn_init(&context, model_data, model_size, RKNN_FLAG_PRIOR_HIGH, NULL);
    if (ret != RKNN_SUCC) {
        printf("rknn_init failed, ret=%d\n", ret);
        return;
    }
    printf("Yolov5s[%d] init OK\n", npu_index);

    // 绑定 NPU core
    rknn_core_mask core_mask;
    switch (npu_index % 3) {
        case 0: core_mask = RKNN_NPU_CORE_0; break;
        case 1: core_mask = RKNN_NPU_CORE_1; break;
        default: core_mask = RKNN_NPU_CORE_2; break;
    }
    ret = rknn_set_core_mask(context, core_mask);
    if (ret != RKNN_SUCC)
        printf("rknn_set_core_mask failed, ret=%d\n", ret);

    // 查询输入输出 tensor 数量
    ret = rknn_query(context, RKNN_QUERY_IN_OUT_NUM, &num_tensors, sizeof(num_tensors));
    if (ret != RKNN_SUCC) {
        printf("rknn_query IN_OUT_NUM failed, ret=%d\n", ret);
        return;
    }
    printf("  n_input=%d n_output=%d\n", num_tensors.n_input, num_tensors.n_output);

    // 查询输入 tensor 属性
    input_attrs.resize(num_tensors.n_input);
    for (int i = 0; i < (int)num_tensors.n_input; i++) {
        input_attrs[i].index = i;
        ret = rknn_query(context, RKNN_QUERY_INPUT_ATTR, &input_attrs[i], sizeof(input_attrs[i]));
        if (ret != RKNN_SUCC)
            printf("rknn_query INPUT_ATTR[%d] failed, ret=%d\n", i, ret);
        printf("input[%d]: ", i);
        print_tensor_attr(&input_attrs[i]);
    }

    // 查询输出 tensor 属性
    output_attrs.resize(num_tensors.n_output);
    for (int i = 0; i < (int)num_tensors.n_output; i++) {
        output_attrs[i].index = i;
        ret = rknn_query(context, RKNN_QUERY_OUTPUT_ATTR, &output_attrs[i], sizeof(output_attrs[i]));
        if (ret != RKNN_SUCC)
            printf("rknn_query OUTPUT_ATTR[%d] failed, ret=%d\n", i, ret);
    }

    // 解析模型输入尺寸（支持 NCHW / NHWC）
    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
        model_channel = input_attrs[0].dims[1];
        model_height  = input_attrs[0].dims[2];
        model_width   = input_attrs[0].dims[3];
    } else { // NHWC
        model_height  = input_attrs[0].dims[1];
        model_width   = input_attrs[0].dims[2];
        model_channel = input_attrs[0].dims[3];
    }
    printf("  model input: %dx%dx%d\n", model_width, model_height, model_channel);

    // 分配中间 RGB888 缓冲（img_width × img_height × 3），用于 YUYV→RGB 色彩转换
    // rknn_create_mem 内部使用 DMA 堆，fd 可直接被 RGA importbuffer_fd 使用
    size_t mid_size = (size_t)img_width * img_height * 3;
    mid_mem_ = rknn_create_mem(context, mid_size);
    if (!mid_mem_) {
        printf("[yolo] rknn_create_mem mid_mem failed\n");
        return;
    }
}

// -------------------- 析构函数 --------------------
Yolov5s::~Yolov5s()
{
    if (mid_mem_) {
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

    // ---------- 1. 创建 RKNN 输入内存，并注册到 context ----------
    // input_attrs[0].size 就是 model_width * model_height * model_channel（RGB888）
    // 将 type 设为 NHWC + UINT8，与 RGA 输出的 RGB888 一致
    input_attrs[0].type = RKNN_TENSOR_UINT8;
    input_attrs[0].fmt  = RKNN_TENSOR_NHWC;

    rknn_tensor_mem *input_mem = rknn_create_mem(context, input_attrs[0].size);
    if (!input_mem) {
        printf("[yolo] rknn_create_mem input failed\n");
        return -1;
    }

    ret = rknn_set_io_mem(context, input_mem, &input_attrs[0]);
    if (ret != RKNN_SUCC) {
        printf("[yolo] rknn_set_io_mem input failed, ret=%d\n", ret);
        rknn_destroy_mem(context, input_mem);
        return -1;
    }

    // ---------- 2. 创建 RKNN 输出内存，并注册到 context ----------
    vector<rknn_tensor_mem *> output_mems(num_tensors.n_output, nullptr);
    for (int i = 0; i < (int)num_tensors.n_output; i++) {
        output_mems[i] = rknn_create_mem(context, output_attrs[i].size);
        if (!output_mems[i]) {
            printf("[yolo] rknn_create_mem output[%d] failed\n", i);
            ret = -1;
            goto cleanup;
        }
        ret = rknn_set_io_mem(context, output_mems[i], &output_attrs[i]);
        if (ret != RKNN_SUCC) {
            printf("[yolo] rknn_set_io_mem output[%d] failed, ret=%d\n", i, ret);
            goto cleanup;
        }
    }

    // ---------- 3. RGA 两步：YUYV→RGB888（色彩转换）→ resize 到模型尺寸 ----------
    // Linux 下正确模式：importbuffer_fd 注册 → wrapbuffer_handle 描述 → RGA操作 → releasebuffer_handle
    {
        // import 三个 fd 到 RGA
        rga_buffer_handle_t h_yuyv  = importbuffer_fd(dmabuf_fd,     img_width,   img_height,   RK_FORMAT_YUYV_422);
        rga_buffer_handle_t h_mid   = importbuffer_fd(mid_mem_->fd,  img_width,   img_height,   RK_FORMAT_RGB_888);
        rga_buffer_handle_t h_model = importbuffer_fd(input_mem->fd, model_width, model_height, RK_FORMAT_RGB_888);

        if (!h_yuyv || !h_mid || !h_model) {
            printf("[yolo] RGA importbuffer_fd failed: h_yuyv=%d h_mid=%d h_model=%d\n",
                   (int)h_yuyv, (int)h_mid, (int)h_model);
            if (h_yuyv)  releasebuffer_handle(h_yuyv);
            if (h_mid)   releasebuffer_handle(h_mid);
            if (h_model) releasebuffer_handle(h_model);
            ret = -1;
            goto cleanup;
        }

        // 步骤 A：YUYV → RGB888（同尺寸色彩转换）
        rga_buffer_t src_yuyv = wrapbuffer_handle(h_yuyv,  img_width,   img_height,   RK_FORMAT_YUYV_422);
        rga_buffer_t dst_rgb  = wrapbuffer_handle(h_mid,   img_width,   img_height,   RK_FORMAT_RGB_888);
        IM_STATUS rga_ret = imcvtcolor(src_yuyv, dst_rgb, RK_FORMAT_YUYV_422, RK_FORMAT_RGB_888);
        if (rga_ret != IM_STATUS_SUCCESS) {
            printf("[yolo] RGA YUYV->RGB failed: %s\n", imStrError(rga_ret));
            releasebuffer_handle(h_yuyv);
            releasebuffer_handle(h_mid);
            releasebuffer_handle(h_model);
            ret = -1;
            goto cleanup;
        }

        // 步骤 B：RGB888 resize → model 尺寸
        rga_buffer_t src_rgb   = wrapbuffer_handle(h_mid,   img_width,   img_height,   RK_FORMAT_RGB_888);
        rga_buffer_t dst_model = wrapbuffer_handle(h_model, model_width, model_height, RK_FORMAT_RGB_888);
        rga_ret = imresize(src_rgb, dst_model);
        if (rga_ret != IM_STATUS_SUCCESS) {
            printf("[yolo] RGA RGB resize failed: %s\n", imStrError(rga_ret));
            releasebuffer_handle(h_yuyv);
            releasebuffer_handle(h_mid);
            releasebuffer_handle(h_model);
            ret = -1;
            goto cleanup;
        }

        releasebuffer_handle(h_yuyv);
        releasebuffer_handle(h_mid);
        releasebuffer_handle(h_model);
    }

    // ---------- 4. NPU 推理 ----------
    ret = rknn_run(context, NULL);
    if (ret != RKNN_SUCC) {
        printf("[yolo] rknn_run failed, ret=%d\n", ret);
        goto cleanup;
    }

    // ---------- 5. 后处理 ----------
    {
        float scale_w = (float)model_width  / img_width;
        float scale_h = (float)model_height / img_height;

        vector<int32_t> qnt_zps;
        vector<float>   qnt_scales;
        for (int i = 0; i < (int)num_tensors.n_output; i++) {
            qnt_zps.emplace_back(output_attrs[i].zp);
            qnt_scales.emplace_back(output_attrs[i].scale);
        }

        post_process(
            (int8_t *)output_mems[0]->virt_addr,
            (int8_t *)output_mems[1]->virt_addr,
            (int8_t *)output_mems[2]->virt_addr,
            model_height, model_width,
            BOX_THRESHOLD, NMS_THRESHOLD,
            scale_w, scale_h,
            qnt_zps, qnt_scales,
            result_group);
    }

    ret = 0;

cleanup:
    // ---------- 6. 释放 RKNN 内存 ----------
    for (int i = 0; i < (int)num_tensors.n_output; i++) {
        if (output_mems[i])
            rknn_destroy_mem(context, output_mems[i]);
    }
    rknn_destroy_mem(context, input_mem);

    return ret;
}

// -------------------- 绘制检测框 --------------------
// orig_img：原始分辨率的 BGR Mat（img_width × img_height）
int Yolov5s::draw_result(cv::Mat &orig_img, detect_result_group_t &result_group)
{
    for (int i = 0; i < result_group.box_count; i++) {
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
