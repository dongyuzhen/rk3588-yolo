#include "post_process.h"
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <algorithm>
#include <set>
#include <cstring>
#include <mutex>

using namespace std;

float anchor0[6] = {10, 13, 16, 30, 33, 23};
float anchor1[6] = {30, 61, 62, 45, 59, 119};
float anchor2[6] = {116, 90, 156, 198, 373, 326};
struct ProbArray
{
    float conf;
    int index;
};

static vector<string> labels;

// Sigmoid 函数：必须用精确 expf()，逼近会导致坐标平方放大误差，画框抖动
static inline float sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

// unsigmoid 函数：根据 sigmoid 结果反推输入值（仅初始化用，非热路径）
static float unsigmoid(float y)
{
    float x = -1.0f * logf(1.0f / y - 1);
    return x;
}
static int sort_descending(vector<ProbArray>& p_arr)
{
    // 使用 lambda 表达式作为排序规则进行排序
    sort(p_arr.begin(), p_arr.end(), 
    [](const ProbArray& a, const ProbArray& b)
    {
        return a.conf > b.conf;
    });

    return 0;
}

static float calculateIOU(float xmin0, float ymin0, float xmax0, float ymax0,
                          float xmin1, float ymin1, float xmax1, float ymax1)
{
    // 计算两个矩形框的交集宽度和高度
    float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
    float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
    // 计算交集面积
    float i = w * h;
    // 计算并集面积
    float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) + (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - i;
    // 计算交并比，如果并集面积为 0，返回 0，否则返回交集面积除以并集面积
    float iou = u <= 0.f? 0.f : (i / u);
    return iou;
}
static int nms(int validCount, vector<float> &boxes, vector<int> &classID,
                vector<int>& indexArray, int currentClass, float nms_threshold)
{
    for(int i = 0; i < validCount; i++)
    {
        int n = indexArray[i];
        if(n == -1 || classID[n] != currentClass)
        {
            continue;
        }

        for(int j = i + 1; j < validCount; j++)
        {
            int m = indexArray[j];
            if(m == -1 || classID[m] != currentClass)
            {
                continue;
            }

            float xmin0 = boxes[n * 4];
            float ymin0 = boxes[n * 4 + 1];
            float xmax0 = boxes[n * 4 + 2] + xmin0;
            float ymax0 = boxes[n * 4 + 3] + ymin0;

            float xmin1 = boxes[m * 4];
            float ymin1 = boxes[m * 4 + 1];
            float xmax1 = boxes[m * 4 + 2] + xmin1;
            float ymax1 = boxes[m * 4 + 3] + ymin1;

            float iou = calculateIOU(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1, xmax1, ymax1);
            if(iou > nms_threshold)
            {
                indexArray[j] = -1;
            }
        }
    }
    return 0;
}

int readLines(const char * LablePath, vector<string> &lable_vector, int maxLines)
{
    ifstream file(LablePath);
    if (!file.is_open())
    {
        std::cerr << "file " << LablePath << " can not open!" << endl;
        return 0;
    }

    string line;
    while (getline(file, line))
    {
        lable_vector.emplace_back(line);
        if (lable_vector.size() >= static_cast<size_t>(maxLines))
        {
            break;
        }
    }
    return lable_vector.size();
}

int LoadLableName(const char * filepath, vector<string> &lable_vector, int num_labels)
{
    int line_num = readLines(filepath, lable_vector, num_labels);
    if(line_num > 0)
    {
        // cout << "标签数量是 " << line_num << endl;
    }
    std::cout << "labels.size()=" << lable_vector.size() << std::endl;
    return line_num;
}

static float deqnt_int8_to_f32(int int_num, int32_t zp, float scale)
{
    float float_num = (float)(int_num - zp) * scale;
    return float_num;
}

inline static int32_t __limit_num(float val, float min, float max)
{
    float f = val <= min ? min : (val >= max ? max : val);
    return static_cast<int32_t>(f);
}

// 量化：将浮点数转换为量化后的 int8_t 类型数据
static int8_t qnt_f32_to_int8(float float_num, int32_t zp, float scale)
{
    float float_qnt_num = (float_num / scale) + zp;
    int8_t int_num = static_cast<int8_t>(__limit_num(float_qnt_num, -128, 127));
    return int_num;
}

/*
参数：
1. input：要处理的 buffer
2. anchor：锚框的长宽参数地址
3. grid_h、grid_w：单元网格数
4. model_height、model_width：模型要求的输入尺寸
5. stride：单元格的步长
6. boxes：存放检测框坐标
7. objProbs：存放目标置信度
8. classID：存放类别索引
9. box_threshold：过滤阈值
10. zp、scale：零点和缩放比例
*/
int process(int8_t *input, float *anchor, int grid_h, int grid_w, int model_height, int model_width, int stride,
            vector<float> &boxes, vector<float> &objProbs, vector<int> &classID, float box_threshold, int32_t zp, float scale)
{
    int validCount = 0;
    int channels_per_pixel = 3 * BOX_NUM_SIZE;

    float box_unsig = unsigmoid(box_threshold);
    int8_t box_int8 = qnt_f32_to_int8(box_unsig, zp, scale);

    // 针对 RKNN 硬件默认输出的 NHWC (内存结构为 [H, W, 3*BOX_NUM_SIZE])，进行 Cache 友好的连续寻址
    for (int i = 0; i < grid_h; i++)
    {
        for (int j = 0; j < grid_w; j++)
        {
            int pixel_offset = (i * grid_w + j) * channels_per_pixel;
            for (int a = 0; a < 3; a++)
            {
                int anchor_offset = pixel_offset + a * BOX_NUM_SIZE;
                int8_t box_anchor_conf = input[anchor_offset + 4];
                if (box_anchor_conf > box_int8)
                {
                    validCount++;
                    int8_t *box_p = input + anchor_offset;
                    
                    // 反量化和 sigmoid 操作获取框的坐标信息 (NHWC 下连续读取)
                    float box_x = sigmoid(deqnt_int8_to_f32(*(box_p + 0), zp, scale)) * 2 - 0.5;
                    float box_y = sigmoid(deqnt_int8_to_f32(*(box_p + 1), zp, scale)) * 2 - 0.5;
                    float box_w = sigmoid(deqnt_int8_to_f32(*(box_p + 2), zp, scale)) * 2.0;
                    float box_h = sigmoid(deqnt_int8_to_f32(*(box_p + 3), zp, scale)) * 2.0;

                    // 计算框的坐标
                    box_x = (box_x + j) * (float)stride;
                    box_y = (box_y + i) * (float)stride;
                    box_w = box_w * box_w * (float)anchor[a*2];
                    box_h = box_h * box_h * (float)anchor[a*2 + 1];

                    box_x = box_x - (box_w / 2.0);
                    box_y = box_y - (box_h / 2.0);

                    boxes.emplace_back(box_x);
                    boxes.emplace_back(box_y);  
                    boxes.emplace_back(box_w);
                    boxes.emplace_back(box_h);
                    
                    // 获取最大类别概率及对应的类别 ID
                    int8_t maxClassProb = *(box_p + 5);
                    int maxClassId = 0;
                    for (int k = 1; k < OBJ_CLASS_NUM; k++)
                    {
                        int8_t prob = *(box_p + 5 + k);
                        if (prob > maxClassProb)
                        {
                            maxClassProb = prob;
                            maxClassId = k;
                        }
                    }

                    // 核心修复：YOLOv5 的最终置信度必须是 obj_conf * cls_conf
                    float obj_conf = sigmoid(deqnt_int8_to_f32(box_anchor_conf, zp, scale));
                    float cls_conf = sigmoid(deqnt_int8_to_f32(maxClassProb, zp, scale));
                    float final_conf = obj_conf * cls_conf;
                    
                    // 严格过滤最终置信度
                    if (final_conf < box_threshold) {
                        boxes.pop_back();
                        boxes.pop_back();
                        boxes.pop_back();
                        boxes.pop_back();
                        validCount--; // 回退之前加上的 validCount
                        continue;
                    }

                    objProbs.emplace_back(final_conf);
                    classID.emplace_back(maxClassId);
                }
            }
        }
    }
    return validCount;
}

inline static int clamp(float val, int min, int max) { return val > min? (val < max? val : max) : min; }


/*
参数：
1. output0, output1, output2：模型的三个输出（量化后的 int8 数据）
2. model_height, model_width：输入图像尺寸
3. box_threshold：锚框的置信度阈值
4. nms_threshold：NMS 的 IoU 阈值
5. scale_w, scale_h：宽和高的缩放比例（映射回原图用）
6. qnt_zps, qnt_scales：三个输出对应的量化零点和缩放系数
*/
int post_process(int8_t *output0, int8_t *output1, int8_t *output2,
                 int model_height, int model_width, float box_threshold,
                 float nms_threshold, float scale_w, float scale_h,
                 int pad_left, int pad_top,
                 std::vector<int32_t>& qnt_zps, std::vector<float>& qnt_scales, detect_result_group_t &result_group)
{
    // 【核心修复】：必须在入口处清空 box_count！
    // 否则当图像中没有任何目标时，有效检测数为 0 会提前 return，
    // 导致未初始化的 result_group 携带着线程池上一帧的“幽灵残影”返回给绘制线程。
    result_group.box_count = 0;

    // 1. 加载标签 (使用 std::call_once 保证多线程并发下绝对的线程安全)
    static std::once_flag flag;
    std::call_once(flag, [&]() {
        LoadLableName(LABLE_PATH, labels, OBJ_CLASS_NUM);
    });
    
    // 示例：量化和反量化测试
    //int8_t int8_num = qnt_f32_to_int8(1.5, 1, 8.0f/255.0f);
    // cout << static_cast<int>(int8_num) << endl;
    // float f = deqnt_int8_to_f32(48, 1, 0.03137f);
    // cout << f << endl;

    vector<float> detect_boxes;
    vector<float> objProbs;
    vector<int> classID;

    // 处理第一个输出
    int stride0 = 8;
    int grid_h0 = model_height / stride0;
    int grid_w0 = model_width / stride0;
    int validCount0 = process(output0, anchor0, grid_h0, grid_w0,
                              model_height, model_width, stride0,
                              detect_boxes, objProbs, classID,
                              box_threshold, qnt_zps[0], qnt_scales[0]);

    // 处理第二个输出
    int stride1 = 16;
    int grid_h1 = model_height / stride1;
    int grid_w1 = model_width / stride1;
    int validCount1 = process(output1, anchor1, grid_h1, grid_w1,
                              model_height, model_width, stride1,
                              detect_boxes, objProbs, classID,
                              box_threshold, qnt_zps[1], qnt_scales[1]);

    // 处理第三个输出
    int stride2 = 32;
    int grid_h2 = model_height / stride2;
    int grid_w2 = model_width / stride2;
    int validCount2 = process(output2, anchor2, grid_h2, grid_w2,
                              model_height, model_width, stride2,
                              detect_boxes, objProbs, classID,
                              box_threshold, qnt_zps[2], qnt_scales[2]);


    int validCount = validCount0 + validCount1 + validCount2;
    if (validCount == 0) {
        return 0;
    }

    std::vector<int> indexArray;

    std::vector<ProbArray> prob_arr;
    for(int i = 0; i<validCount; i++)
    {
        ProbArray temp;
        temp.conf = objProbs[i];
        temp.index = i;
        prob_arr.emplace_back(temp);
    }
    sort_descending(prob_arr);

    objProbs.clear();
    indexArray.clear();

    for (int i = 0; i < validCount; i++) {
        objProbs.emplace_back(prob_arr[i].conf);
        indexArray.emplace_back(prob_arr[i].index);
    }

    // 只统计有效候选的类别（按排序映射后的真实索引）
    std::set<int> class_set;
    for (int i = 0; i < validCount; ++i) {
        int n = indexArray[i];
        if (n >= 0) {
            class_set.insert(classID[n]);
        }
    }

    for(const int& id : class_set)
    {
        nms(validCount, detect_boxes, classID, indexArray, id, nms_threshold);
    }

    int count = 0;
    result_group.box_count = 0;
    
    for(int i = 0; i < validCount; i++)
    {
        if(indexArray[i] == -1 || count >= MAX_OBJ_BOXS)
        {
            continue;
        }
        int n = indexArray[i];
        
        float xmin      = detect_boxes[4*n + 0];
        float ymin      = detect_boxes[4*n + 1];
        float xmax      = detect_boxes[4*n + 2] + xmin;
        float ymax      = detect_boxes[4*n + 3] + ymin;
        float box_conf  = objProbs[i];
        
        // 【新增终极拦截】：把乘积后低于阈值的残次品彻底杀掉！
        if (box_conf < box_threshold) {
            continue;
        }

        int id          = classID[n];

        // 1. 限制在模型输入分辨率范围内
        xmin = clamp(xmin, 0, model_width);
        ymin = clamp(ymin, 0, model_height);
        xmax = clamp(xmax, 0, model_width);
        ymax = clamp(ymax, 0, model_height);

        // 2. 映射回原图（必须先减去 Padding，再除以 Scale）
        result_group.result[count].box.xmin = (int)((xmin - pad_left) / scale_w);
        result_group.result[count].box.ymin = (int)((ymin - pad_top) / scale_h);
        result_group.result[count].box.xmax = (int)((xmax - pad_left) / scale_w);
        result_group.result[count].box.ymax = (int)((ymax - pad_top) / scale_h);
        result_group.result[count].box_conf = box_conf;

        const char *label_temp = labels[id].c_str();
        // 将类别名称复制到检测结果组中（保证 null 终止，防止 UB）
        strncpy(result_group.result[count].label, label_temp, 32);
        result_group.result[count].label[31] = '\0';

        // printf("%s\n", labels[id].c_str());
        count++;
        result_group.box_count = count;
    }
   
    return 0;
}
