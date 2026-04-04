#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <future>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "SafeQueue.h"
#include "camera/cma_buffer.h"
#include "camera/v4l2_camera.h"
#include "streamer.h"
#include "thread_poll.h"

// RGA
#include "RgaUtils.h"
#include "drmrga.h"
#include "im2d.h"
#include "rga.h"
#include "memory"
#include "camera/cma_buffer.h"
#define ALIGN(x, a) (((x) + (a)-1) & ~((a)-1))

namespace {

// ------------------------ 全局运行参数 ------------------------
int g_width = 0;
int g_height = 0;
int g_hor_stride = 0;
int g_ver_stride = 0;

// 采集缓冲数（V4L2 + DMABUF）
constexpr int BUFFER_NUM = 6;
// 采集->处理、处理->写出 两段队列深度
constexpr size_t READ_QUEUE_CAP = 24;
constexpr size_t WRITE_QUEUE_CAP = 24;
// 默认处理帧数
constexpr int DEFAULT_FRAME_LIMIT = 500;
// YOLO线程池 worker 数
constexpr int PROCESS_THREAD_NUM = 3;

// 解析命令行：--no-rtmp
bool parseEnableRtmp(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--no-rtmp") == 0) {
            return false;
        }
    }
    return true;
}


// ------------------------ 阶段间数据结构 ------------------------
// capture 线程输出：携带原始 FramePacket + 捕获时刻
struct CapturedFrame {
    int index{-1};
    FramePacket_t packet{};
};

// process 线程输出：携带 YOLO 检测结果
struct YoloOutputFrame {
    int index{-1};
    FramePacket_t packet{};
    detect_result_group_t detections{};
};

// process 线程内部 inflight 映射项：
// index -> future，保证按序输出（next_out）
struct InflightTask {
    FramePacket_t packet{};
    std::future<ProcessResult> fut;
};

// pipeline 中 NV12 工作缓冲使用单独 heap fd
int g_nv12_heap_fd = -1;

// 全局模块与队列
V4L2Camera g_camera;
std::unique_ptr<CmaBufferPool> g_pool;
SafeQueue<CapturedFrame> g_readQueue(READ_QUEUE_CAP);
SafeQueue<YoloOutputFrame> g_writeQueue(WRITE_QUEUE_CAP);

// 线程退出信号
std::atomic<bool> g_readFinish(false);
std::atomic<bool> g_processFinish(false);

// YUYV dmabuf fd → BGR 虚拟地址，使用 RGA 硬件转换
static void YUYV_to_BGR_rga(int yuyv_fd, int w, int h, uint8_t *bgr)
{
    const size_t bgr_size = static_cast<size_t>(w) * h * 3;
    rga_buffer_handle_t yuyv_h = importbuffer_fd(yuyv_fd, w, h, RK_FORMAT_YUYV_422);
    rga_buffer_handle_t bgr_h  = importbuffer_virtualaddr(bgr, bgr_size);
    if (yuyv_h == 0 || bgr_h == 0) {
        std::printf("[Write] import yuyv/bgr failed\n");
        if (yuyv_h) releasebuffer_handle(yuyv_h);
        if (bgr_h)  releasebuffer_handle(bgr_h);
        return;
    }
    rga_buffer_t src = wrapbuffer_handle(yuyv_h, w, h, RK_FORMAT_YUYV_422);
    rga_buffer_t dst = wrapbuffer_handle(bgr_h,  w, h, RK_FORMAT_BGR_888);
    int ret = imcvtcolor(src, dst, RK_FORMAT_YUYV_422, RK_FORMAT_BGR_888);
    if (ret != IM_STATUS_SUCCESS)
        std::printf("[Write] RGA YUYV->BGR failed: %s\n", imStrError((IM_STATUS)ret));
    releasebuffer_handle(yuyv_h);
    releasebuffer_handle(bgr_h);
}

// BGR 虚拟地址 → NV12 虚拟地址，使用 RGA 硬件转换
static void BGR_to_NV12_rga(uint8_t *bgr, int w, int h,
                              uint8_t *nv12, int w_stride, int h_stride)
{
    const size_t bgr_size  = static_cast<size_t>(w) * h * 3;
    const size_t nv12_size = static_cast<size_t>(w_stride) * h_stride * 3 / 2;
    rga_buffer_handle_t bgr_h  = importbuffer_virtualaddr(bgr,  bgr_size);
    rga_buffer_handle_t nv12_h = importbuffer_virtualaddr(nv12, nv12_size);
    if (bgr_h == 0 || nv12_h == 0) {
        std::printf("[Write] import bgr/nv12 va failed\n");
        if (bgr_h)  releasebuffer_handle(bgr_h);
        if (nv12_h) releasebuffer_handle(nv12_h);
        return;
    }
    rga_buffer_t src = wrapbuffer_handle(bgr_h,  w, h, RK_FORMAT_BGR_888);
    rga_buffer_t dst = wrapbuffer_handle(nv12_h, w, h, RK_FORMAT_YCbCr_420_SP, w_stride, h_stride);
    int ret = imcvtcolor(src, dst, RK_FORMAT_BGR_888, RK_FORMAT_YCbCr_420_SP);
    if (ret != IM_STATUS_SUCCESS)
        std::printf("[Write] RGA BGR->NV12 failed: %s\n", imStrError((IM_STATUS)ret));
    releasebuffer_handle(bgr_h);
    releasebuffer_handle(nv12_h);
}

// MJPEG 解码：
// - baseline：先拷贝 compressed bytes 再 imdecode
// - optimized：直接把 packet.data 作为 Mat 视图解码
// cv::Mat decodeMjpegFrame(FramePacket_t& packet) {
//     if (!packet.data || packet.bytes_used == 0) {
//         return cv::Mat();
//     }
//     cv::Mat encoded_view(1, static_cast<int>(packet.bytes_used), CV_8UC1, const_cast<uint8_t*>(packet.data));
//     //这里固定发生在cpu上，后续可以移植mipi摄像头驱动
//     return cv::imdecode(encoded_view, cv::IMREAD_COLOR);
// }


// ------------------------ 线程1：采集 ------------------------
// 从 V4L2 连续抓帧，把 FramePacket 放入 read 队列。
// 注意：这里只 DQ，不做 release，release 由写线程在“真正用完”后执行。
void captureThreadFunc(int frame_limit) {
    int idx = 0;
    while (idx < frame_limit) {
        FramePacket_t packet;
        if (!g_camera.captureFrame(packet, 100)) {
            continue;
        }
        CapturedFrame f;
        f.index = idx;
        f.packet = packet;
        g_readQueue.enqueue(f);
        ++idx;
    }

    g_readFinish = true;
}

// ------------------------ 线程2：处理（decode + YOLO） ------------------------
// 关键点：
// 1) 允许多任务 inflight，提高 NPU 利用率
// 2) 用 next_out 保证输出顺序
// 3) decode/yolo 失败时，及时 releaseFrame，避免占死采集缓冲
void processThreadFunc(ThreadPoll& npu_pool) {
    constexpr size_t MAX_INFLIGHT = 8;
    int next_out = 0;

    std::map<int, InflightTask> inflight;

    while (true) {
        // A. 尽量从 readQueue 取新帧并提交 YOLO
        if (!g_readQueue.empty() && inflight.size() < MAX_INFLIGHT) {
            CapturedFrame in;
            if (g_readQueue.dequeue(in)) {
                InflightTask task;
                task.packet = in.packet;
                task.fut = npu_pool.submit_task_async(in.index, task.packet.dmabuf_fd);
                inflight.emplace(in.index, std::move(task));
            }
        }

        // B. 仅按 next_out 检查 future，就绪后输出到 writeQueue
        auto it = inflight.find(next_out);
        if (it != inflight.end()) {
            auto status = it->second.fut.wait_for(std::chrono::milliseconds(1));
            if (status == std::future_status::ready) {
                ProcessResult result = it->second.fut.get();
                if (!result.success) {
                    std::cerr << "[Process] yolo failed index=" << next_out << " err=" << result.error_msg << std::endl;
                    g_camera.releaseFrame(it->second.packet);
                } else {
                    YoloOutputFrame out;
                    out.index = next_out;
                    out.packet = it->second.packet;
                    out.detections = result.detection_results;
                    g_writeQueue.enqueue(out);
                }
                inflight.erase(it);
                ++next_out;
            }
        }

        // C. 退出条件：采集结束 + 输入队列空 + inflight 清空
        if (g_readFinish && g_readQueue.empty() && inflight.empty()) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    g_processFinish = true;
}

// ------------------------ 线程3：写出（画框 + 编码/推流） ------------------------
void writeThreadFunc(cv::VideoWriter &writer) {
    // 两块复用缓冲：BGR 用于画框，NV12 用于送编码
    const size_t bgr_size  = static_cast<size_t>(g_width) * g_height * 3;
    const size_t nv12_size = static_cast<size_t>(g_hor_stride) * g_ver_stride * 3 / 2;

    std::vector<uint8_t> bgr_buf(bgr_size);

    CmaBuffer nv12_cma;
    if (!nv12_cma.allocate(g_nv12_heap_fd, nv12_size, "pipeline_nv12_work")) {
        std::cerr << "[Write] failed to allocate NV12 cma buffer" << std::endl;
        g_processFinish = true;
        return;
    }

    while (true) {
        if (g_processFinish && g_writeQueue.empty()) break;

        YoloOutputFrame out;
        if (!g_writeQueue.dequeue(out)) {
            if (g_processFinish) break;
            continue;
        }

        uint8_t *bgr_ptr  = bgr_buf.data();
        uint8_t *nv12_ptr = static_cast<uint8_t *>(nv12_cma.addr());

        // 1. YUYV dmabuf → BGR（RGA 硬件转换）
        YUYV_to_BGR_rga(out.packet.dmabuf_fd, g_width, g_height, bgr_ptr);

        // 2. 在 BGR 上画检测框和标签（OpenCV CPU）
        cv::Mat bgr_mat(g_height, g_width, CV_8UC3, bgr_ptr);
        for (int i = 0; i < out.detections.box_count; i++) {
            const detect_result_t &det = out.detections.result[i];
            cv::rectangle(bgr_mat,
                          cv::Point(det.box.xmin, det.box.ymin),
                          cv::Point(det.box.xmax, det.box.ymax),
                          cv::Scalar(0, 255, 0), 2);

            std::ostringstream ss;
            ss << det.label << ": "
               << std::fixed << std::setprecision(1)
               << det.box_conf * 100.f << "%";
            cv::Point text_org(det.box.xmin, std::max(20, det.box.ymin - 8));
            // 黑色描边
            cv::putText(bgr_mat, ss.str(), text_org,
                        cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
            // 黄色正文
            cv::putText(bgr_mat, ss.str(), text_org,
                        cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }

        // 3. BGR（带框）→ NV12（RGA 硬件转换），送 MPP 编码（零拷贝，传 fd）
        BGR_to_NV12_rga(bgr_ptr, g_width, g_height, nv12_ptr, g_hor_stride, g_ver_stride);
        process_frame_fd(nv12_cma.fd(), static_cast<int>(nv12_cma.size()));

        // 4. 本地 VideoWriter（保存带框画面，调试用）
        if (writer.isOpened())
            writer.write(bgr_mat);

        // 5. 归还 V4L2 buffer
        if (!g_camera.releaseFrame(out.packet))
            std::cerr << "[Write] releaseFrame failed index=" << out.index << std::endl;
    }
}

} // namespace

// ------------------------ 主流程 ------------------------
int main(int argc, char* argv[]) {
    // 1) 解析参数
    const bool enable_rtmp = parseEnableRtmp(argc, argv);
    const int frame_limit = DEFAULT_FRAME_LIMIT;

    std::cout << "[Config] frames=" << frame_limit
              << " rtmp=" << (enable_rtmp ? "on" : "off") << std::endl;

    int cma_heap_fd = open("/dev/dma_heap/cma", O_RDWR);
    if (cma_heap_fd < 0) {
        perror("open /dev/dma_heap/cma");
        return -1;
    }
   g_pool = std::make_unique<CmaBufferPool>(cma_heap_fd);
    
    // 2) 初始化相机（V4L2 + DMABUF）并启动采集
    if (!g_camera.init("/dev/video0", 640, 480, BUFFER_NUM, V4L2_PIX_FMT_YUYV, g_pool.get())) {
        std::cerr << "Fail to init V4L2Camera" << std::endl;
        return -1;
    }

    if (!g_camera.startStreaming()) {
        std::cerr << "Fail to start V4L2 stream" << std::endl;
        return -1;
    }

    // 3) 编码/推流参数（与摄像头采集分辨率一致）
    const int width = 640;
    const int height = 480;
    const int fps = 30;

    g_width = width;
    g_height = height;
    // 统一按 16 对齐，避免 720->768 产生可见 padding 行
    g_hor_stride = ALIGN(width, 16);
    g_ver_stride = ALIGN(height, 16);

    const int bitrate = width * height / 8 * fps;
    const std::string rtmpPath = "rtmp://192.168.137.1:1935/live/app";

    // 4) 为 pipeline NV12 工作缓冲打开 system heap（非连续内存，RGA 通过 IOMMU 访问，不占 CMA）
    g_nv12_heap_fd = open("/dev/dma_heap/system", O_RDWR);
    if (g_nv12_heap_fd < 0) {
        perror("open /dev/dma_heap/system for pipeline nv12");
        g_camera.stopStreaming();
        g_camera.closeDevice();
        return -1;
    }

    // 5) 初始化 streamer（MPP/FFmpeg）
    if (init_streamer(width, height, fps, bitrate, rtmpPath.c_str(), enable_rtmp ? 1 : 0) != 0) {
        std::cerr << "init_streamer failed" << std::endl;
        close(g_nv12_heap_fd);
        g_nv12_heap_fd = -1;
        return -1;
    }

    // 6) 本地输出（便于离线查看）
    const std::string outPath = "../output.avi";
    int fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');
    cv::VideoWriter writer(outPath, fourcc, fps, cv::Size(width, height));
    if (!writer.isOpened()) {
        std::cerr << "Fail to create output video: " << outPath << std::endl;
        close_streamer();
        close(g_nv12_heap_fd);
        g_nv12_heap_fd = -1;
        return -1;
    }

    // 7) 初始化 YOLO 线程池
    ThreadPoll npu_pool("../model/yolov5s.rknn", PROCESS_THREAD_NUM);

    // 8) 启动三段流水线线程
    const auto t_start = std::chrono::steady_clock::now();

    std::thread t_cap(captureThreadFunc, frame_limit);
    std::thread t_proc(processThreadFunc, std::ref(npu_pool));
    std::thread t_write(writeThreadFunc, std::ref(writer));

    // 9) 等待线程收敛并停止队列
    t_cap.join();
    t_proc.join();

    g_readQueue.stop();
    g_writeQueue.stop();

    t_write.join();

    // 10) 清理资源
    writer.release();
    close_streamer();
    if (g_nv12_heap_fd >= 0) {
        close(g_nv12_heap_fd);
        g_nv12_heap_fd = -1;
    }
    g_camera.stopStreaming();
    g_camera.closeDevice();

    return 0;
}
