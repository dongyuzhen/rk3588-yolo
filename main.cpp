#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <future>
#include <iomanip>
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
constexpr int DEFAULT_FRAME_LIMIT = 1000;
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
    V4L2Camera::FramePacket packet{};
    std::chrono::steady_clock::time_point t_capture{};
};

// process 线程输出：携带 YOLO 结果图 + 时延中间值
struct YoloOutputFrame {
    int index{-1};
    V4L2Camera::FramePacket packet{};
    cv::Mat processed;
    std::chrono::steady_clock::time_point t_capture{};
};

// process 线程内部 inflight 映射项：
// index -> future，保证按序输出（next_out）
struct InflightTask {
    V4L2Camera::FramePacket packet{};
    std::chrono::steady_clock::time_point t_capture{};
    std::future<ProcessResult> fut;
};

// pipeline 中 NV12 工作缓冲使用单独 heap fd
int g_nv12_heap_fd = -1;

// 全局模块与队列
V4L2Camera g_camera;
SafeQueue<CapturedFrame> g_readQueue(READ_QUEUE_CAP);
SafeQueue<YoloOutputFrame> g_writeQueue(WRITE_QUEUE_CAP);

// 线程退出信号
std::atomic<bool> g_readFinish(false);
std::atomic<bool> g_processFinish(false);

// BGR -> NV12（RGA）
// 输入：YOLO画框后的 BGR
// 输出：NV12，供 MPP/streamer 使用
static void BGR_to_NV12(uint8_t* bgr, uint8_t* nv12, int width, int height) {
    rga_buffer_handle_t bgr_handle, yuv_handle;

    const size_t bgr_size = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
    const size_t nv12_size = static_cast<size_t>(g_hor_stride) * static_cast<size_t>(g_ver_stride) * 3 / 2;
    std::memset(nv12, 0x00, nv12_size);

    bgr_handle = importbuffer_virtualaddr(bgr, bgr_size);
    yuv_handle = importbuffer_virtualaddr(nv12, nv12_size);

    if (bgr_handle == 0 || yuv_handle == 0) {
        std::printf("import va failed.\n");
        return;
    }

    rga_buffer_t bgr_src = wrapbuffer_handle(bgr_handle, width, height, RK_FORMAT_BGR_888);
    rga_buffer_t yuv_src = wrapbuffer_handle(yuv_handle, width, height, RK_FORMAT_YCbCr_420_SP,
                                             g_hor_stride, g_ver_stride);

    int ret = imcheck(bgr_src, yuv_src, {}, {});
    if (ret != IM_STATUS_NOERROR) {
        std::printf("%d, imcheck error! %s\n", __LINE__, imStrError((IM_STATUS)ret));
    }

    ret = imcvtcolor(bgr_src, yuv_src, RK_FORMAT_BGR_888, RK_FORMAT_YCbCr_420_SP);
    if (ret != IM_STATUS_SUCCESS) {
        std::printf("%d, cvtColor error! %s\n", __LINE__, imStrError((IM_STATUS)ret));
    }

    if (bgr_handle) releasebuffer_handle(bgr_handle);
    if (yuv_handle) releasebuffer_handle(yuv_handle); 
}

// MJPEG 解码：
// - baseline：先拷贝 compressed bytes 再 imdecode
// - optimized：直接把 packet.data 作为 Mat 视图解码
cv::Mat decodeMjpegFrame(const V4L2Camera::FramePacket& packet) {
    if (!packet.data || packet.bytes_used == 0) return cv::Mat();
    cv::Mat encoded_view(1, static_cast<int>(packet.bytes_used), CV_8UC1, const_cast<uint8_t*>(packet.data));
    return cv::imdecode(encoded_view, cv::IMREAD_COLOR);
}

// ------------------------ 线程1：采集 ------------------------
// 从 V4L2 连续抓帧，把 FramePacket 放入 read 队列。
// 注意：这里只 DQ，不做 release，release 由写线程在“真正用完”后执行。
void captureThreadFunc(int frame_limit) {
    int idx = 0;
    while (idx < frame_limit) {
        V4L2Camera::FramePacket packet;
        if (!g_camera.captureFrame(packet, 100)) {
            continue;
        }

        CapturedFrame f;
        f.index = idx;
        f.packet = packet;
        f.t_capture = std::chrono::steady_clock::now();

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
                cv::Mat decoded = decodeMjpegFrame(in.packet);
                if (decoded.empty()) {
                    std::cerr << "[Process] decode failed index=" << in.index << std::endl;
                    g_camera.releaseFrame(in.packet);
                } else {
                    InflightTask task;
                    task.packet = in.packet;
                    task.t_capture = in.t_capture;
                    task.fut = npu_pool.submit_task_async(in.index, decoded);
                    inflight.emplace(in.index, std::move(task));
                }
            }
        }

        // B. 仅按 next_out 检查 future，就绪后输出到 writeQueue
        auto it = inflight.find(next_out);
        if (it != inflight.end()) {
            auto status = it->second.fut.wait_for(std::chrono::milliseconds(1));
            if (status == std::future_status::ready) {
                ProcessResult result = it->second.fut.get();
                if (!result.success || result.processed_img.empty()) {
                    std::cerr << "[Process] yolo failed index=" << next_out << " err=" << result.error_msg << std::endl;
                    g_camera.releaseFrame(it->second.packet);
                } else {
                    YoloOutputFrame out;
                    out.index = next_out;
                    out.packet = it->second.packet;
                    out.processed = std::move(result.processed_img);
                    out.t_capture = it->second.t_capture;
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

// ------------------------ 线程3：写出（颜色转换 + 编码/推流） ------------------------
// 这里分配一块 CMA NV12 工作缓冲复用：
// YOLO 输出(BGR) -> RGA NV12 -> process_frame(MPP/FFmpeg)
// 并在最后 releaseFrame 把采集 buffer 归还给 V4L2。
void writeThreadFunc(cv::VideoWriter& writer) {
    const size_t nv12_size = static_cast<size_t>(g_hor_stride) * static_cast<size_t>(g_ver_stride) * 3 / 2;
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

        if (!out.processed.empty()) {
            uint8_t* nv12_ptr = static_cast<uint8_t*>(nv12_cma.addr());
            BGR_to_NV12(out.processed.data, nv12_ptr, out.processed.cols, out.processed.rows);
            process_frame(nv12_ptr, static_cast<int>(nv12_cma.size()));
            writer.write(out.processed);
        }

        if (!g_camera.releaseFrame(out.packet)) {
            std::cerr << "[Write] releaseFrame failed index=" << out.index << std::endl;
        }
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

    // 2) 初始化相机（V4L2 + DMABUF）并启动采集
    if (!g_camera.init("/dev/video0", 1280, 720, BUFFER_NUM, V4L2_PIX_FMT_MJPEG)) {
        std::cerr << "Fail to init V4L2Camera" << std::endl;
        return -1;
    }

    if (!g_camera.startStreaming()) {
        std::cerr << "Fail to start V4L2 stream" << std::endl;
        return -1;
    }

    // 3) 编码/推流参数
    const int width = 1280;
    const int height = 720;
    const int fps = 30;

    g_width = width;
    g_height = height;
    // 统一按 16 对齐，避免 720->768 产生可见 padding 行
    g_hor_stride = ALIGN(width, 16);
    g_ver_stride = ALIGN(height, 16);

    const int bitrate = width * height / 8 * fps;
    const std::string rtmpPath = "rtmp://192.168.137.1:1935/live/app";

    // 4) 为 pipeline NV12 工作缓冲打开 CMA heap
    g_nv12_heap_fd = open("/dev/dma_heap/cma", O_RDWR);
    if (g_nv12_heap_fd < 0) {
        perror("open /dev/dma_heap/cma for pipeline nv12");
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
