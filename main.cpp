#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <future>
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
#include <sys/stat.h>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <set>

#include "SafeQueue.h"
#include "camera/cma_buffer.h"
#include "camera/v4l2_camera.h"
#include "mpp/mpp_encoder.h"
#include "streamer/rtsp.h"

#include "thread_poll.h"

//mpp
#include <rockchip/rk_mpi.h>

// RGA
#include "RgaUtils.h"
#include "drmrga.h"
#include "im2d.h"
#include "rga.h"
#include "rga_utils.h" // 提取出的 RGA 硬件转换工具函数
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
constexpr int DEFAULT_FRAME_LIMIT = 100000;
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
// 默认关闭本地保存，避免 CPU 软编码；传 --save-local 开启
bool parseEnableLocalSave(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--save-local") == 0) {
            return true;
        }
    }
    return false;
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

// 报警事件：写线程 → 报警引擎
// 仅在检测到违规时才克隆帧，无违规帧零开销
struct AlarmEvent {
    std::set<std::string>                 violation_labels; // 本帧所有违规类别
    cv::Mat                               snapshot;         // 深拷贝 BGR，与 dmabuf 生命周期完全解耦
    std::chrono::steady_clock::time_point frame_time;       // 帧采集时刻（用于时基防抖）
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
SafeQueue<EncodedPacket> rtsp_queue(24);
// 线程退出信号
std::atomic<bool> g_readFinish(false);
std::atomic<bool> g_processFinish(false);
std::atomic<bool> g_writeFinish(false);  // 写线程完成后通知报警引擎退出

// 报警引擎队列（容量16，溢出时自动丢弃最老事件）
SafeQueue<AlarmEvent> g_alarmQueue(16);


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
        // 一句话完成：入队，如果满了就自动弹掉最老的帧，并用 lambda 释放它（零延迟策略）
        g_readQueue.enqueue_drop_oldest(f, [](CapturedFrame& dropped) {
            g_camera.releaseFrame(dropped.packet);
        });
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
            auto status = it->second.fut.wait_for(std::chrono::milliseconds(0));
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

        // D. 自适应等待：有 inflight 时等 next_out 完成，否则短暂休眠
        it = inflight.find(next_out);
        if (it != inflight.end()) {
            it->second.fut.wait_for(std::chrono::milliseconds(2));
        } else if (inflight.empty() && g_readQueue.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    g_processFinish = true;
}

// ------------------------ 线程4：报警引擎（防抖 + 多类独立冷却 + 推送） ------------------------
// 设计亮点：
//   1. 单一职责：从写线程剥离 IO 密集型报警逻辑，视频流水线不再受网络抖动影响
//   2. 漏桶防抖：出现 +1，缺席 -2，归零才清除，容忍 NPU 偶发漏检（约33%容错率）
//   3. 多类别独立冷却：每类违规各自维护30s冷却期，互不压制（替代原来的 clear() 全局重置）
void alarmEngineThreadFunc() {
    struct LabelState {
        int    score     = 0;
        std::chrono::steady_clock::time_point last_seen  = std::chrono::steady_clock::now() - std::chrono::seconds(1000);
        std::chrono::steady_clock::time_point last_alarm = std::chrono::steady_clock::now() - std::chrono::seconds(1000);
    };
    std::map<std::string, LabelState> states;

    constexpr int DEBOUNCE_SCORE   = 15;  // 积分≥15触发报警（约0.5s@30fps）
    constexpr int SCORE_INC        = 1;   // 每帧出现：+1
    constexpr int SCORE_DEC        = 2;   // 每帧缺席：-2（漏桶衰减，33%容错率）
    constexpr int COOLDOWN_SEC     = 30;  // 每类独立冷却（秒），互不压制
    constexpr int DECAY_TIMEOUT_MS = 300; // 超过300ms未见同类事件，视为缺席帧

    while (true) {
        if (g_writeFinish && g_alarmQueue.empty()) break;

        AlarmEvent ev;
        if (!g_alarmQueue.dequeue(ev)) {
            if (g_writeFinish) break;
            continue;
        }

        auto now = ev.frame_time;

        // 1. 漏桶衰减：对本帧缺席的类别按超时惩罚扣分
        for (auto& [label, st] : states) {
            if (ev.violation_labels.count(label) == 0) {
                auto absent_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_seen).count();
                if (absent_ms > DECAY_TIMEOUT_MS)
                    st.score = std::max(0, st.score - SCORE_DEC);
            }
        }
        // 2. 出现的类别积分 +1，更新最后出现时间
        for (const auto& label : ev.violation_labels) {
            auto& st     = states[label];
            st.score    += SCORE_INC;
            st.last_seen = now;
        }

        // 3. 检查各类别是否独立触发报警
        for (auto& [label, st] : states) {
            if (st.score < DEBOUNCE_SCORE) continue;
            auto cooldown_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - st.last_alarm).count();
            if (cooldown_elapsed < COOLDOWN_SEC) continue;

            // ── 触发报警 ──
            st.last_alarm = now;
            st.score      = 0; // 触发后归零，防持续轰炸

            auto t  = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream time_ss;
            time_ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
            std::string time_str = time_ss.str();

            std::string img_path = "violations/alarm_" + label + "_" + time_str + ".jpg";
            std::replace(img_path.begin(), img_path.end(), ' ', '_');

            std::string msg = "\u26a0\ufe0f 安全告警：检测到违规行为 [" + label + "]，请相关班组立即整改！";
            std::string webhook_url = "https://open.feishu.cn/open-apis/bot/v2/hook/94522d3e-46d7-420e-be43-7965cfdb1d74";
            std::string curl_cmd = "curl -s --max-time 5 -X POST -H 'Content-Type: application/json'"
                                   " -d '{\"msg_type\":\"text\",\"content\":{\"text\":\"" + msg + "\"}}' " + webhook_url;

            // 异步脱缰线程：存图 + Webhook，物理隔离 IO，不阻塞报警引擎
            std::thread([snap = ev.snapshot.clone(), img_path, curl_cmd, time_str, label]() {
                cv::imwrite(img_path, snap);
                if (system(curl_cmd.c_str()) != 0) {
                    std::cerr << "[ERROR] Webhook failed for " << label << " at " << time_str << std::endl;
                    std::ofstream err_log("violations/error_log.txt", std::ios::app);
                    if (err_log.is_open())
                        err_log << time_str << ", WEBHOOK_FAILED, " << label << "\n";
                }
            }).detach();

            // 结构化 CSV 日志（追加写入极快，留在报警引擎线程）
            if (std::ofstream log("violations/alarm_log.csv", std::ios::app); log.is_open())
                log << time_str << "," << label << "," << img_path << "\n";

            std::cout << "[ALARM] " << label << " -> " << img_path << " (Async IO)" << std::endl;
        }

        // 清理长时间不活跃的状态条目，防止 map 无限增长
        for (auto it = states.begin(); it != states.end();) {
            auto inactive_sec = std::chrono::duration_cast<std::chrono::seconds>(now - it->second.last_seen).count();
            it = (it->second.score <= 0 && inactive_sec > 60) ? states.erase(it) : std::next(it);
        }
    }
}

// ------------------------ 线程3：写出（画框 + 编码/推流） ------------------------
void writeThreadFunc(cv::VideoWriter *writer) {
    // 两块复用缓冲：BGR 用于画框，NV12 用于送编码
    // 注意：NV12 buffer 大小要按 stride 计算，且要保证足够大，否则 RGA 转换会失败
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
            
            // 智慧工地动态报警逻辑（红绿灯配色）
            std::string label_str = det.label;
            cv::Scalar box_color;
            if (label_str == "no helmet" || label_str == "no vest" || label_str == "no glove" || label_str == "no boots") {
                box_color = cv::Scalar(0, 0, 255); // 违规：红色 (BGR)
            } else if (label_str == "helmet_blue" || label_str == "helmet_white" || label_str == "helmet_yellow" || 
                       label_str == "vest" || label_str == "boots" || label_str == "helmet on" || label_str == "gloves") {
                box_color = cv::Scalar(0, 255, 0); // 合规：绿色 (BGR)
            } else {
                box_color = cv::Scalar(255, 0, 0); // 普通(如 person)：蓝色 (BGR)
            }

            cv::rectangle(bgr_mat,
                          cv::Point(det.box.xmin, det.box.ymin),
                          cv::Point(det.box.xmax, det.box.ymax),
                          box_color, 2);

            char label_buf[64];
            snprintf(label_buf, sizeof(label_buf), "%s: %.1f%%", det.label, det.box_conf * 100.f);
            cv::Point text_org(det.box.xmin, std::max(20, det.box.ymin - 8));
            
            // 黑色描边
            cv::putText(bgr_mat, label_buf, text_org,
                        cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 0, 0), 3, cv::LINE_8);
            // 黄色正文
            cv::putText(bgr_mat, label_buf, text_org,
                        cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 255), 1, cv::LINE_8);
        }

        // 检测违规 → 推送至报警引擎（快速路径，仅违规帧才克隆）
        {
            std::set<std::string> vlabels;
            for (int i = 0; i < out.detections.box_count; i++) {
                std::string lbl = out.detections.result[i].label;
                if (lbl == "no helmet" || lbl == "no vest" || lbl == "no glove" || lbl == "no boots")
                    vlabels.insert(lbl);
            }
            if (!vlabels.empty()) {
                AlarmEvent ev;
                ev.violation_labels = std::move(vlabels);
                ev.snapshot         = bgr_mat.clone(); // 深拷贝，与 dmabuf/writeThread 生命周期解耦
                ev.frame_time       = std::chrono::steady_clock::now();
                g_alarmQueue.enqueue_drop_oldest(ev, [](AlarmEvent &) {}); // 队列满时丢弃最老事件
            }
        }
        // ========================================================

        // 3. BGR（带框）→ NV12（RGA 硬件转换），送 MPP 编码（零拷贝，传 fd）
        BGR_to_NV12_rga(bgr_ptr, g_width, g_height, nv12_ptr, g_hor_stride, g_ver_stride);
        process_frame_fd(nv12_cma.fd(), static_cast<int>(nv12_cma.size()));

        // 4. 本地 VideoWriter（仅 --save-local 时启用）
        if (writer && writer->isOpened())
            writer->write(bgr_mat);

        // 5. 归还 V4L2 buffer
        if (!g_camera.releaseFrame(out.packet))
            std::cerr << "[Write] releaseFrame failed index=" << out.index << std::endl;
    }

    // 通知报警引擎：写线程已完成，可以安全退出
    g_writeFinish = true;
}

//入队回调
int enc_pkt_enqueue(void* pkt, uint8_t* data_ptr, int size)
{
    // 强转回真身
    MppPacket mpp_pkt = (MppPacket)pkt;
    EncodedPacket enc_pkt;
    enc_pkt.size = size;

    // 从硬件包里榨取时间戳
    enc_pkt.pts = mpp_packet_get_pts(mpp_pkt);
    enc_pkt.dts = mpp_packet_get_dts(mpp_pkt);
    
    // 用智能指针接管 C 语言的包释放
    // 当这个 shared_ptr 走到生命周期尽头（也就是 RTSP 发送完之后），会自动执行这里面的 lambda 销毁
    enc_pkt.data = std::shared_ptr<uint8_t>(data_ptr, [mpp_pkt](uint8_t* p) {
        MppPacket temp_pkt = mpp_pkt; // 需要一个拷贝去给 deinit 传指针
        mpp_packet_deinit(&temp_pkt);
    });
    // 4. 压入队列，丢弃老包时 shared_ptr 会自动调用上面的 deinit 释放内存
    rtsp_queue.enqueue_drop_oldest(enc_pkt, [](EncodedPacket& dropped){});
    return 0;
}

} // namespace

// ------------------------ 主流程 ------------------------
int main(int argc, char* argv[]) {
    // 创建告警抓拍持久化目录
    mkdir("violations", 0777);

    // 1) 解析参数
    const bool enable_rtmp = parseEnableRtmp(argc, argv);
    const bool enable_local_save = parseEnableLocalSave(argc, argv);
    const int frame_limit = DEFAULT_FRAME_LIMIT;

    std::cout << "[Config] frames=" << frame_limit
              << " rtmp=" << (enable_rtmp ? "on" : "off")
              << " save=" << (enable_local_save ? "on" : "off") << std::endl;

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

    // 4) 为 pipeline NV12 工作缓冲打开 system heap（非连续内存，RGA 通过 IOMMU 访问，不占 CMA）
    g_nv12_heap_fd = open("/dev/dma_heap/system", O_RDWR);
    if (g_nv12_heap_fd < 0) {
        perror("open /dev/dma_heap/system for pipeline nv12");
        g_camera.stopStreaming();
        g_camera.closeDevice();
        return -1;
    }

    
    const std::string rtspPath = "rtsp://192.168.163.8:8554/live"; // 换成你的 RTSP 服务地址
    
    // 5) 初始化 MPP 和 RTSP 推流器
    if (init_mpp_encoder(width, height, fps, bitrate, enc_pkt_enqueue) != 0) {
        std::cerr << "init_mpp_encoder failed" << std::endl;
        close(g_nv12_heap_fd);
        g_nv12_heap_fd = -1;
        return -1;
    }

    // --- 架构核心：在此处启动网络推流后端 ---
    uint8_t* sps_data = nullptr;
    int sps_size = 0;
    get_mpp_sps_pps(&sps_data, &sps_size); // 拿到弥足珍贵的 SPS/PPS 头

    // 创建推流器对象，它会在后台自动启动推流死循环，并且生命周期绑定在 main 函数上
    RtspStreamer rtsp_streamer(rtspPath, width, height, fps, rtsp_queue, sps_data, sps_size);
    std::cout << "[Main] RTSP Streamer started: " << rtspPath << std::endl;


    // 6) 本地输出（默认关闭省 CPU，传 --save-local 开启）
    const std::string outPath = "../output.avi";
    cv::VideoWriter* writer = nullptr;
    if (enable_local_save) {
        int fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');
        writer = new cv::VideoWriter(outPath, fourcc, fps, cv::Size(width, height));
        if (!writer->isOpened()) {
            std::cerr << "Fail to create output video: " << outPath << std::endl;
            delete writer;
            writer = nullptr;
        }
    }
    
    //  初始化 YOLO 线程池 (传入相机分辨率用于动态计算 Letterbox)
    ThreadPoll npu_pool("../model/best.rknn", PROCESS_THREAD_NUM, width, height);

    // 启动三段流水线线程
    const auto t_start = std::chrono::steady_clock::now();

    std::thread t_cap(captureThreadFunc, frame_limit);
    std::thread t_proc(processThreadFunc, std::ref(npu_pool));
    std::thread t_write(writeThreadFunc, writer);
    std::thread t_alarm(alarmEngineThreadFunc); // 报警引擎：独立线程，接管防抖/冷却/推送

    //  等待线程收敛并停止队列（流水线顺序退出）
    t_cap.join();
    t_proc.join();

    g_readQueue.stop();
    g_writeQueue.stop();

    t_write.join();
    g_alarmQueue.stop(); // 写线程已退出，安全停止报警队列
    t_alarm.join();

    //  清理资源
    if (writer) { writer->release(); delete writer; }
    close_mpp_encoder();
    if (g_nv12_heap_fd >= 0) {
        close(g_nv12_heap_fd);
        g_nv12_heap_fd = -1;
    }
    g_camera.stopStreaming();
    g_camera.closeDevice();

    return 0;
}
