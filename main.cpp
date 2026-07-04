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
#include <csignal>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <set>

#include "SafeQueue.h"
#include "camera/cma_buffer.h"
#include "camera/v4l2_camera.h"
#include "mpp/mpp_encoder.h"
#include "streamer/rtsp.h"
#include "tracker/simple_tracker.h"  // IoU 目标追踪
#include "watchdog.h"               // 摄像头/NPU 异常看门狗

#include "pool/thread_poll.h"

#include "bench/perf_timer.h"
#include "bench/rga_vs_opencv.h"

//mpp
#include <rockchip/rk_mpi.h>

// RGA
#include "RgaUtils.h"
#include "drmrga.h"
#include "im2d.h"
#include "rga.h"
#include "rga_utils.h" // 提取出的 RGA 硬件转换工具函数
#include "memory"
#define ALIGN(x, a) (((x) + (a)-1) & ~((a)-1))


namespace {

// ------------------------ 全局运行参数 ------------------------
int g_width = 0;
int g_height = 0;
int g_hor_stride = 0;
int g_ver_stride = 0;

// 采集缓冲数（V4L2 + DMABUF）
// 需要足够容纳流水线延迟：
//   YOLO 推理 ~50ms + MPP H.265 编码等待 ~16ms = 总准备 >= 4帧
// 12 = 安全宿，可容纳纺线延迟干扫，避免 POLLERR
constexpr int BUFFER_NUM = 12;
// 队列深度：releaseFrame 在 RGA blend 后立即调用（早于 MPP 编码），
// 因此 buffer 归还很及时，不需要刻意压低容量。
// 留 4 帧吸收 YOLO 推理抖动即可。
constexpr size_t READ_QUEUE_CAP  = 4;
constexpr size_t WRITE_QUEUE_CAP = 4;
// 默认处理帧数
constexpr int DEFAULT_FRAME_LIMIT = 1500;
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
// RGA vs OpenCV 对比测试模式：采集一帧 → 跑完所有对比 → 退出
bool parseBenchRga(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--bench-rga") == 0) {
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

// SIGINT 优雅退出标志
std::atomic<bool> g_quit_requested{false};


// ------------------------ 线程1：采集 ------------------------
// 从 V4L2 连续抓帧，把 FramePacket 放入 read 队列。
// 注意：这里只 DQ，不做 release，release 由写线程在"真正用完"后执行。
// 看门狗协议：连续采集失败 30 次 → 请求看门狗重启摄像头，自旋等待恢复完成。
void captureThreadFunc(int frame_limit) {
    int idx = 0;
    while (idx < frame_limit && !g_quit_requested) {
        // ── 等待看门狗完成摄像头恢复 ──────────────────────────────────────
        if (g_cam_recovering.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        FramePacket_t packet;
        if (!g_camera.captureFrame(packet, 100)) {
            int fails = ++g_cam_fail_count;
            if (fails >= 30 && !g_cam_need_restart.exchange(true)) {
                std::cerr << "[CAP] camera stall detected (" << fails
                          << " consecutive failures), requesting watchdog restart." << std::endl;
            }
            continue;
        }

        // 采集成功：清零失败计数
        g_cam_fail_count.store(0, std::memory_order_relaxed);

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


// 看门狗协议：连续 NPU 推理失败 10 次 → 请求看门狗重建 ThreadPoll，自旋等待。
void processThreadFunc() {
    // MAX_INFLIGHT 必须与 READ_QUEUE_CAP + WRITE_QUEUE_CAP + 1 一起满足 < BUFFER_NUM(12)
    // 4 + 4 + 2 + 1 = 11 < 12，确保相机驱动队列永远有至少 1 个 QBUF'd buffer 待命
    constexpr size_t MAX_INFLIGHT    = 4;
    constexpr int    NPU_FAIL_THRESH = 10;
    int next_out = -1;
    std::map<int, InflightTask> inflight;

    while (true) {
        // ── 等待看门狗完成 NPU 恢复 ───────────────────────────────────────
        if (g_npu_recovering.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        ThreadPoll* pool = g_npu_pool_ptr;
        if (!pool) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // A. 尽量从 readQueue 取新帧并提交 YOLO
        if (!g_readQueue.empty() && inflight.size() < MAX_INFLIGHT) {
            CapturedFrame in;
            if (g_readQueue.dequeue(in)) {
                InflightTask task;
                task.packet = in.packet;
                task.fut = pool->submit_task_async(in.index, task.packet.dmabuf_fd);
                inflight.emplace(in.index, std::move(task));
            }
        }

        // B. 仅按 next_out 检查 future，就绪后输出到 writeQueue
        if (!inflight.empty()) {
            if (next_out == -1 || next_out < inflight.begin()->first) {
                next_out = inflight.begin()->first;
            }
        }

        auto it = inflight.find(next_out);
        if (it != inflight.end()) {
            auto status = it->second.fut.wait_for(std::chrono::milliseconds(0));
            if (status == std::future_status::ready) {
                ProcessResult result = it->second.fut.get();
                if (!result.success) {
                    std::cerr << "[Process] yolo failed index=" << next_out
                              << " err=" << result.error_msg << std::endl;
                    g_camera.releaseFrame(it->second.packet);

                    int fails = ++g_npu_fail_count;
                    if (fails >= NPU_FAIL_THRESH && !g_npu_need_restart.exchange(true)) {
                        std::cerr << "[PROC] NPU stall detected (" << fails
                                  << " consecutive failures), requesting watchdog restart."
                                  << std::endl;
                    }
                } else {
                    g_npu_fail_count.store(0, std::memory_order_relaxed);
                    YoloOutputFrame out;
                    out.index      = next_out;
                    out.packet     = it->second.packet;
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
            // 从环境变量读取 Webhook URL，避免密钥硬编码注入代码
            const char* webhook_env = getenv("FEISHU_WEBHOOK_URL");
            std::string webhook_url = webhook_env ? webhook_env : "";

            // 对标签中可能含有的 shell 单引号 ' 进行转义，防止命令注入
            std::string safe_msg = msg;
            std::string escaped_msg;
            for (char c : safe_msg) {
                if (c == '\'') {
                    escaped_msg += "'\"'\"'";
                } else {
                    escaped_msg += c;
                }
            }
            std::string curl_cmd;
            if (!webhook_url.empty()) {
                curl_cmd = "curl -s --max-time 5 -X POST -H 'Content-Type: application/json'"
                           " -d '{\"msg_type\":\"text\",\"content\":{\"text\":\"" + escaped_msg + "\"}}' " + webhook_url;
            }

            // 异步脱缰线程：存图 + Webhook，物理隔离 IO，不阻塞报警引擎
            std::thread([snap = ev.snapshot.clone(), img_path, curl_cmd, time_str, label]() {
                cv::imwrite(img_path, snap);
                if (!curl_cmd.empty() && system(curl_cmd.c_str()) != 0) {
                    std::cerr << "[ERROR] Webhook failed for " << label << " at " << time_str << std::endl;
                    std::ofstream err_log("violations/error_log.txt", std::ios::app);
                    if (err_log.is_open())
                        err_log << time_str << ", WEBHOOK_FAILED, " << label << "\n";
                } else if (curl_cmd.empty()) {
                    std::cerr << "[WARN] FEISHU_WEBHOOK_URL not set, skipping webhook for " << label << std::endl;
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

// ------------------------ 线程3：写出（画框 + IoU追踪 + 编码/推流） ------------------------
// 新增：SimpleTracker 在此线程内单实例运行（写线程串行，无需加锁）
// 追踪输出：每个检测框附加稳定 track_id，报警引擎按 track_id 进行 ID 级别去重
void writeThreadFunc(cv::VideoWriter *writer) {
    // 两个缓冲：BGRA 透明图层用于画框，NV12 用于送编码
    // 注意：NV12 buffer 大小要按 stride 计算
    const size_t osd_size  = static_cast<size_t>(g_width) * g_height * 4; // BGRA

    CmaBuffer osd_cma;
    if (!osd_cma.allocate(g_nv12_heap_fd, osd_size, "pipeline_osd_work")) {
        std::cerr << "[Write] failed to allocate OSD cma buffer" << std::endl;
        g_writeFinish = true;
        return;
    }

    // 获取 MPP 内部 buffer fd
    int mpp_fd = get_mpp_input_fd();
    if (mpp_fd < 0) {
        std::cerr << "[Write] failed to get mpp fd" << std::endl;
        g_writeFinish = true;
        return;
    }

    // ★ 预缓存 RGA handle：osd_fd 和 mpp_fd 全生命周期不变，import 一次即可
    //   每帧节省 2 次 importbuffer_fd 内核调用 (~0.3ms/帧)
    rga_buffer_handle_t osd_handle_cached = importbuffer_fd(osd_cma.fd(), g_width, g_height, RK_FORMAT_BGRA_8888);
    rga_buffer_handle_t mpp_handle_cached = importbuffer_fd(mpp_fd, g_width, g_height, RK_FORMAT_YCbCr_420_SP);
    if (osd_handle_cached == 0 || mpp_handle_cached == 0) {
        std::cerr << "[Write] failed to cache RGA handles" << std::endl;
        if (osd_handle_cached) releasebuffer_handle(osd_handle_cached);
        if (mpp_handle_cached) releasebuffer_handle(mpp_handle_cached);
        g_writeFinish = true;
        return;
    }

    // IoU 追踪器：在写线程内单实例运行，串行调用无需加锁
    SimpleTracker tracker;

    // 创建 OpenCV 透明画布 (BGRA)，直接绑定到 OSD 的 DMA 虚拟地址上
    cv::Mat bgra_mat(g_height, g_width, CV_8UC4, osd_cma.addr());

    int write_frame_cnt = 0;

    while (true) {
        PERF_SCOPE("WriteFrame_total");
        if (g_processFinish && g_writeQueue.empty()) break;

        YoloOutputFrame out;
        if (!g_writeQueue.dequeue(out)) {
            if (g_processFinish) break;
            continue;
        }

        // 1. 每帧清空 OSD 画布为透明
        osd_cma.syncStartWrite(); // 获取写权限，准备 CPU 画图
        // ★ 优化：BGRA 全零 = 全透明，直接用 memset 比 cv::Mat::setTo 快 ~3x
        //   cv::Mat::setTo 走通用像素赋值路径，memset 走 ARM64 DC ZVA 缓存零填充
        memset(osd_cma.addr(), 0, osd_size);

        // 2. IoU 追踪器更新：为每个检测框分配稳定 track_id
        std::vector<TrackedResult> tracked;
        {
            PERF_SCOPE("Tracker_IoU");
            tracked = tracker.update(out.detections);
        }

        // 3. 在 BGRA 上画检测框和标签（含 track_id）
        {
            PERF_SCOPE("OSD_draw");
        for (const TrackedResult& tr : tracked) {
            const detect_result_t& det = tr.det;

            // 智慧工地动态报警逻辑（红绿灯配色 - BGRA格式）
            std::string label_str = det.label;
            cv::Scalar box_color;
            if (label_str == "no-helmet" || label_str == "no-vest") {
                box_color = cv::Scalar(0, 0, 255, 255); // 违规：红色 (B=0, G=0, R=255)
            } else if (label_str == "helmet" || label_str == "vest") {
                box_color = cv::Scalar(0, 255, 0, 255); // 合规：绿色 (B=0, G=255, R=0)
            } else {
                box_color = cv::Scalar(255, 0, 0, 255); // 普通：蓝色 (B=255, G=0, R=0)
            }

            // 强制将坐标对齐到偶数，避免 RGA 渲染 NV12 时的 UV 抽样错位(导致的彩色虚线/色度撕裂)
            int xmin = det.box.xmin & ~1;
            int ymin = det.box.ymin & ~1;
            int xmax = det.box.xmax & ~1;
            int ymax = det.box.ymax & ~1;

            cv::rectangle(bgra_mat,
                          cv::Point(xmin, ymin),
                          cv::Point(xmax, ymax),
                          box_color, 4);

            // 标签格式：label: conf%  #track_id
            char label_buf[80];
            snprintf(label_buf, sizeof(label_buf), "%s: %.1f%%  #%d",
                     det.label, det.box_conf * 100.f, tr.track_id);
            cv::Point text_org(det.box.xmin, std::max(20, det.box.ymin - 8));

            // 黑色描边 (Alpha=255)
            cv::putText(bgra_mat, label_buf, text_org,
                        cv::FONT_HERSHEY_SIMPLEX, 1.2,
                        cv::Scalar(0, 0, 0, 255), 4, cv::LINE_8);
            // 黄色正文 (Alpha=255) (B=0, G=255, R=255)
            cv::putText(bgra_mat, label_buf, text_org,
                        cv::FONT_HERSHEY_SIMPLEX, 1.2,
                        cv::Scalar(0, 255, 255, 255), 2, cv::LINE_8);
        }
        } // OSD_draw scope

        // 4. 检测违规 → 推送至报警引擎（track_id 级别去重，快速路径）
        {
            std::set<std::string> vlabels;
            for (const TrackedResult& tr : tracked) {
                std::string lbl = tr.det.label;
                // 标签携带 track_id（格式 "no-helmet#3"），报警引擎按此做 ID 级冷却
                if (lbl == "no-helmet" || lbl == "no-vest")
                    vlabels.insert(lbl + "#" + std::to_string(tr.track_id));
            }
            if (!vlabels.empty()) {
                AlarmEvent ev;
                ev.violation_labels = std::move(vlabels);
                
                // 发生违规时，提取带框的 BGR 图像用于快照报警（仅在这时发生 NV12->BGR）
                cv::Mat snapshot_bgr(g_height, g_width, CV_8UC3);
                NV12_to_BGR_rga(out.packet.dmabuf_fd, g_width, g_height, snapshot_bgr.data);
                
                // 在生成的 BGR 上再次画框（用于报警截图保存，这部分走的是正常的 BGR 软件流程，无需反色）
                for (const TrackedResult& tr : tracked) {
                    const detect_result_t& det = tr.det;
                    std::string lbl = det.label;
                    cv::Scalar c = (lbl == "no-helmet" || lbl == "no-vest") ? cv::Scalar(0,0,255) : 
                                   (lbl == "helmet" || lbl == "vest") ? cv::Scalar(0,255,0) : cv::Scalar(255,0,0);
                    cv::rectangle(snapshot_bgr, cv::Point(det.box.xmin, det.box.ymin), cv::Point(det.box.xmax, det.box.ymax), c, 4);
                }

                ev.snapshot         = snapshot_bgr; // 无需 clone，因为 snapshot_bgr 是我们刚分配的
                ev.frame_time       = std::chrono::steady_clock::now();
                g_alarmQueue.enqueue_drop_oldest(ev, [](AlarmEvent &) {}); // 队列满时丢弃最老事件
            }
        }
        // ========================================================

        // 5. 零拷贝硬件 OSD 合成：将透明图层 osd_cma 混合到底层的 camera NV12 画面上，并输出到 MPP 内部缓冲
        osd_cma.syncEndWrite(); // 画完必须 Flush Cache 到物理内存！否则硬件 RGA 会读到脏数据（即截图里的画面鬼影/撕裂）
        
        begin_mpp_input_sync(); // 通知内核 MPP buffer 准备被写入
        {
            PERF_SCOPE("RGA_blend");
            copy_and_blend_osd_rga_cached(out.packet.dmabuf_fd,
                                          osd_handle_cached, mpp_handle_cached,
                                          g_width, g_height, g_hor_stride, g_ver_stride);
        }

        // ★ 关键：RGA blend 完成后，摄像头原始帧已经不再需要（数据已合成进 mpp_fd）
        // 立即归还 V4L2 buffer，不要等 MPP 编码完成！
        // 否则：encode_mpp_frame 阻塞等 H.265 编码时，camera buffer 被长期占用 → POLLERR
        if (!g_camera.releaseFrame(out.packet))
            std::cerr << "[Write] releaseFrame failed index=" << out.index << std::endl;
        {
            PERF_SCOPE("MPP_encode");
            encode_mpp_frame();
        }

        // 6. 本地 VideoWriter（仅 --save-local 时启用）
        // 如果开启了本地保存，为了简单，我们可以保存刚才画完的 NV12_cma 转换后的 BGR
        if (writer && writer->isOpened()) {
            cv::Mat out_bgr(g_height, g_width, CV_8UC3);
            NV12_to_BGR_rga(mpp_fd, g_width, g_height, out_bgr.data);
            writer->write(out_bgr);
        }

        // ── 定期输出性能报告 + CSV（每300帧） ────────────────────
#ifdef BENCH_MODE
        if (++write_frame_cnt % 300 == 0) {
            PerfTimer::report("Pipeline @ frame " + std::to_string(write_frame_cnt));
            PerfTimer::write_csv("violations/perf_profile.csv");
        }
#endif
    }

    // ── 性能报告：最终汇总 + CSV ─────────────────────────────────
    PERF_REPORT("FINAL Pipeline Profile");
    PERF_CSV("violations/perf_profile.csv");

    // ★ 释放预缓存的 RGA handle
    releasebuffer_handle(osd_handle_cached);
    releasebuffer_handle(mpp_handle_cached);

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
    
    // 由于 mpp.c 内部会在 write_frame 返回后立刻调用 mpp_packet_deinit(&packet)，
    // 我们必须深拷贝数据，否则推流线程读取时就是 Use-After-Free，且之前的 lambda 会造成 Double-Free 导致硬件死锁！
    uint8_t* safe_data = new uint8_t[size];
    memcpy(safe_data, data_ptr, size);

    // 用智能指针接管 safe_data 的生命周期
    enc_pkt.data = std::shared_ptr<uint8_t>(safe_data, std::default_delete<uint8_t[]>());

    // 4. 压入队列，丢弃老包时 shared_ptr 会自动调用上面的 deinit 释放内存
    rtsp_queue.enqueue_drop_oldest(enc_pkt, [](EncodedPacket& dropped){});
    return 0;
}

// SIGINT 信号处理器：请求优雅退出
void sigint_handler(int) {
    g_quit_requested = true;
    std::cout << "\n[SIGINT] Graceful shutdown requested..." << std::endl;
}

} // namespace

// ------------------------ 主流程 ------------------------
int main(int argc, char* argv[]) {
    // 注册 SIGINT 处理器
    std::signal(SIGINT, sigint_handler);

    // 创建告警抓拍持久化目录
    mkdir("violations", 0777);

    // 1) 解析参数
    const bool enable_rtmp = parseEnableRtmp(argc, argv);
    const bool enable_local_save = parseEnableLocalSave(argc, argv);
    const int frame_limit = DEFAULT_FRAME_LIMIT;

    std::cout << "[Config] frames=" << frame_limit
              << " rtmp=" << (enable_rtmp ? "on" : "off")
              << " save=" << (enable_local_save ? "on" : "off") << std::endl;

    // 经过深度排查，cmdline 的 cma=128M 会破坏设备树(DTB)的 CMA 节点，导致 /dev/dma_heap/cma 丢失。
    // 2) 初始化相机（V4L2 + MMAP Export）并启动采集
    // 我们不再从用户态创建 CmaBufferPool 给相机用了，直接让相机的驱动在内核态用 MMAP 自行分配最合适的内存（比如它保留的 CMA），
    // 并且相机会自动导出这些内存的 DMABUF fd 供 RGA/MPP 零拷贝使用，彻底解决各种 DMA IOCTL 问题！
    if (!g_camera.init("/dev/video-camera0", 1920, 1080, BUFFER_NUM, V4L2_PIX_FMT_NV12)) {
        std::cerr << "Fail to init V4L2Camera" << std::endl;
        return -1;
    }

    if (!g_camera.startStreaming()) {
        std::cerr << "Fail to start V4L2 stream" << std::endl;
        return -1;
    }

    // ── --bench-rga: RGA vs OpenCV 对比测试模式 ──────────────────
    if (parseBenchRga(argc, argv)) {
        std::cout << "[Bench] Capturing one frame for RGA vs OpenCV test..." << std::endl;
        FramePacket_t bench_pkt;
        if (!g_camera.captureFrame(bench_pkt, 2000)) {
            std::cerr << "[Bench] Failed to capture frame" << std::endl;
            g_camera.stopStreaming();
            g_camera.closeDevice();
            return -1;
        }
        run_rga_vs_opencv_benchmark(bench_pkt.dmabuf_fd, 1920, 1080, 100);
        g_camera.releaseFrame(bench_pkt);
        g_camera.stopStreaming();
        g_camera.closeDevice();
        std::cout << "[Bench] Done." << std::endl;
        return 0;
    }

    // 3) 编码/推流参数（与摄像头采集分辨率一致）
    const int width = 1920;
    const int height = 1080;
    const int fps = 60;

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
    std::unique_ptr<cv::VideoWriter> writer;
    if (enable_local_save) {
        int fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');
        writer = std::make_unique<cv::VideoWriter>(outPath, fourcc, fps, cv::Size(width, height));
        if (!writer->isOpened()) {
            std::cerr << "Fail to create output video: " << outPath << std::endl;
            writer.reset(); // 自动释放，无需手动 delete
        }
    }
    
    //  初始化 YOLO 线程池 (传入相机分辨率用于动态计算 Letterbox)
    ThreadPoll npu_pool("../model/best.rknn", PROCESS_THREAD_NUM, width, height);

    // ── 初始化看门狗全局上下文（必须在启动线程前完成） ────────────────────────
    g_npu_pool_ptr = &npu_pool;       // 处理线程通过 g_npu_pool_ptr 间接访问
    g_cam_width    = width;
    g_cam_height   = height;
    g_cam_bufnum   = BUFFER_NUM;

    // 启动四段流水线线程 + 看门狗线程
    const auto t_start = std::chrono::steady_clock::now();

    std::thread t_cap(captureThreadFunc, frame_limit);
    std::thread t_proc(processThreadFunc);           // 通过 g_npu_pool_ptr 访问 NPU
    std::thread t_write(writeThreadFunc, writer.get());
    std::thread t_alarm(alarmEngineThreadFunc);       // 报警引擎：独立线程，接管防抖/冷却/推送
    std::thread t_watchdog(watchdogThreadFunc, std::ref(g_camera)); // 看门狗：监控摄像头/NPU

    //  等待线程收敛并停止队列（流水线顺序退出）
    t_cap.join();
    t_proc.join();

    g_readQueue.stop();
    g_writeQueue.stop();

    t_write.join();
    g_alarmQueue.stop(); // 写线程已退出，安全停止报警队列
    t_alarm.join();

    // 通知看门狗退出并等待
    g_watchdog_stop = true;
    t_watchdog.join();
    g_npu_pool_ptr = nullptr;  // 指针清空，防止悬空

    //  清理资源
    if (writer) { writer->release(); } // unique_ptr 自动释放，无需 delete
    close_mpp_encoder();
    if (g_nv12_heap_fd >= 0) {
        close(g_nv12_heap_fd);
        g_nv12_heap_fd = -1;
    }
    g_camera.stopStreaming();
    g_camera.closeDevice();

    return 0;
}
