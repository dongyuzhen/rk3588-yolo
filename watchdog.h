#pragma once

/**
 * @file watchdog.h
 * @brief 摄像头 / NPU 异常看门狗线程
 *
 * 职责：
 *  - 独立线程轮询全局故障标志
 *  - 摄像头故障：stopStreaming → closeDevice → re-init → startStreaming
 *  - NPU 故障：调用 ThreadPoll::reinit() 重载 rknn 模型
 *  - 最多重试 MAX_RETRY 次，超限后记录 FATAL 日志
 *
 * 与流水线线程的协议：
 *  - 采集/处理线程 设置 need_restart 标志后，进入自旋等待（spin on `recovering`）
 *  - 看门狗 检测到 need_restart → 设置 recovering → 执行恢复 → 清除 recovering
 *  - 采集/处理线程 检测到 recovering==false 后继续运行
 */

#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "camera/v4l2_camera.h"
#include "pool/thread_poll.h"

// ─── 全局故障信号（由采集/处理线程写，看门狗读/清） ──────────────────────────

// 摄像头
inline std::atomic<int>  g_cam_fail_count{0};   // 连续采集失败帧数
inline std::atomic<bool> g_cam_need_restart{false}; // 采集线程请求重启
inline std::atomic<bool> g_cam_recovering{false};   // 看门狗正在恢复中（采集线程自旋等待）

// NPU
inline std::atomic<int>  g_npu_fail_count{0};   // 连续推理失败帧数
inline std::atomic<bool> g_npu_need_restart{false}; // 处理线程请求重启
inline std::atomic<bool> g_npu_recovering{false};   // 看门狗正在恢复中

// 看门狗退出信号（由 main 在流水线结束后置 true）
inline std::atomic<bool> g_watchdog_stop{false};

// 全局 NPU 指针（main 中构造 ThreadPoll 后立即赋值）
inline ThreadPoll* g_npu_pool_ptr = nullptr;

// 摄像头设备路径（与 main 中 init 参数一致）
inline const char*  g_cam_dev     = "/dev/video-camera0";
inline int          g_cam_width   = 1920;
inline int          g_cam_height  = 1080;
inline int          g_cam_bufnum  = 12;

// ─── 故障日志追加写入 ─────────────────────────────────────────────────────────
inline void watchdog_log(const std::string& msg) {
    std::cerr << "[WATCHDOG] " << msg << std::endl;
    std::ofstream log("violations/watchdog_log.txt", std::ios::app);
    if (log.is_open()) log << msg << "\n";
}

// ─── 看门狗线程函数 ───────────────────────────────────────────────────────────
inline void watchdogThreadFunc(V4L2Camera& camera) {
    constexpr int MAX_RETRY     = 3;
    constexpr int POLL_INTERVAL = 200;  // ms

    int cam_retry_count = 0;
    int npu_retry_count = 0;

    while (!g_watchdog_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(POLL_INTERVAL));

        // ══════════════════════════════════════════════════════════════════════
        // 1. 摄像头恢复
        // ══════════════════════════════════════════════════════════════════════
        if (g_cam_need_restart.exchange(false)) {
            if (cam_retry_count >= MAX_RETRY) {
                watchdog_log("FATAL: camera recovery failed after " +
                             std::to_string(MAX_RETRY) + " retries. Giving up.");
                // 不再继续，采集线程会因 g_cam_recovering 永远不清除而阻塞
                // 实际产品中可触发进程退出或上报运维平台
                continue;
            }

            g_cam_recovering = true;
            watchdog_log("Camera fault detected (fail=" +
                         std::to_string(g_cam_fail_count.load()) +
                         "), retry " + std::to_string(cam_retry_count + 1) +
                         "/" + std::to_string(MAX_RETRY));
            g_cam_fail_count = 0;

            // ── 恢复流程 ──────────────────────────────────────────────────────
            camera.stopStreaming();
            camera.closeDevice();

            // 短暂等待设备重新上线
            // ISP 管线需要约 2s 预热，过短会导致恢复后立刻再次 Stall
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            bool ok = camera.init(g_cam_dev, g_cam_width, g_cam_height,
                                  g_cam_bufnum, V4L2_PIX_FMT_NV12);
            if (ok) ok = camera.startStreaming();

            if (ok) {
                cam_retry_count = 0;
                // 写入 -150：让采集线程有约 5s 的宽限期（@30fps）
                // 期间 ISP 管线预热产生的 POLLERR 不会重新触发 Stall
                g_cam_fail_count.store(-150, std::memory_order_relaxed);
                watchdog_log("Camera recovered successfully.");
            } else {
                cam_retry_count++;
                watchdog_log("Camera recovery attempt " +
                             std::to_string(cam_retry_count) + " failed.");
                // 如果还有重试机会，重新标记请求（下轮再试）
                if (cam_retry_count < MAX_RETRY)
                    g_cam_need_restart = true;
            }

            g_cam_recovering = false;  // 释放采集线程
        }

        // ══════════════════════════════════════════════════════════════════════
        // 2. NPU 恢复
        // ══════════════════════════════════════════════════════════════════════
        if (g_npu_need_restart.exchange(false)) {
            if (npu_retry_count >= MAX_RETRY) {
                watchdog_log("FATAL: NPU recovery failed after " +
                             std::to_string(MAX_RETRY) + " retries. Giving up.");
                continue;
            }

            g_npu_recovering = true;
            watchdog_log("NPU fault detected (fail=" +
                         std::to_string(g_npu_fail_count.load()) +
                         "), retry " + std::to_string(npu_retry_count + 1) +
                         "/" + std::to_string(MAX_RETRY));
            g_npu_fail_count = 0;

            // ── 恢复流程：销毁并重建 ThreadPoll ──────────────────────────────
            // 注意：此时处理线程处于自旋等待，不会再向 ThreadPoll 提交任务
            if (g_npu_pool_ptr) {
                // 析构旧的（内部 RKNN context 会被释放）
                delete g_npu_pool_ptr;
                g_npu_pool_ptr = nullptr;

                std::this_thread::sleep_for(std::chrono::milliseconds(300));

                try {
                    g_npu_pool_ptr = new ThreadPoll("../model/best.rknn", 3,
                                                    g_cam_width, g_cam_height);
                    npu_retry_count = 0;
                    watchdog_log("NPU recovered successfully (new ThreadPoll created).");
                } catch (const std::exception& e) {
                    npu_retry_count++;
                    watchdog_log("NPU recovery attempt " +
                                 std::to_string(npu_retry_count) +
                                 " failed: " + e.what());
                    if (npu_retry_count < MAX_RETRY)
                        g_npu_need_restart = true;
                }
            }

            g_npu_recovering = false;  // 释放处理线程
        }
    }

    watchdog_log("Watchdog thread exiting normally.");
}
