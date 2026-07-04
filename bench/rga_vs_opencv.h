#pragma once

/**
 * @file rga_vs_opencv.h
 * @brief RGA 硬件加速 vs OpenCV CPU 实现 性能对比测试
 *
 * 用法：
 *   ./app --bench-rga           → 采集一帧后跑全部对比，打印表格后退出
 *
 * 测试项：
 *   1. NV12→BGR 色彩转换 (1920×1080)
 *   2. NV12→RGB + 缩放至 640×640 (YOLO 预处理仿真)
 *   3. BGR→NV12 色彩转换 (1920×1080)
 *   4. BGRA→NV12 Alpha 叠加 (OSD 仿真)
 */

#include <chrono>
#include <cstdio>
#include <cstring>
#include <sys/mman.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>

#include <opencv2/opencv.hpp>

#include "RgaUtils.h"
#include "im2d.h"
#include "rga.h"

using namespace std::chrono;

// ── 辅助：计算中位数 ──────────────────────────────────────────────
inline double median(std::vector<double>& v) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

// ── 辅助：计时并返回 ms ───────────────────────────────────────────
inline double elapsed_ms(uint64_t t0) {
    uint64_t t1 = steady_clock::now().time_since_epoch().count();
    return (t1 - t0) / 1'000'000.0;
}

// ═══════════════════════════════════════════════════════════════════
// 测试入口：传入一个 NV12 dmabuf_fd (1920×1080)，迭代 iterations 次
// ═══════════════════════════════════════════════════════════════════
inline void run_rga_vs_opencv_benchmark(int dmabuf_fd, int w, int h, int iterations = 100) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║  RGA vs OpenCV Benchmark  (%dx%d NV12, %d iterations)         ║\n", w, h, iterations);
    printf("╠══════════════════════════════════════════════════════════════════╣\n");

    const size_t rgb_size   = (size_t)w * h * 3;
    const size_t nv12_size  = (size_t)w * h * 3 / 2;
    const size_t bgra_size  = (size_t)w * h * 4;
    const size_t small_size = 640 * 640 * 3;

    // ── 一次性分配所有测试用缓冲 ──────────────────────────────────
    uint8_t* bgr_buf     = new uint8_t[rgb_size];
    uint8_t* nv12_buf    = new uint8_t[nv12_size];
    uint8_t* bgra_buf    = new uint8_t[bgra_size];
    uint8_t* small_bgr   = new uint8_t[small_size];
    uint8_t* nv12_dst    = new uint8_t[nv12_size];

    // 填充 BGRA 测试图案（半透明红框，模拟 OSD 图层）
    cv::Mat bgra_test(h, w, CV_8UC4, bgra_buf);
    bgra_test.setTo(cv::Scalar(0, 0, 0, 0));
    cv::rectangle(bgra_test, cv::Point(100, 100), cv::Point(500, 500),
                  cv::Scalar(0, 0, 255, 128), -1);

    // mmap NV12 源帧（供 OpenCV 读取）
    void* nv12_ptr = mmap(nullptr, nv12_size, PROT_READ, MAP_SHARED, dmabuf_fd, 0);
    cv::Mat nv12_mat(h * 3 / 2, w, CV_8UC1, nv12_ptr);

    // ═══════════════════════════════════════════════════════════════
    // 测试 1：NV12 → BGR 色彩转换
    // ═══════════════════════════════════════════════════════════════
    {
        std::vector<double> rga_times, ocv_times;

        for (int i = 0; i < iterations; i++) {
            // ── RGA ──
            {
                rga_buffer_handle_t src_h = importbuffer_fd(dmabuf_fd, w, h, RK_FORMAT_YCbCr_420_SP);
                rga_buffer_handle_t dst_h = importbuffer_virtualaddr(bgr_buf, rgb_size);
                rga_buffer_t src = wrapbuffer_handle(src_h, w, h, RK_FORMAT_YCbCr_420_SP);
                rga_buffer_t dst = wrapbuffer_handle(dst_h, w, h, RK_FORMAT_BGR_888);

                auto t0 = steady_clock::now().time_since_epoch().count();
                imcvtcolor(src, dst, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_BGR_888);
                rga_times.push_back(elapsed_ms(t0));

                releasebuffer_handle(src_h);
                releasebuffer_handle(dst_h);
            }

            // ── OpenCV ──
            {
                cv::Mat bgr;
                auto t0 = steady_clock::now().time_since_epoch().count();
                cv::cvtColor(nv12_mat, bgr, cv::COLOR_YUV2BGR_NV12);
                ocv_times.push_back(elapsed_ms(t0));
            }
        }

        double rga_med = median(rga_times), ocv_med = median(ocv_times);
        printf("║  %-60s  ║\n", "1. NV12->BGR (1920x1080)");
        printf("║    RGA:    %8.2f ms  |  OpenCV: %8.2f ms  |  speedup: %5.1fx  ║\n",
               rga_med, ocv_med, ocv_med / rga_med);
    }

    // ═══════════════════════════════════════════════════════════════
    // 测试 2：NV12 → RGB + 缩放到 640×640（YOLO 预处理）
    // ═══════════════════════════════════════════════════════════════
    {
        std::vector<double> rga_times, ocv_times;

        for (int i = 0; i < iterations; i++) {
            // ── RGA: improcess 一步完成 ──
            {
                rga_buffer_handle_t src_h = importbuffer_fd(dmabuf_fd, w, h, RK_FORMAT_YCbCr_420_SP);
                rga_buffer_handle_t dst_h = importbuffer_virtualaddr(small_bgr, small_size);
                rga_buffer_t src = wrapbuffer_handle(src_h, w, h, RK_FORMAT_YCbCr_420_SP);
                rga_buffer_t dst = wrapbuffer_handle(dst_h, 640, 640, RK_FORMAT_RGB_888);
                im_rect srect = {0, 0, w, h};
                im_rect drect = {0, 0, 640, 640};
                im_rect prect = {0, 0, 0, 0};
                rga_buffer_t empty_pat;
                memset(&empty_pat, 0, sizeof(rga_buffer_t));

                auto t0 = steady_clock::now().time_since_epoch().count();
                improcess(src, dst, empty_pat, srect, drect, prect, IM_SYNC);
                rga_times.push_back(elapsed_ms(t0));

                releasebuffer_handle(src_h);
                releasebuffer_handle(dst_h);
            }

            // ── OpenCV: cvtColor + resize 两步 ──
            {
                cv::Mat bgr, small;
                auto t0 = steady_clock::now().time_since_epoch().count();
                cv::cvtColor(nv12_mat, bgr, cv::COLOR_YUV2BGR_NV12);
                cv::resize(bgr, small, cv::Size(640, 640));
                ocv_times.push_back(elapsed_ms(t0));
            }
        }

        double rga_med = median(rga_times), ocv_med = median(ocv_times);
        printf("║  %-60s  ║\n", "2. NV12->RGB + resize to 640x640 (YOLO preprocess)");
        printf("║    RGA:    %8.2f ms  |  OpenCV: %8.2f ms  |  speedup: %5.1fx  ║\n",
               rga_med, ocv_med, ocv_med / rga_med);
    }

    // ═══════════════════════════════════════════════════════════════
    // 测试 3：BGR → NV12 色彩转换
    // ═══════════════════════════════════════════════════════════════
    {
        std::vector<double> rga_times, ocv_times;

        // 先用 RGA 生成一份 BGR 源（填入 nv12_buf 转出来的 bgr_buf）
        {
            rga_buffer_handle_t src_h = importbuffer_fd(dmabuf_fd, w, h, RK_FORMAT_YCbCr_420_SP);
            rga_buffer_handle_t dst_h = importbuffer_virtualaddr(bgr_buf, rgb_size);
            rga_buffer_t src = wrapbuffer_handle(src_h, w, h, RK_FORMAT_YCbCr_420_SP);
            rga_buffer_t dst = wrapbuffer_handle(dst_h, w, h, RK_FORMAT_BGR_888);
            imcvtcolor(src, dst, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_BGR_888);
            releasebuffer_handle(src_h);
            releasebuffer_handle(dst_h);
        }
        cv::Mat bgr_src(h, w, CV_8UC3, bgr_buf);

        for (int i = 0; i < iterations; i++) {
            // ── RGA ──
            {
                rga_buffer_handle_t src_h = importbuffer_virtualaddr(bgr_buf, rgb_size);
                rga_buffer_handle_t dst_h = importbuffer_virtualaddr(nv12_dst, nv12_size);
                rga_buffer_t src = wrapbuffer_handle(src_h, w, h, RK_FORMAT_BGR_888);
                rga_buffer_t dst = wrapbuffer_handle(dst_h, w, h, RK_FORMAT_YCbCr_420_SP);

                auto t0 = steady_clock::now().time_since_epoch().count();
                imcvtcolor(src, dst, RK_FORMAT_BGR_888, RK_FORMAT_YCbCr_420_SP);
                rga_times.push_back(elapsed_ms(t0));

                releasebuffer_handle(src_h);
                releasebuffer_handle(dst_h);
            }

            // ── OpenCV ──
            {
                cv::Mat nv12_out;
                auto t0 = steady_clock::now().time_since_epoch().count();
                cv::cvtColor(bgr_src, nv12_out, cv::COLOR_BGR2YUV_I420);
                ocv_times.push_back(elapsed_ms(t0));
            }
        }

        double rga_med = median(rga_times), ocv_med = median(ocv_times);
        printf("║  %-60s  ║\n", "3. BGR->NV12 (1920x1080)");
        printf("║    RGA:    %8.2f ms  |  OpenCV: %8.2f ms  |  speedup: %5.1fx  ║\n",
               rga_med, ocv_med, ocv_med / rga_med);
    }

    // ═══════════════════════════════════════════════════════════════
    // 测试 4：BGRA → NV12 Alpha 叠加（OSD 仿真）
    // ═══════════════════════════════════════════════════════════════
    {
        std::vector<double> rga_times, ocv_times;

        // 准备底图 NV12（从相机帧拷贝一份）
        uint8_t* nv12_base = new uint8_t[nv12_size];
        {
            rga_buffer_handle_t src_h = importbuffer_fd(dmabuf_fd, w, h, RK_FORMAT_YCbCr_420_SP);
            rga_buffer_handle_t dst_h = importbuffer_virtualaddr(nv12_base, nv12_size);
            rga_buffer_t src = wrapbuffer_handle(src_h, w, h, RK_FORMAT_YCbCr_420_SP);
            rga_buffer_t dst = wrapbuffer_handle(dst_h, w, h, RK_FORMAT_YCbCr_420_SP);
            imcopy(src, dst);
            releasebuffer_handle(src_h);
            releasebuffer_handle(dst_h);
        }

        for (int i = 0; i < iterations; i++) {
            // ── RGA: imcopy + imblend ──
            {
                rga_buffer_handle_t src_h = importbuffer_virtualaddr(nv12_base, nv12_size);
                rga_buffer_handle_t osd_h = importbuffer_virtualaddr(bgra_buf, bgra_size);
                rga_buffer_handle_t dst_h = importbuffer_virtualaddr(nv12_dst, nv12_size);
                rga_buffer_t src = wrapbuffer_handle(src_h, w, h, RK_FORMAT_YCbCr_420_SP);
                rga_buffer_t osd = wrapbuffer_handle(osd_h, w, h, RK_FORMAT_BGRA_8888);
                rga_buffer_t dst = wrapbuffer_handle(dst_h, w, h, RK_FORMAT_YCbCr_420_SP);

                auto t0 = steady_clock::now().time_since_epoch().count();
                imcopy(src, dst);
                imblend(osd, dst, IM_ALPHA_BLEND_SRC_OVER);
                rga_times.push_back(elapsed_ms(t0));

                releasebuffer_handle(src_h);
                releasebuffer_handle(osd_h);
                releasebuffer_handle(dst_h);
            }

            // ── OpenCV: 手工 Alpha 混合（NV12→BGR→混合→NV12） ──
            {
                cv::Mat base_bgr;
                auto t0 = steady_clock::now().time_since_epoch().count();

                // NV12→BGR
                cv::Mat base_nv12_mat(h * 3 / 2, w, CV_8UC1, nv12_base);
                cv::cvtColor(base_nv12_mat, base_bgr, cv::COLOR_YUV2BGR_NV12);

                // Alpha 混合 BGRA → BGR
                for (int r = 0; r < h; r++) {
                    for (int c = 0; c < w; c++) {
                        uint8_t* bg = base_bgr.ptr<uint8_t>(r, c);
                        uint8_t* fg = bgra_buf + (r * w + c) * 4;
                        float alpha = fg[3] / 255.0f;
                        bg[0] = (uint8_t)(fg[0] * alpha + bg[0] * (1 - alpha));
                        bg[1] = (uint8_t)(fg[1] * alpha + bg[1] * (1 - alpha));
                        bg[2] = (uint8_t)(fg[2] * alpha + bg[2] * (1 - alpha));
                    }
                }

                ocv_times.push_back(elapsed_ms(t0));
            }
        }

        delete[] nv12_base;

        double rga_med = median(rga_times), ocv_med = median(ocv_times);
        printf("║  %-60s  ║\n", "4. BGRA->NV12 Alpha blend (OSD overlay)");
        printf("║    RGA:    %8.2f ms  |  OpenCV: %8.2f ms  |  speedup: %5.1fx  ║\n",
               rga_med, ocv_med, ocv_med / rga_med);
    }

    printf("╚══════════════════════════════════════════════════════════════════╝\n");
    printf("\n[Bench] RGA total: %d iterations per test, median values shown.\n\n", iterations);

    // ── 清理 ──────────────────────────────────────────────────────
    munmap(nv12_ptr, nv12_size);
    delete[] bgr_buf;
    delete[] nv12_buf;
    delete[] bgra_buf;
    delete[] small_bgr;
    delete[] nv12_dst;
}
