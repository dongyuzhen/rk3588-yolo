#pragma once

#include <cstdint>
#include <cstdio>
#include "RgaUtils.h"
#include "drmrga.h"
#include "im2d.h"
#include "rga.h"

// NV12 fd → BGR 虚拟地址，使用 RGA 硬件转换 (用于报警截图 + 本地保存)
inline void NV12_to_BGR_rga(int nv12_fd, int w, int h, uint8_t *bgr)
{
    const size_t bgr_size = static_cast<size_t>(w) * h * 3;
    rga_buffer_handle_t nv12_h = importbuffer_fd(nv12_fd, w, h, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_handle_t bgr_h  = importbuffer_virtualaddr(bgr, bgr_size);
    if (nv12_h == 0 || bgr_h == 0) {
        std::printf("[Write] import nv12/bgr failed\n");
        if (nv12_h) releasebuffer_handle(nv12_h);
        if (bgr_h)  releasebuffer_handle(bgr_h);
        return;
    }
    rga_buffer_t src = wrapbuffer_handle(nv12_h, w, h, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t dst = wrapbuffer_handle(bgr_h,  w, h, RK_FORMAT_BGR_888);
    int ret = imcvtcolor(src, dst, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_BGR_888);
    if (ret != IM_STATUS_SUCCESS)
        std::printf("[Write] RGA NV12->BGR failed: %s\n", imStrError((IM_STATUS)ret));
    releasebuffer_handle(nv12_h);
    releasebuffer_handle(bgr_h);
}

// ── OSD 叠加 (缓存版)：osd/dst handle 提前 import，热路径只 import 变化的 src_fd ──
// 每帧节省 2 次 importbuffer_fd 内核调用 (~0.3ms)
inline void copy_and_blend_osd_rga_cached(int src_fd,
                                          rga_buffer_handle_t osd_h,
                                          rga_buffer_handle_t dst_h,
                                          int w, int h, int w_stride, int h_stride)
{
    rga_buffer_handle_t src_h = importbuffer_fd(src_fd, w, h, RK_FORMAT_YCbCr_420_SP);
    if (src_h == 0) {
        std::printf("[Write] import src_fd for blend failed\n");
        return;
    }

    rga_buffer_t src = wrapbuffer_handle(src_h, w, h, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t osd = wrapbuffer_handle(osd_h, w, h, RK_FORMAT_BGRA_8888);
    rga_buffer_t dst = wrapbuffer_handle(dst_h, w, h, RK_FORMAT_YCbCr_420_SP, w_stride, h_stride);

    im_rect srect = {0, 0, w, h};
    im_rect drect = {0, 0, w, h};
    im_rect prect = {0, 0, 0, 0};
    IM_STATUS ret = improcess(src, dst, osd, srect, drect, prect, IM_SYNC);
    if (ret != IM_STATUS_SUCCESS) {
        std::printf("[Write] RGA improcess blend failed: %s\n", imStrError((IM_STATUS)ret));
    }

    releasebuffer_handle(src_h);
}
