#pragma once

#include <cstdint>
#include <cstdio>
#include "RgaUtils.h"
#include "drmrga.h"
#include "im2d.h"
#include "rga.h"

// YUYV dmabuf fd → BGR 虚拟地址，使用 RGA 硬件转换
inline void YUYV_to_BGR_rga(int yuyv_fd, int w, int h, uint8_t *bgr)
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
inline void BGR_to_NV12_rga(uint8_t *bgr, int w, int h, uint8_t *nv12, int w_stride, int h_stride)
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
