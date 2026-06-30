#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mpp.h"
#include "rtmp.h"

// 全局变量：控制是否启用RTMP推流
// 1=启用RTMP推流，0=禁用RTMP推流（仅保留编码功能）
int g_enable_rtmp = 1;  // 默认启用RTMP

typedef struct {
    MppContext *mpp_ctx;
    RtmpContext *rtmp_ctx;
    SpsHeader sps_header;
    int is_initialized;
    int enable_rtmp;  // 是否启用RTMP推流
} StreamerContext;

static StreamerContext g_streamer_ctx = {0};

int init_streamer(int width, int height, int fps, int bitrate, const char *rtmp_url, int enable_rtmp) {
    if (g_streamer_ctx.is_initialized) {
        return 0;
    }

    // 设置全局RTMP启用标志
    g_enable_rtmp = enable_rtmp;
    g_streamer_ctx.enable_rtmp = enable_rtmp;

    // 初始化MPP编码器
    g_streamer_ctx.mpp_ctx = alloc_mpp_context();
    if (!g_streamer_ctx.mpp_ctx) {
        printf("Failed to allocate MPP context\n");
        return -1;
    }

    // 配置MPP编码器参数
    g_streamer_ctx.mpp_ctx->width = width;
    g_streamer_ctx.mpp_ctx->height = height;

    g_streamer_ctx.mpp_ctx->fps_in_flex = 0;  // 使用固定帧率模式
    g_streamer_ctx.mpp_ctx->fps_in_num = fps;
    g_streamer_ctx.mpp_ctx->fps_in_den = 1;

    g_streamer_ctx.mpp_ctx->fps_in_flex = 0;  // 使用固定帧率模式
    g_streamer_ctx.mpp_ctx->fps_out_num = fps;
    g_streamer_ctx.mpp_ctx->fps_out_den = 1;
    
    g_streamer_ctx.mpp_ctx->bps = bitrate;
    g_streamer_ctx.mpp_ctx->gop_len = fps * 2;  // GOP长度为帧率的2倍
    g_streamer_ctx.mpp_ctx->write_frame = write_frame;
    g_streamer_ctx.mpp_ctx->type = MPP_VIDEO_CodingAVC;  // 设置H.264编码
    g_streamer_ctx.mpp_ctx->fmt = MPP_FMT_YUV420SP;  // 设置YUV420P格式
    g_streamer_ctx.mpp_ctx->rc_mode = MPP_ENC_RC_MODE_CBR;

    printf("初始化流媒体推送器...\n");
    printf("视频参数: %dx%d, %d fps, %d bps\n", width, height, fps, bitrate);
    if (enable_rtmp) {
        printf("RTMP地址: %s\n", rtmp_url);
    } else {
        printf("RTMP推流已禁用\n");
    }
    
    // 初始化MPP
    int ret = g_streamer_ctx.mpp_ctx->init_mpp(g_streamer_ctx.mpp_ctx);
    if(ret != 0)
    {
        printf("mpp init fail!\n");
        return -1;
    }
    else
    {
        printf("mpp init success!\n");
    }

    // 获取SPS/PPS信息
    if (!g_streamer_ctx.mpp_ctx->get_header(g_streamer_ctx.mpp_ctx, &g_streamer_ctx.sps_header)) {
        printf("Failed to get SPS/PPS header\n");
        return -1;
    }
    
    // 根据enable_rtmp决定是否初始化RTMP
    if (enable_rtmp) {
        g_streamer_ctx.rtmp_ctx = (RtmpContext *)malloc(sizeof(RtmpContext));

        g_streamer_ctx.rtmp_ctx->codec_id = AV_CODEC_ID_H264;
        g_streamer_ctx.rtmp_ctx->pix_fmt = AV_PIX_FMT_NV12;  // YUV420P格式
        g_streamer_ctx.rtmp_ctx->width = width;
        g_streamer_ctx.rtmp_ctx->height = height;
        g_streamer_ctx.rtmp_ctx->fps = fps;
        g_streamer_ctx.rtmp_ctx->max_b_frames = 0;  // 禁用B帧
        g_streamer_ctx.rtmp_ctx->profile = FF_PROFILE_H264_HIGH;  // 设置H.264 profile
        g_streamer_ctx.rtmp_ctx->level = 31;  // 设置H.264 level
        g_streamer_ctx.rtmp_ctx->extradata = g_streamer_ctx.sps_header.data;
        g_streamer_ctx.rtmp_ctx->extradata_size = g_streamer_ctx.sps_header.size;

        printf("初始化RTMP...\n");
        // 初始化RTMP
        if (init_rtmp_streamer((char*)rtmp_url, g_streamer_ctx.rtmp_ctx) < 0) {
            printf("Failed to initialize RTMP streamer\n");
            return -1;
        }
        printf("初始化RTMP成功\n");
    } else {
        g_streamer_ctx.rtmp_ctx = NULL;
        printf("RTMP初始化已跳过\n");
    }

    g_streamer_ctx.is_initialized = 1;
    return 0;
}

int process_frame(uint8_t *frame_data, int frame_size) {
    if (!g_streamer_ctx.is_initialized || !g_streamer_ctx.mpp_ctx) {
        return -1;
    }

    static int frame_log_counter = 0;
    if ((frame_log_counter++ % 30) == 0) {
        printf("输入帧信息: 大小=%d bytes, 格式=nv12\n", frame_size);
    }

    if (!g_streamer_ctx.mpp_ctx->process_image(frame_data, frame_size, g_streamer_ctx.mpp_ctx)) {
        printf("Failed to process frame\n");
        return -1;
    }

    return 0;
}

int process_frame_fd(int fd, int frame_size) {
    if (!g_streamer_ctx.is_initialized || !g_streamer_ctx.mpp_ctx) {
        return -1;
    }

    if (!process_image_fd(fd, frame_size, g_streamer_ctx.mpp_ctx)) {
        printf("Failed to process frame fd\n");
        return -1;
    }

    return 0;
}

void close_streamer() {
    if (g_streamer_ctx.mpp_ctx) {
        g_streamer_ctx.mpp_ctx->close(g_streamer_ctx.mpp_ctx);
        g_streamer_ctx.mpp_ctx = NULL;
    }
    
    // 释放 rttmp_ctx 内存
    if (g_streamer_ctx.rtmp_ctx) {
        // 重要：将 extradata 置为 NULL，因为它是 sps_header.data 的指针
        // RTMP 库会尝试释放 extradata，但这块内存应该由 sps_header.data 释放
        g_streamer_ctx.rtmp_ctx->extradata = NULL;
        g_streamer_ctx.rtmp_ctx->extradata_size = 0;
        free(g_streamer_ctx.rtmp_ctx);
        g_streamer_ctx.rtmp_ctx = NULL;
    }
    
    if (g_streamer_ctx.sps_header.data) {
        free(g_streamer_ctx.sps_header.data);
        g_streamer_ctx.sps_header.data = NULL;
    }
    
    g_streamer_ctx.is_initialized = 0;
    g_streamer_ctx.enable_rtmp = 0;
    g_enable_rtmp = 0;  // 重置全局标志
} 