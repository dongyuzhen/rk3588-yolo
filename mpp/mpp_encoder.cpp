#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mpp.h"
#include "mpp_encoder.h"

struct MppEncoderContext {
    MppContext *mpp_ctx;
    SpsHeader sps_header;
    int is_initialized;
};

static MppEncoderContext g_mpp_enc_ctx = {0};

int init_mpp_encoder(int width, int height, int fps, int bitrate, int (*callback)(void*, uint8_t*, int)) {
    if (g_mpp_enc_ctx.is_initialized) return 0;

    g_mpp_enc_ctx.mpp_ctx = alloc_mpp_context();
    if (!g_mpp_enc_ctx.mpp_ctx) return -1;

    g_mpp_enc_ctx.mpp_ctx->width = width;
    g_mpp_enc_ctx.mpp_ctx->height = height;
    g_mpp_enc_ctx.mpp_ctx->fps_in_flex = 0;
    g_mpp_enc_ctx.mpp_ctx->fps_in_num = fps;
    g_mpp_enc_ctx.mpp_ctx->fps_in_den = 1;
    g_mpp_enc_ctx.mpp_ctx->fps_out_num = fps;
    g_mpp_enc_ctx.mpp_ctx->fps_out_den = 1;
    
    g_mpp_enc_ctx.mpp_ctx->bps = bitrate;
    g_mpp_enc_ctx.mpp_ctx->gop_len = fps * 2;
    g_mpp_enc_ctx.mpp_ctx->write_frame = callback; // 移交控制权给上层队列回调
    g_mpp_enc_ctx.mpp_ctx->type = MPP_VIDEO_CodingAVC;
    g_mpp_enc_ctx.mpp_ctx->fmt = MPP_FMT_YUV420SP;
    g_mpp_enc_ctx.mpp_ctx->rc_mode = MPP_ENC_RC_MODE_CBR;

    if(g_mpp_enc_ctx.mpp_ctx->init_mpp(g_mpp_enc_ctx.mpp_ctx) != 0) return -1;

    // 获取SPS/PPS信息，这是 RTSP 推流所必须的 extradata！
    if (!g_mpp_enc_ctx.mpp_ctx->get_header(g_mpp_enc_ctx.mpp_ctx, &g_mpp_enc_ctx.sps_header)) return -1;

    g_mpp_enc_ctx.is_initialized = 1;
    return 0;
}

int process_frame_fd(int fd, int frame_size) {
    if (!g_mpp_enc_ctx.is_initialized || !g_mpp_enc_ctx.mpp_ctx) return -1;
    if (!g_mpp_enc_ctx.mpp_ctx->process_frame_fd(fd, frame_size, g_mpp_enc_ctx.mpp_ctx)) return -1;
    return 0;
}

void get_mpp_sps_pps(uint8_t** data, int* size) {
    if (g_mpp_enc_ctx.is_initialized) {
        *data = g_mpp_enc_ctx.sps_header.data;
        *size = g_mpp_enc_ctx.sps_header.size;
    }
}

void close_mpp_encoder() {
    if (g_mpp_enc_ctx.mpp_ctx) {
        g_mpp_enc_ctx.mpp_ctx->close(g_mpp_enc_ctx.mpp_ctx);
        g_mpp_enc_ctx.mpp_ctx = nullptr;
    }
    if (g_mpp_enc_ctx.sps_header.data) {
        free(g_mpp_enc_ctx.sps_header.data);
        g_mpp_enc_ctx.sps_header.data = nullptr;
    }
    g_mpp_enc_ctx.is_initialized = 0;
}