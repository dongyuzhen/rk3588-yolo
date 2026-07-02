/**
 * @file mpp.c
 * @brief MPP（Media Process Platform）编码器实现
 * 
 * 本文件实现了基于Rockchip MPP的视频编码功能，支持H.264编码。
 * 主要功能包括：
 * 1. MPP编码器的初始化和配置
 * 2. 视频帧的编码处理
 * 3. 编码器头信息的获取
 * 4. 资源的分配和释放
 */
//============================================使用模板============================================
    // //分配
    // mpp_ctx = alloc_mpp_context();
    // // 配置MPP编码器参数
    // mpp_ctx->width = width;
    // mpp_ctx->height = height;

    // mpp_ctx->fps_in_flex = 0;  // 使用固定帧率模式
    // mpp_ctx->fps_in_num = fps;
    // mpp_ctx->fps_in_den = 1;

    // mpp_ctx->fps_in_flex = 0;  // 使用固定帧率模式
    // mpp_ctx->fps_out_num = fps;
    // mpp_ctx->fps_out_den = 1;
    
    // mpp_ctx->bps = bitrate;
    // mpp_ctx->gop_len = fps * 2;  // GOP长度为帧率的2倍
    // mpp_ctx->write_frame = write_frame;
    // mpp_ctx->type = MPP_VIDEO_CodingAVC;  // 设置H.264编码
    // mpp_ctx->fmt = MPP_FMT_YUV420SP;  // 设置YUV420P格式
    // mpp_ctx->rc_mode = MPP_ENC_RC_MODE_CBR;

    // // 初始化MPP
    // mpp_ctx->init_mpp(mpp_ctx);
    
//============================================使用模板============================================

#include "mpp.h"

// 声明内部函数
static void mpp_close(MppContext* ctx);
static int init_mpp(MppContext *mpp_enc_data);
static _Bool get_header(MppContext *mpp_enc_data, SpsHeader *sps_header);
_Bool process_image_fd(int fd, int size, MppContext *mpp_enc_data);



/**
 * @brief 分配并初始化MPP上下文
 * 
 * 创建MPP上下文结构体，并初始化所有回调函数指针
 * 
 * @return MppContext* 成功返回MPP上下文指针，失败返回NULL
 */
MppContext * alloc_mpp_context()
{
    // 必须清零，避免 bps_min / bps_max / fps_out_flex 等字段带随机值
    MppContext *ctx = (MppContext *)calloc(1, sizeof(MppContext));
    if (!ctx) {
        return NULL;
    }

    ctx->work_mode = 0;
    ctx->init_mpp = init_mpp;
    ctx->close = mpp_close;
    ctx->get_header = get_header;
    ctx->process_frame_fd = process_image_fd;

    return ctx;
}

/**
 * @brief 关闭并清理MPP编码器资源
 * 
 * 该函数负责清理所有MPP相关的资源，包括：
 * 1. 重置MPP上下文
 * 2. 销毁MPP上下文
 * 3. 释放帧缓冲区
 * 4. 释放MPP上下文结构体
 * 
 * @param ctx MPP上下文指针
 */
static void mpp_close(MppContext* ctx)
{
    MPP_RET ret = MPP_OK;

    if (!ctx) {
        return;
    }

    // 重置MPP上下文
    if (ctx->mpi && ctx->ctx) {
        ret = ctx->mpi->reset(ctx->ctx);
        if (ret)
        {
                printf("mpi->reset failed\n");
        }
    }

    // 释放解码资源
    if (ctx->dec_pkt) {
        mpp_packet_deinit(&ctx->dec_pkt);
        ctx->dec_pkt = NULL;
    }

    if (ctx->dec_frame) {
        mpp_frame_deinit(&ctx->dec_frame);
        ctx->dec_frame = NULL;
    }

    // 销毁MPP上下文
    if (ctx->ctx)
    {
            mpp_destroy(ctx->ctx);
            ctx->ctx = NULL;
    }

    // 释放包缓冲区
    if (ctx->pkt_buf)
    {
            mpp_buffer_put(ctx->pkt_buf);
            ctx->pkt_buf = NULL;
    }

    // 释放帧缓冲区
    if (ctx->frm_buf)
    {
            mpp_buffer_put(ctx->frm_buf);
            ctx->frm_buf = NULL;
    }

    // 释放缓冲区组
    if (ctx->buf_grp)
    {
            mpp_buffer_group_put(ctx->buf_grp);
            ctx->buf_grp = NULL;
    }

    // 释放编码器配置
    if (ctx->cfg)
    {
            mpp_enc_cfg_deinit(ctx->cfg);
            ctx->cfg = NULL;
    }

    // 释放MPP上下文结构体
    free(ctx);
}

/**
 * @brief 获取当前SoC类型
 * 
 * 返回当前使用的Rockchip SoC类型
 * 目前固定返回RK3588，可根据实际需求修改
 * 
 * @return RockchipSocType 返回SoC类型枚举值
 */
static int mpp_get_soc_type()
{
        return ROCKCHIP_SOC_RK3588;
}

/**
 * @brief 初始化MPP编码器
 * 
 * 该函数完成MPP编码器的完整初始化流程，包括：
 * 1. 设置基本编码参数（分辨率、格式等）
 * 2. 初始化缓冲区
 * 3. 创建MPP上下文
 * 4. 配置编码器参数
 * 5. 设置码率控制
 * 6. 配置H.264编码参数
 * 
 * @param mpp_enc_data MPP上下文指针
 * @return int 成功返回0，失败返回错误码
 */
static int init_mpp(MppContext *mpp_enc_data)
{
    MPP_RET ret = MPP_OK;
    MppPollType timeout = MPP_POLL_BLOCK;

    printf("start to init mpp...\n ");
    // 设置基本编码参数
    // 与主流程保持一致，统一按 16 对齐
    mpp_enc_data->hor_stride   = MPP_ALIGN(mpp_enc_data->width, 16);
    mpp_enc_data->ver_stride   = MPP_ALIGN(mpp_enc_data->height, 16);
    // 根据编码的输入格式计算视频帧的大小
    switch (mpp_enc_data->fmt & MPP_FRAME_FMT_MASK) {
    case MPP_FMT_YUV420SP:
    case MPP_FMT_YUV420P: {
        mpp_enc_data->frame_size = MPP_ALIGN(mpp_enc_data->hor_stride, 64) * MPP_ALIGN(mpp_enc_data->ver_stride, 64) * 3 / 2;
    } break;

    case MPP_FMT_YUV422_YUYV :
    case MPP_FMT_YUV422_YVYU :
    case MPP_FMT_YUV422_UYVY :
    case MPP_FMT_YUV422_VYUY :
    case MPP_FMT_YUV422P :
    case MPP_FMT_YUV422SP : {
        mpp_enc_data->frame_size = MPP_ALIGN(mpp_enc_data->hor_stride, 64) * MPP_ALIGN(mpp_enc_data->ver_stride, 64) * 2;
    } break;
    case MPP_FMT_YUV400:
    case MPP_FMT_RGB444 :
    case MPP_FMT_BGR444 :
    case MPP_FMT_RGB555 :
    case MPP_FMT_BGR555 :
    case MPP_FMT_RGB565 :
    case MPP_FMT_BGR565 :
    case MPP_FMT_RGB888 :
    case MPP_FMT_BGR888 :
    case MPP_FMT_RGB101010 :
    case MPP_FMT_BGR101010 :
    case MPP_FMT_ARGB8888 :
    case MPP_FMT_ABGR8888 :
    case MPP_FMT_BGRA8888 :
    case MPP_FMT_RGBA8888 : {
        mpp_enc_data->frame_size = MPP_ALIGN(mpp_enc_data->hor_stride, 64) * MPP_ALIGN(mpp_enc_data->ver_stride, 64);
    } break;

    default: {
        mpp_enc_data->frame_size = MPP_ALIGN(mpp_enc_data->hor_stride, 64) * MPP_ALIGN(mpp_enc_data->ver_stride, 64) * 4;
    } break;
    }

    // =================================初始化=================================
    // 初始化缓冲区组（使用DMA堆分配器，适用于实际硬件）
    ret = mpp_buffer_group_get_internal(&mpp_enc_data->buf_grp, MPP_BUFFER_TYPE_DMA_HEAP);
    if (ret) {
        printf("failed to get mpp buffer group ret %d\n", ret);
        goto MPP_INIT_OUT;
    }
    else
    {
        printf("get mpp buffer group\n");
    }

    // 分配输入帧缓冲区，这里由于并没有使用FPC，所以frame_size并不需要加上header_size
    printf("mpp: about to get frm_buf, buf_grp=%p, frame_size=%zu\n", mpp_enc_data->buf_grp, mpp_enc_data->frame_size);
    fflush(stdout);
    ret = mpp_buffer_get(mpp_enc_data->buf_grp, &mpp_enc_data->frm_buf, mpp_enc_data->frame_size);
    if (ret) {
        printf("failed to get buffer for input frame ret %d\n", ret);
        goto MPP_INIT_OUT;
    }
    printf("mpp: got frm_buf %p\n", mpp_enc_data->frm_buf);

    // 分配输出包缓冲区
    printf("mpp: about to get pkt_buf, buf_grp=%p, frame_size=%zu\n", mpp_enc_data->buf_grp, mpp_enc_data->frame_size);
    fflush(stdout);
    ret = mpp_buffer_get(mpp_enc_data->buf_grp, &mpp_enc_data->pkt_buf, mpp_enc_data->frame_size);
    if (ret) {
        printf("failed to get buffer for output packet ret %d\n", ret);
        goto MPP_INIT_OUT;
    }
    printf("mpp: got pkt_buf %p\n", mpp_enc_data->pkt_buf);

    // =================================编码=================================
    // 创建MPP上下文
    ret = mpp_create(&mpp_enc_data->ctx, &mpp_enc_data->mpi);
    if (ret) {
        printf("mpp_create failed ret %d\n", ret);
        goto MPP_INIT_OUT;
    }
    printf("mpp: created ctx %p mpi %p\n", mpp_enc_data->ctx, mpp_enc_data->mpi);

    // 设置输出超时
    ret = mpp_enc_data->mpi->control(mpp_enc_data->ctx, MPP_SET_OUTPUT_TIMEOUT, &timeout);
    if (ret) {
        printf("mpi control set output timeout failed ret %d\n", ret);
        goto MPP_INIT_OUT;
    }
    printf("mpp: set output timeout ok\n");

    // 初始化编码器
    ret = mpp_init(mpp_enc_data->ctx, MPP_CTX_ENC, mpp_enc_data->type);
    if (ret) {
        printf("mpp_init failed ret %d\n", ret);
        goto MPP_INIT_OUT;
    }
    printf("mpp: mpp_init ok\n");

    // 初始化编码器配置
    ret = mpp_enc_cfg_init(&mpp_enc_data->cfg);
    if (ret) {
        printf("mpp_enc_cfg_init failed ret %d\n", ret);
        goto MPP_INIT_OUT;
    }

    // 获取默认配置
    ret = mpp_enc_data->mpi->control(mpp_enc_data->ctx, MPP_ENC_GET_CFG, mpp_enc_data->cfg);
    if (ret) {
        printf("get enc cfg failed ret %d\n", ret);
        goto MPP_INIT_OUT;
    }

    // 设置编码器基本参数，这些参数基本都是外部配置的mpp参数以及计算出来的值
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "prep:width", mpp_enc_data->width);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "prep:height", mpp_enc_data->height);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "prep:hor_stride", mpp_enc_data->hor_stride);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "prep:ver_stride", mpp_enc_data->ver_stride);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "prep:format", mpp_enc_data->fmt);

    // 设置码率控制参数
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "rc:mode", mpp_enc_data->rc_mode);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "rc:bps_target", mpp_enc_data->bps);
    mpp_enc_cfg_set_u32(mpp_enc_data->cfg, "rc:max_reenc_times", 0);
    mpp_enc_cfg_set_u32(mpp_enc_data->cfg, "rc:super_mode", 0);
    // 设置CBR模式下的码率范围
    mpp_enc_data->bps_max = mpp_enc_data->bps_max ? mpp_enc_data->bps_max : mpp_enc_data->bps * 17 / 16;
    mpp_enc_data->bps_min = mpp_enc_data->bps_min ? mpp_enc_data->bps_min : mpp_enc_data->bps * 15 / 16;
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "rc:bps_max", mpp_enc_data->bps_max);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "rc:bps_min", mpp_enc_data->bps_min);

    // 设置帧率参数
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "rc:fps_in_flex", mpp_enc_data->fps_in_flex);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "rc:fps_in_num", mpp_enc_data->fps_in_num);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "rc:fps_in_denom", mpp_enc_data->fps_in_den);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "rc:fps_out_flex", mpp_enc_data->fps_out_flex);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "rc:fps_out_num", mpp_enc_data->fps_out_num);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "rc:fps_out_denom", mpp_enc_data->fps_out_den);

    // 设置GOP参数
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "rc:gop", mpp_enc_data->gop_len);

    // 设置编码器类型和H.264参数
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "codec:type", mpp_enc_data->type);
    RK_U32 constraint_set;
    /*
        * H.264 profile_idc parameter
        * 66  - Baseline profile
        * 77  - Main profile
        * 100 - High profile
        */
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "h264:profile", 100); // High profile
    /*
        * H.264 level_idc parameter
        * 10 / 11 / 12 / 13    - qcif@15fps / cif@7.5fps / cif@15fps / cif@30fps
        * 20 / 21 / 22         - cif@30fps / half-D1@@25fps / D1@12.5fps
        * 30 / 31 / 32         - D1@25fps / 720p@30fps / 720p@60fps
        * 40 / 41 / 42         - 1080p@30fps / 1080p@30fps / 1080p@60fps
        * 50 / 51 / 52         - 4K@30fps
        */
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "h264:level", 31);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "h264:cabac_en", 1);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "h264:cabac_idc", 0);
    mpp_enc_cfg_set_s32(mpp_enc_data->cfg, "h264:trans8x8", 1);

    
    // 应用配置
    ret = mpp_enc_data->mpi->control(mpp_enc_data->ctx, MPP_ENC_SET_CFG, mpp_enc_data->cfg);
    if (ret) {
        printf("mpi control enc set cfg failed ret %d\n", ret);
        goto MPP_INIT_OUT;
    }

    // 设置SEI模式
    mpp_enc_data->sei_mode = MPP_ENC_SEI_MODE_ONE_FRAME;
    ret = mpp_enc_data->mpi->control(mpp_enc_data->ctx, MPP_ENC_SET_SEI_CFG, &mpp_enc_data->sei_mode);
    if (ret) {
        printf("mpi control enc set sei cfg failed ret %d\n", ret);
        goto MPP_INIT_OUT;
    }

    // 打印编码器配置信息
    printf("\n========== MPP编码器配置参数 ==========\n");
    printf("基础参数:\n");
    printf("  分辨率: %dx%d\n", mpp_enc_data->width, mpp_enc_data->height);
    printf("  像素格式: %s\n", mpp_enc_data->fmt == MPP_FMT_YUV420SP ? "YUV420SP" : 
                              mpp_enc_data->fmt == MPP_FMT_YUV420P ? "YUV420P" :
                              mpp_enc_data->fmt == MPP_FMT_BGR888 ? "BGR888" : "Unknown");
    printf("  编码类型: %s\n", mpp_enc_data->type == MPP_VIDEO_CodingAVC ? "H.264" : 
                              mpp_enc_data->type == MPP_VIDEO_CodingHEVC ? "H.265" : "Unknown");

    printf("\n帧率配置:\n");
    printf("  输入帧率: %d/%d (%s模式)\n", 
           mpp_enc_data->fps_in_num, 
           mpp_enc_data->fps_in_den,
           mpp_enc_data->fps_in_flex ? "灵活" : "固定");
    printf("  输出帧率: %d/%d (%s模式)\n", 
           mpp_enc_data->fps_out_num, 
           mpp_enc_data->fps_out_den,
           mpp_enc_data->fps_out_flex ? "灵活" : "固定");

    printf("\n码率控制:\n");
    printf("  控制模式: %s\n", mpp_enc_data->rc_mode == MPP_ENC_RC_MODE_CBR ? "CBR(固定码率)" : 
                              mpp_enc_data->rc_mode == MPP_ENC_RC_MODE_VBR ? "VBR(可变码率)" : "Unknown");
    printf("  目标码率: %.2f Mbps\n", mpp_enc_data->bps / (1024.0 * 1024.0));
    printf("  最大码率: %.2f Mbps\n", mpp_enc_data->bps_max / (1024.0 * 1024.0));
    printf("  最小码率: %.2f Mbps\n", mpp_enc_data->bps_min / (1024.0 * 1024.0));

    printf("\n编码参数:\n");
    printf("  GOP长度: %d\n", mpp_enc_data->gop_len);
    if (mpp_enc_data->type == MPP_VIDEO_CodingAVC) {
        printf("  H.264 Profile: %s\n", 
               mpp_enc_data->cfg ? "High" : "Unknown");
        printf("  H.264 Level: 31 (720p@30fps)\n");
    }

    printf("\n缓冲区配置:\n");
    printf("  帧缓冲区大小: %zu bytes\n", mpp_enc_data->frame_size);
    printf("  头信息大小: %zu bytes\n", mpp_enc_data->header_size);

    printf("=======================================\n\n");

    return 0;

MPP_INIT_OUT:
    // 错误处理：清理资源（按相反顺序释放）
    if (mpp_enc_data->ctx) {
        mpp_destroy(mpp_enc_data->ctx);
        mpp_enc_data->ctx = NULL;
    }

    if (mpp_enc_data->pkt_buf) {
        mpp_buffer_put(mpp_enc_data->pkt_buf);
        mpp_enc_data->pkt_buf = NULL;
    }

    if (mpp_enc_data->frm_buf) {
        mpp_buffer_put(mpp_enc_data->frm_buf);
        mpp_enc_data->frm_buf = NULL;
    }

    if (mpp_enc_data->buf_grp) {
        mpp_buffer_group_put(mpp_enc_data->buf_grp);
        mpp_enc_data->buf_grp = NULL;
    }

    if (mpp_enc_data->cfg) {
        mpp_enc_cfg_deinit(mpp_enc_data->cfg);
        mpp_enc_data->cfg = NULL;
    }

    printf("init mpp failed!\n");
    return ret;
}

/**
 * @brief 获取编码器头信息
 * 
 * 该函数用于获取H.264/HEVC编码器的头信息（SPS/PPS等）
 * 
 * @param mpp_enc_data MPP上下文指针
 * @param sps_header 用于存储头信息的结构体指针
 * @return _Bool 成功返回true，失败返回false
 */
static _Bool get_header(MppContext *mpp_enc_data, SpsHeader *sps_header)
{
    MPP_RET ret = MPP_OK;
    MppPacket packet = NULL;

    printf("开始获取编码器header信息...\n");
    if (mpp_enc_data->type == MPP_VIDEO_CodingAVC || mpp_enc_data->type == MPP_VIDEO_CodingHEVC) {
        if (!mpp_enc_data->pkt_buf) {
            printf("error: pkt_buf is NULL, cannot get header\n");
            return 0;
        }
        
        // 初始化数据包
        mpp_packet_init_with_buffer(&packet, mpp_enc_data->pkt_buf);
        mpp_packet_set_length(packet, 0);

        // 获取编码器头信息
        ret = mpp_enc_data->mpi->control(mpp_enc_data->ctx, MPP_ENC_GET_HDR_SYNC, packet);
        if (ret) {
            printf("mpi control enc get extra info failed\n");
            return 1;
        }

        // 保存头信息
        if (packet) {
            void *ptr = mpp_packet_get_pos(packet);
            size_t len = mpp_packet_get_length(packet);
            
            if (sps_header) {
                sps_header->data = (uint8_t*)malloc(len);
                if (!sps_header->data) {
                    printf("failed to allocate memory for sps header\n");
                    mpp_packet_deinit(&packet);
                    return 1;
                }
                sps_header->size = len;
                memcpy(sps_header->data, ptr, len);
            }
        }

        mpp_packet_deinit(&packet);
        printf("开始获取编码器header信息成功\n");
    }

    return 1;
}

/**
 * @brief 零拷贝编码：直接从外部 dmabuf fd 导入输入帧
 *
 * 与 process_image() 的区别：不做 memcpy，
 * 而是通过 mpp_buffer_import_with_tag 把外部 CMA fd 包装成
 * MppBuffer，让编码器直接读取该内存，实现跨驱动零拷贝。
 *
 * @param fd            NV12 数据的 dmabuf fd
 * @param size          帧字节数（hor_stride * ver_stride * 3/2）
 * @param mpp_enc_data  MPP 上下文
 * @return _Bool        成功返回 true，失败返回 false
 */
_Bool process_image_fd(int fd, int size, MppContext *mpp_enc_data)
{
    MPP_RET ret = MPP_OK;
    MppFrame  frame  = NULL;
    MppPacket packet = NULL;
    MppMeta   meta   = NULL;
    MppBuffer frm_buf_ext = NULL;
    MppBufferGroup ext_grp = NULL;
    RK_U32 eoi = 1;
    static int frame_log_counter = 0;

    if (!mpp_enc_data || !mpp_enc_data->pkt_buf) {
        printf("[mpp] pkt_buf is NULL\n");
        return 0;
    }

    // 1. 创建 EXT_DMA 类型的 external group，专用于外部 fd import
    ret = mpp_buffer_group_get_external(&ext_grp, MPP_BUFFER_TYPE_EXT_DMA);
    if (ret || !ext_grp) {
        printf("[mpp] mpp_buffer_group_get_external failed, ret=%d\n", ret);
        return 0;
    }

    // 2. 把外部 dmabuf fd 导入为 MppBuffer（不拷贝数据）
    MppBufferInfo buf_info;
    memset(&buf_info, 0, sizeof(buf_info));
    buf_info.type  = MPP_BUFFER_TYPE_EXT_DMA;
    buf_info.fd    = fd;
    buf_info.size  = (size_t)size;
    buf_info.ptr   = NULL;

    ret = mpp_buffer_import_with_tag(ext_grp, &buf_info, &frm_buf_ext,
                                     "nv12_ext_fd", __FUNCTION__);
    if (ret || !frm_buf_ext) {
        printf("[mpp] mpp_buffer_import_with_tag failed, ret=%d\n", ret);
        mpp_buffer_group_put(ext_grp);
        return 0;
    }

    // 2. 初始化并设置编码帧，直接绑定外部 buffer
    ret = mpp_frame_init(&frame);
    if (ret) {
        printf("[mpp] mpp_frame_init failed\n");
        mpp_buffer_put(frm_buf_ext);
        mpp_buffer_group_put(ext_grp);
        return 0;
    }

    mpp_frame_set_width(frame,      mpp_enc_data->width);
    mpp_frame_set_height(frame,     mpp_enc_data->height);
    mpp_frame_set_hor_stride(frame, mpp_enc_data->hor_stride);
    mpp_frame_set_ver_stride(frame, mpp_enc_data->ver_stride);
    mpp_frame_set_fmt(frame,        mpp_enc_data->fmt);
    mpp_frame_set_buffer(frame,     frm_buf_ext);   // ← 直接用外部 fd 的 buffer
    mpp_frame_set_eos(frame,        mpp_enc_data->frm_eos);

    // 3. 绑定输出 packet
    meta = mpp_frame_get_meta(frame);
    mpp_packet_init_with_buffer(&packet, mpp_enc_data->pkt_buf);
    mpp_packet_set_length(packet, 0);
    mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, packet);

    // 4. 送帧编码
    ret = mpp_enc_data->mpi->encode_put_frame(mpp_enc_data->ctx, frame);
    if (ret) {
        printf("[mpp] encode_put_frame failed, ret=%d\n", ret);
        mpp_frame_deinit(&frame);
        mpp_buffer_put(frm_buf_ext);
        mpp_buffer_group_put(ext_grp);
        return 0;
    }
    mpp_frame_deinit(&frame);
    // 编码器持有引用，这里减一；编码器完成后会自动释放
    mpp_buffer_put(frm_buf_ext);

    // 5. 取编码结果
    do {
        ret = mpp_enc_data->mpi->encode_get_packet(mpp_enc_data->ctx, &packet);
        if (ret) {
            printf("[mpp] encode_get_packet failed, ret=%d\n", ret);
            mpp_buffer_group_put(ext_grp);
            return 0;
        }

        if (packet) {
            void   *ptr = mpp_packet_get_pos(packet);
            size_t  len = mpp_packet_get_length(packet);

            mpp_enc_data->pkt_eos = mpp_packet_get_eos(packet);

            if (mpp_enc_data->write_frame)
                mpp_enc_data->write_frame(packet, ptr, (int)len);

            if (mpp_packet_is_partition(packet)) {
                eoi = mpp_packet_is_eoi(packet);
                mpp_enc_data->frm_pkt_cnt = eoi ? 0 : mpp_enc_data->frm_pkt_cnt + 1;
            }

            if ((frame_log_counter++ % 30) == 0)
                printf("[mpp] fd encode frame %-4d size %-7zu\n",
                       mpp_enc_data->frame_count, len);

            mpp_packet_deinit(&packet);
            mpp_enc_data->stream_size += len;
            mpp_enc_data->frame_count += eoi;
        }
    } while (!eoi);

    mpp_buffer_group_put(ext_grp);

    if (mpp_enc_data->frame_num > 0 &&
        mpp_enc_data->frame_count >= mpp_enc_data->frame_num)
        return 0;

    if (mpp_enc_data->frm_eos && mpp_enc_data->pkt_eos)
        return 0;

    return 1;
}

