#ifndef _MPP_H
#define _MPP_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <rockchip/rk_mpi.h>

#ifdef __cplusplus
        extern "C"
        {
#endif

#define MPP_ALIGN(x, a)         (((x)+(a)-1)&~((a)-1))

typedef struct {
        uint8_t *data;
        uint32_t size;
} SpsHeader;

/**
 * @brief MPP编码器上下文结构体
 * 
 * 该结构体包含了MPP编码器运行所需的所有参数和状态信息
 */

struct MppContext;

typedef struct MppContext {
        // 基础MPP上下文
        MppCtx ctx;          ///< MPP上下文句柄
        MppApi *mpi;         ///< MPP API接口指针

        // 全局流程控制标志
        RK_U32 frm_eos;      ///< 帧结束标志
        RK_U32 pkt_eos;      ///< 包结束标志
        RK_U32 frm_pkt_cnt;  ///< 当前帧的包计数
        RK_S32 frame_num;    ///< 需要编码的总帧数
        RK_S32 frame_count;  ///< 已编码的帧数
        RK_U64 stream_size;  ///< 已编码的流大小

        // 编码器配置
        MppEncCfg cfg;       ///< 编码器配置

        // 输入/输出缓冲区
        MppBufferGroup buf_grp;  ///< 缓冲区组
        MppBuffer frm_buf;       ///< 编码输入帧缓冲区
        MppBuffer pkt_buf;       ///< 编码输出包缓冲区
        MppEncSeiMode sei_mode;  ///< SEI模式

        // 解码相关资源
        MppPacket dec_pkt;       ///< 解码输入包对象
        MppFrame dec_frame;      ///< 解码输出帧对象

        // 资源分配参数
        RK_U32 width;           ///< 图像宽度
        RK_U32 height;          ///< 图像高度
        RK_U32 hor_stride;      ///< 水平步长
        RK_U32 ver_stride;      ///< 垂直步长
        MppFrameFormat fmt;     ///< 帧格式
        MppCodingType type;     ///< 编码类型

        // 资源大小
        size_t frame_size;      ///< 帧大小

        // 码率控制参数
        RK_S32 fps_in_flex;     ///< 输入帧率灵活模式
        RK_S32 fps_in_den;      ///< 输入帧率分母
        RK_S32 fps_in_num;      ///< 输入帧率分子
        RK_S32 fps_out_flex;    ///< 输出帧率灵活模式
        RK_S32 fps_out_den;     ///< 输出帧率分母
        RK_S32 fps_out_num;     ///< 输出帧率分子
        RK_S32 bps;             ///< 目标码率
        RK_S32 bps_max;         ///< 最大码率
        RK_S32 bps_min;         ///< 最小码率
        RK_S32 rc_mode;         ///< 码率控制模式
        RK_S32 gop_len;         ///< GOP长度

        // 回调函数（编码）
        int (*write_frame)(void* packet, uint8_t* data, int size);
        int (*init_mpp)(struct MppContext *mpp_enc_data);
        bool (*encode_mpp_frame)(struct MppContext *mpp_enc_data);
        bool (*get_header)(struct MppContext *mpp_enc_data,SpsHeader *sps_header);
        void (*close)(struct MppContext *mpp_enc_data);

} MppContext;

MppContext* alloc_mpp_context();

#ifdef __cplusplus
        }
#endif

#endif /* !_MPP_H */