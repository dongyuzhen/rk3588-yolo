#ifndef _MPP_H
#define _MPP_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>
#include <rockchip/rk_mpi.h>

#ifdef __cplusplus
        extern "C"
        {
#endif

#define MPP_ALIGN(x, a)         (((x)+(a)-1)&~((a)-1))

//rk系列芯片枚举
typedef enum RockchipSocType_e {
    ROCKCHIP_SOC_AUTO,
    ROCKCHIP_SOC_RK3036,
    ROCKCHIP_SOC_RK3066,
    ROCKCHIP_SOC_RK3188,
    ROCKCHIP_SOC_RK3288,
    ROCKCHIP_SOC_RK312X,
    ROCKCHIP_SOC_RK3368,
    ROCKCHIP_SOC_RK3399,
    ROCKCHIP_SOC_RK3228H,
    ROCKCHIP_SOC_RK3328,
    ROCKCHIP_SOC_RK3228,
    ROCKCHIP_SOC_RK3229,
    ROCKCHIP_SOC_RV1108,
    ROCKCHIP_SOC_RV1109,
    ROCKCHIP_SOC_RV1126,
    ROCKCHIP_SOC_RK3326,
    ROCKCHIP_SOC_RK3128H,
    ROCKCHIP_SOC_PX30,
    ROCKCHIP_SOC_RK1808,
    ROCKCHIP_SOC_RK3566,
    ROCKCHIP_SOC_RK3567,
    ROCKCHIP_SOC_RK3568,
    ROCKCHIP_SOC_RK3588,
    ROCKCHIP_SOC_RK3528,
    ROCKCHIP_SOC_RK3562,
    ROCKCHIP_SOC_RK3576,
    ROCKCHIP_SOC_RV1126B,
    ROCKCHIP_SOC_BUTT,
} RockchipSocType;

typedef struct {
        uint8_t *data;
        uint32_t size;
} SpsHeader;

/**
 * @brief MPP编码器上下文结构体
 * 
 * 该结构体包含了MPP编码器运行所需的所有参数和状态信息
 */
typedef struct MppContext {
        // 基础MPP上下文
        MppCtx ctx;          ///< MPP上下文句柄
        MppApi *mpi;         ///< MPP API接口指针
        RK_S32 chn;          ///< 通道号
        RK_U32 work_mode;    ///< 工作模式: 0-编码, 1-解码

        // 全局流程控制标志
        RK_U32 frm_eos;      ///< 帧结束标志
        RK_U32 pkt_eos;      ///< 包结束标志
        RK_U32 frm_pkt_cnt;  ///< 当前帧的包计数
        RK_S32 frame_num;    ///< 需要编码的总帧数
        RK_S32 frame_count;  ///< 已编码的帧数
        RK_U64 stream_size;  ///< 已编码的流大小
        volatile RK_U32 loop_end;  ///< 循环结束标志

        // 编码器配置
        MppEncCfg cfg;       ///< 编码器配置
        MppEncPrepCfg   prep_cfg;
        MppEncRcCfg     rc_cfg;
        MppEncCodecCfg  codec_cfg;
        MppEncSliceSplit split_cfg;
        MppEncOSDPltCfg osd_plt_cfg;
        MppEncOSDPlt    osd_plt;
        MppEncOSDData   osd_data;
        MppEncROICfg    roi_cfg;

        // 输入/输出缓冲区
        MppBufferGroup buf_grp;  ///< 缓冲区组
        MppBuffer frm_buf;       ///< 编码输入帧缓冲区
        MppBuffer pkt_buf;       ///< 编码输出包缓冲区
        MppBuffer md_info;
        MppEncSeiMode sei_mode;  ///< SEI模式
        MppEncHeaderMode header_mode;

        // 解码相关资源
        MppPacket dec_pkt;       ///< 解码输入包对象
        MppFrame dec_frame;      ///< 解码输出帧对象
        RK_U32 dec_pkt_eos;      ///< 解码包结束标志
        RK_U32 dec_frm_eos;      ///< 解码帧结束标志
        RK_U32 dec_pkt_done;     ///< 解码输入包是否处理完成

        // 资源分配参数
        RK_U32 width;           ///< 图像宽度
        RK_U32 height;          ///< 图像高度
        RK_U32 hor_stride;      ///< 水平步长
        RK_U32 ver_stride;      ///< 垂直步长
        MppFrameFormat fmt;     ///< 帧格式
        MppCodingType type;     ///< 编码类型
        RK_S32 loop_times;

        // 资源大小
        size_t header_size;     ///< 头信息大小
        size_t frame_size;      ///< 帧大小
        size_t mdinfo_size;
        size_t packet_size;     ///< 包大小

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
        RK_S32 gop_mode;
        RK_S32 gop_len;         ///< GOP长度
        RK_S32 vi_len;
        RK_S32 scene_mode;
        RK_S32 cu_qp_delta_depth;
        RK_S32 anti_flicker_str;
        RK_S32 atr_str_i;
        RK_S32 atr_str_p;
        RK_S32 atl_str;
        RK_S32 sao_str_i;
        RK_S32 sao_str_p;
        RK_S64 first_frm;       ///< 第一帧时间戳
        RK_S64 first_pkt;       ///< 第一个包时间戳

        // 回调函数（编码）
        int (*write_frame)(uint8_t*data,int size);  ///< 写入编码后帧数据的回调函数
        int (*init_mpp)(struct MppContext *mpp_enc_data);        ///< 初始化MPP编码器的回调函数
        _Bool (*process_image)(uint8_t *p, int size, struct MppContext *mpp_enc_data);  ///< 处理图像编码的回调函数
        _Bool (*get_header)(struct MppContext *mpp_enc_data,SpsHeader *sps_header);  ///< 获取编码头信息的回调函数

        // 回调函数（解码）
        int (*init_decoder)(struct MppContext *mpp_dec_data);    ///< 初始化MPP解码器的回调函数
        _Bool (*process_packet)(uint8_t *p, int size, struct MppContext *mpp_dec_data); ///< 处理码流解码的回调函数
        int (*read_frame)(uint8_t *data, int size, int width, int height, MppFrameFormat fmt); ///< 输出解码后帧数据的回调函数

        void (*close)(struct MppContext* ctx);                   ///< 关闭MPP的回调函数

} MppContext;

MppContext* alloc_mpp_context();

// 零拷贝编码：直接从外部 dmabuf fd 导入，不做 memcpy
// fd   : NV12 数据的 dmabuf fd（来自 CmaBuffer）
// size : 帧大小（hor_stride * ver_stride * 3/2）
_Bool process_image_fd(int fd, int size, MppContext *mpp_enc_data);


#ifdef __cplusplus
        }
#endif

#endif /* !_MPP_H */