#pragma once
#include <stdint.h>

// 获取硬件编码器生成的 SPS/PPS 头数据，用于 RTSP 会话 SDP 生成
void get_mpp_sps_pps(uint8_t** data, int* size);

// 初始化 MPP 硬件编码器
// callback：处理函数（由 main.cpp 传入，用于压包入队）
int init_mpp_encoder(int width, int height, int fps, int bitrate, int (*callback)(void*, uint8_t*, int));

// 零拷贝编码：直接传入 NV12 dmabuf fd，编码器跨驱动直接读取
// fd        : CmaBuffer 的 dmabuf fd
// frame_size: 字节数（hor_stride * ver_stride * 3/2）
int process_frame_fd(int fd, int frame_size);

// 关闭编码器
void close_mpp_encoder();