#pragma once
#include <stdint.h>

// 获取硬件编码器生成的 SPS/PPS 头数据，用于 RTSP 会话 SDP 生成
void get_mpp_sps_pps(uint8_t** data, int* size);

// 初始化 MPP 硬件编码器
// callback：处理函数（由 main.cpp 传入，用于压包入队）
int init_mpp_encoder(int width, int height, int fps, int bitrate, int (*callback)(void*, uint8_t*, int));

// 零拷贝编码：启动编码，不再需要传 fd 和 size，直接使用内部 buffer
int encode_mpp_frame();

// 导出 MPP 内部 buffer 供外部（如 RGA）直接写入
int get_mpp_input_fd();
void begin_mpp_input_sync();

// 关闭编码器
void close_mpp_encoder();