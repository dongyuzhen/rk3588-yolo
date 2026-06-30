#ifndef _STREAMER_H_
#define _STREAMER_H_

#ifdef __cplusplus
extern "C" {
#endif

// 初始化流媒体推送器
// width: 视频宽度
// height: 视频高度
// fps: 帧率
// bitrate: 比特率
// rtmp_url: RTMP推流地址
// enable_rtmp: 是否启用RTMP推流 (1=启用, 0=禁用，仅保留编码功能)
int init_streamer(int width, int height, int fps, int bitrate, const char *rtmp_url, int enable_rtmp);

// 处理一帧图像数据（有拷贝）
// frame_data: NV12 数据虚拟地址
// frame_size: 字节数
int process_frame(uint8_t *frame_data, int frame_size);

// 零拷贝编码：直接传入 NV12 dmabuf fd，编码器跨驱动直接读取
// fd        : CmaBuffer 的 dmabuf fd
// frame_size: 字节数（hor_stride * ver_stride * 3/2）
int process_frame_fd(int fd, int frame_size);

// 关闭流媒体推送器
void close_streamer();

#ifdef __cplusplus
}
#endif

#endif /* _STREAMER_H_ */ 