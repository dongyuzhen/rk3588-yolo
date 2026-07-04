#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <linux/videodev2.h>

#include "cma_buffer.h"

typedef struct FramePacket {
    const uint8_t* data{nullptr}; // 指向 CMA 映射地址，直到 releaseFrame 前有效，已经零拷贝到用户态
    size_t bytes_used{0};           // 实际数据长度（可能小于缓冲大小）
    uint32_t buffer_index{0};       // 对应 V4L2 buffer slot index/后续是captureFrame/releaseFrame的索引
    int dmabuf_fd{-1};                  // 对应 CMA buffer 的 dmabuf fd
    bool valid{false};              //包是否有效（captureFrame 成功时为 true，失败或 releaseFrame 后为 false）
} FramePacket_t;

// V4L2 相机封装（MMAP + DMABUF Export 版本）：
// - V4L2 驱动侧负责分配连续内存（V4L2_MEMORY_MMAP）
// - 用户态自动通过 VIDIOC_EXPBUF 导出 dmabuf fd 供后续硬件（RGA/MPP）零拷贝使用
// - 调用方通过 captureFrame/releaseFrame 取还帧
class V4L2Camera {
public:
    // 单帧数据描述。
    // 注意：data 指向内部 CMA 缓冲映射地址，直到 releaseFrame 前有效。

    V4L2Camera();
    ~V4L2Camera();

    V4L2Camera(const V4L2Camera&) = delete;
    V4L2Camera& operator=(const V4L2Camera&) = delete;

    // 初始化设备与缓冲池：
    // 1) 打开 /dev/videoX
    // 2) 设置分辨率/格式
    // 3) 申请 V4L2 MMAP 队列
    // 4) mmap 映射并 export 导出 dmabuf fd
    // 5) 全量 QBUF 入队
    bool init(const std::string& dev_path, int width, int height, int buffer_num, uint32_t pixfmt = V4L2_PIX_FMT_MJPEG);

    // 启动采集流
    bool startStreaming();

    // 抓取一帧：poll -> DQBUF -> syncStartRead -> 返回 FramePacket
    bool captureFrame(FramePacket& out_packet, int poll_timeout_ms = 100);

    // 释放一帧：syncEndRead -> QBUF 回驱动
    bool releaseFrame(const FramePacket& packet);

    // 停止采集流
    bool stopStreaming();

    // 关闭设备并清理缓冲
    void closeDevice();

private:
    // 仅记录每个缓冲当前是否已 DQ 未 Q
    struct BufferState {
        bool dequeued{false};
    };

    // 将全部缓冲先行 QBUF
    bool queueAllBuffers();

    struct InternalBuffer {
        void* start;
        size_t length;
        int dmabuf_fd;
    };

    int fd_{-1};
    int heap_fd_{-1};
    int width_{0};
    int height_{0};
    int buffer_num_{0};
    uint32_t pixfmt_{V4L2_PIX_FMT_MJPEG};
    bool is_streaming_{false};
    uint32_t buf_type_{V4L2_BUF_TYPE_VIDEO_CAPTURE};

    // 内部采集缓冲映射
    std::vector<InternalBuffer> buffers_;
    std::vector<BufferState> buffer_states_;
};
