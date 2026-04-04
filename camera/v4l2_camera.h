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

// V4L2 相机封装（DMABUF/CMA 版本）：
// - V4L2 驱动侧使用 V4L2_MEMORY_DMABUF
// - 用户态通过 CmaBufferPool 管理采集缓冲
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
    // 3) 申请 V4L2 DMABUF 队列
    // 4) 从 /dev/dma_heap/cma 分配同等数量的 CmaBuffer
    // 5) 全量 QBUF 入队
    bool init(const std::string& dev_path, int width, int height, int buffer_num, uint32_t pixfmt = V4L2_PIX_FMT_MJPEG, CmaBufferPool* pool = nullptr);

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

    // 将 cma_pool_ 中全部缓冲先行 QBUF
    bool queueAllBuffers();

    int fd_{-1};
    int heap_fd_{-1};
    int width_{0};
    int height_{0};
    int buffer_num_{0};
    uint32_t pixfmt_{V4L2_PIX_FMT_MJPEG};
    bool is_streaming_{false};

    // 采集缓冲池：一项对应一个 V4L2 buffer slot
    CmaBufferPool* cma_pool_;
    std::vector<BufferState> buffer_states_;
};
