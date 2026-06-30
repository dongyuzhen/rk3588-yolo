#pragma once
#include <string>
#include <thread>
#include <atomic>
#include "stream_types.h"
#include "../SafeQueue.h" 

// 提前声明 FFmpeg 结构
struct AVFormatContext;
struct AVStream;

class RtspStreamer{
public:
    RtspStreamer(const std::string& url, int width, int height, int fps, SafeQueue<EncodedPacket>& queue);
    ~RtspStreamer();

private:
    std::string url_;
    int width_, height_, fps_;

    // 引用外部的队列（不占所有权，只做消费者）
    SafeQueue<EncodedPacket>& packet_queue_;

   // FFmpeg 相关的上下文
    AVFormatContext* ofmt_ctx_ = nullptr;
    AVStream* out_stream_ = nullptr;

    // 独立的后台推流线程
    std::thread push_thread_;
    std::atomic<bool> is_running_;

    // 内部方法
    bool initFFmpeg();
    void pushLoop(); // 真正调用 FFmpeg API 的线程函数
    void cleanupFFmpeg();
}