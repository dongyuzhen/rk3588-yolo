#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// 单个 CMA/DMABUF 封装：
// - 从 /dev/dma_heap/cma 分配
// - mmap 到用户态可读写
// - 提供 DMA_BUF_IOCTL_SYNC 的读同步接口
class CmaBuffer {
public:
    CmaBuffer() = default;
    ~CmaBuffer();

    CmaBuffer(const CmaBuffer&) = delete;
    CmaBuffer& operator=(const CmaBuffer&) = delete;

    // 允许移动，便于放进 std::vector 管理
    CmaBuffer(CmaBuffer&& other) noexcept;
    CmaBuffer& operator=(CmaBuffer&& other) noexcept;

    // 分配一块 CMA 连续内存并映射到用户态。
    // @param heap_fd  已打开的 /dev/dma_heap/cma fd
    // @param size     分配字节数
    // @param tag      调试标记（日志定位用）
    bool allocate(int heap_fd, size_t size, const std::string& tag = "");

    // 释放资源（munmap + close）
    void release();

    // CPU 读前同步：通知内核/设备做 cache 同步开始
    bool syncStartRead() const;

    // CPU 读后同步：通知内核/设备做 cache 同步结束
    bool syncEndRead() const;

    // CPU 写前同步：获取 buffer 的写权限
    bool syncStartWrite() const;

    // CPU 写后同步：将 CPU cache 刷入到设备物理内存（极重要，防撕裂/鬼影）
    bool syncEndWrite() const;

    int fd() const { return fd_; }
    void* addr() const { return addr_; }
    size_t size() const { return size_; }

    // 资源是否完整有效
    bool valid() const { return fd_ >= 0 && addr_ != nullptr && size_ > 0; }

private:
    int fd_{-1};           // dmabuf fd
    void* addr_{nullptr};  // mmap 后的虚拟地址
    size_t size_{0};       // buffer 字节数
    std::string tag_;      // 调试标签
};

// 多个 CmaBuffer 的池化封装：
// 常用于 V4L2 多缓冲、编码输入池等场景
class CmaBufferPool {
public:
    // @param heap_fd  已打开的 dma-heap fd（由调用方管理生命周期）
    // @param owns_fd  为 true 时析构时负责 close heap_fd，默认 false（外部管理）
    CmaBufferPool(int heap_fd, bool owns_fd = false);
    ~CmaBufferPool();

    // 不允许复制，避免误用导致资源管理混乱
    CmaBufferPool(const CmaBufferPool&) = delete;
    CmaBufferPool& operator=(const CmaBufferPool&) = delete;

    // 批量创建 count 个等大小 buffer
    bool init(size_t count, size_t each_size, const std::string& tag_prefix = "pool");

    // 清空池子（触发每个 CmaBuffer 自动释放）
    void clear();

    size_t size() const { return buffers_.size(); }
    CmaBuffer& at(size_t idx) { return buffers_.at(idx); }
    const CmaBuffer& at(size_t idx) const { return buffers_.at(idx); } 

private:
    std::vector<CmaBuffer> buffers_;    // buffer 列表
    int heap_fd_{-1};  // 分配用的 CMA heap fd，便于统一管理和调试
    bool owns_fd_{false}; // 是否由本池负责关闭 heap_fd_（默认 false，外部管理）
};
