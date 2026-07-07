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
