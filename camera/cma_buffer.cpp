#include "cma_buffer.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

CmaBuffer::~CmaBuffer() {
    release();
}

CmaBuffer::CmaBuffer(CmaBuffer&& other) noexcept {
    // 转移所有权：接管 other 的 fd/addr/size
    fd_ = other.fd_;
    addr_ = other.addr_;
    size_ = other.size_;
    tag_ = std::move(other.tag_);

    // 将 other 置空，避免双重释放
    other.fd_ = -1;
    other.addr_ = nullptr;
    other.size_ = 0;
}

CmaBuffer& CmaBuffer::operator=(CmaBuffer&& other) noexcept {
    if (this == &other) return *this;

    // 先释放当前对象资源，再接管 other 资源
    release();

    fd_ = other.fd_;
    addr_ = other.addr_;
    size_ = other.size_;
    tag_ = std::move(other.tag_);

    other.fd_ = -1;
    other.addr_ = nullptr;
    other.size_ = 0;

    return *this;
}

bool CmaBuffer::allocate(int heap_fd, size_t size, const std::string& tag) {
    // 支持重复 allocate：先释放旧资源
    release();

    if (heap_fd < 0 || size == 0) {
        std::cerr << "[CmaBuffer] invalid alloc args" << std::endl;
        return false;
    }

    // 向 dma-heap 申请一块 CMA 连续内存，返回 dmabuf fd
    dma_heap_allocation_data alloc{};
    alloc.len = size;
    alloc.fd_flags = O_RDWR | O_CLOEXEC;
    alloc.heap_flags = 0;

    if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &alloc) < 0) {
        std::perror("DMA_HEAP_IOCTL_ALLOC");
        return false;
    }

    // 映射到用户态地址，后续可直接读写
    void* mapped = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, alloc.fd, 0);
    if (mapped == MAP_FAILED) {
        std::perror("mmap dmabuf");
        close(alloc.fd);
        return false;
    }

    fd_ = alloc.fd;
    addr_ = mapped;
    size_ = size;
    tag_ = tag;
    return true;
}

void CmaBuffer::release() {
    // 先解除映射
    if (addr_ && size_ > 0) {
        munmap(addr_, size_);
        addr_ = nullptr;
    }

    // 再关闭 fd
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }

    size_ = 0;
    tag_.clear();
}

bool CmaBuffer::syncStartWrite() const {
    if (fd_ < 0) return false;

    // CPU 写前同步
    dma_buf_sync sync_start{};
    sync_start.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE;
    if (ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync_start) < 0) {
        std::perror("DMA_BUF_IOCTL_SYNC START WRITE");
        return false;
    }
    return true;
}

bool CmaBuffer::syncEndWrite() const {
    if (fd_ < 0) return false;

    // CPU 写后同步：刷 CPU Cache，保证 RGA 读取时不闪烁、不鬼影
    dma_buf_sync sync_end{};
    sync_end.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
    if (ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync_end) < 0) {
        std::perror("DMA_BUF_IOCTL_SYNC END WRITE");
        return false;
    }
    return true;
}
