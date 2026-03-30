#include "v4l2_camera.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

V4L2Camera::V4L2Camera() = default;

V4L2Camera::~V4L2Camera() {
    // 析构兜底：尽量优雅停流并释放资源
    stopStreaming();
    closeDevice();
}

bool V4L2Camera::init(const std::string& dev_path, int width, int height, int buffer_num, uint32_t pixfmt) {
    // 支持重复初始化：先清理旧状态
    closeDevice();

    // 非阻塞打开视频设备
    fd_ = open(dev_path.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
        perror("open camera");
        return false;
    }

    // 校验设备能力（必须支持 capture + streaming）
    v4l2_capability cap{};
    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("VIDIOC_QUERYCAP");
        closeDevice();
        return false;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) || !(cap.capabilities & V4L2_CAP_STREAMING)) {
        std::cerr << "[V4L2] device doesn't support capture+streaming\n";
        closeDevice();
        return false;
    }

    width_ = width;
    height_ = height;
    buffer_num_ = buffer_num;
    pixfmt_ = pixfmt;

    // 配置采集格式
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixfmt;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        closeDevice();
        return false;
    }

    // 向驱动申请 DMABUF 队列
    v4l2_requestbuffers req{};
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.count = static_cast<uint32_t>(buffer_num_);
    req.memory = V4L2_MEMORY_DMABUF;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS(DMABUF)");
        closeDevice();
        return false;
    }

    // 过少缓冲容易流水线不足
    if (req.count < 2) {
        std::cerr << "[V4L2] insufficient DMABUF count: " << req.count << "\n";
        closeDevice();
        return false;
    }

    buffer_num_ = static_cast<int>(req.count);

    // 打开 CMA heap（用于分配连续物理内存）
    heap_fd_ = open("/dev/dma_heap/cma", O_RDWR);
    if (heap_fd_ < 0) {
        perror("open /dev/dma_heap/cma");
        closeDevice();
        return false;
    }

    // 建立采集缓冲池：每个 V4L2 slot 对应一个 CmaBuffer
    if (!cma_pool_.init(static_cast<size_t>(buffer_num_), heap_fd_, fmt.fmt.pix.sizeimage, "capture")) {
        std::cerr << "[V4L2] failed to init cma pool\n";
        closeDevice();
        return false;
    }

    buffer_states_.assign(static_cast<size_t>(buffer_num_), BufferState{});

    // 启动前把所有 buffer 先 QBUF 入队
    if (!queueAllBuffers()) {
        closeDevice();
        return false;
    }

    is_streaming_ = false;
    return true;
}

bool V4L2Camera::queueAllBuffers() {
    for (int i = 0; i < buffer_num_; ++i) {
        CmaBuffer& b = cma_pool_.at(static_cast<size_t>(i));

        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_DMABUF;
        buf.index = static_cast<uint32_t>(i);
        buf.m.fd = b.fd();
        buf.length = static_cast<uint32_t>(b.size());

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF(DMABUF init)");
            return false;
        }

        buffer_states_[static_cast<size_t>(i)].dequeued = false;
    }

    return true;
}

bool V4L2Camera::startStreaming() {
    if (fd_ < 0) return false;
    if (is_streaming_) return true;

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        return false;
    }

    is_streaming_ = true;
    return true;
}

bool V4L2Camera::captureFrame(FramePacket& out_packet, int poll_timeout_ms) {
    // 默认置空，只有完整成功后才标记 valid=true
    out_packet = FramePacket{};

    if (fd_ < 0 || !is_streaming_) return false;

    // 用 poll 等待可读，避免盲目阻塞 DQBUF
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    int pret = poll(&pfd, 1, poll_timeout_ms);
    if (pret < 0) {
        if (errno == EINTR) return false;
        perror("poll");
        return false;
    }
    if (pret == 0) return false;  // 超时
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        std::cerr << "[V4L2] poll revents error: 0x" << std::hex << pfd.revents << std::dec << "\n";
        return false;
    }
    if (!(pfd.revents & POLLIN)) return false;

    // 出队一帧
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) return false;
        perror("VIDIOC_DQBUF");
        return false;
    }

    if (buf.index >= cma_pool_.size()) {
        std::cerr << "[V4L2] invalid buffer index: " << buf.index << "\n";
        return false;
    }

    CmaBuffer& b = cma_pool_.at(buf.index);
    if (!b.valid()) {
        std::cerr << "[V4L2] invalid cma buffer at index " << buf.index << "\n";
        return false;
    }

    // CPU 读取前做 cache sync start
    if (!b.syncStartRead()) {
        return false;
    }

    buffer_states_[buf.index].dequeued = true;

    // 填充输出包：调用方只读使用
    out_packet.data = static_cast<const uint8_t*>(b.addr());
    out_packet.bytes_used = static_cast<size_t>(buf.bytesused);
    out_packet.buffer_index = buf.index;
    out_packet.dmabuf_fd = b.fd();
    out_packet.driver_timestamp_us =
        static_cast<uint64_t>(buf.timestamp.tv_sec) * 1000000ULL + static_cast<uint64_t>(buf.timestamp.tv_usec);
    out_packet.valid = true;

    return true;
}

bool V4L2Camera::releaseFrame(const FramePacket& packet) {
    if (fd_ < 0 || !packet.valid) return false;
    if (packet.buffer_index >= cma_pool_.size()) return false;

    CmaBuffer& b = cma_pool_.at(packet.buffer_index);
    BufferState& state = buffer_states_[packet.buffer_index];

    // 必须是“已 DQ 未 Q”的帧才能释放
    if (!state.dequeued || !b.valid()) return false;

    // CPU 读取完成后做 cache sync end
    if (!b.syncEndRead()) {
        return false;
    }

    // 归还给驱动继续复用
    v4l2_buffer qbuf{};
    qbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    qbuf.memory = V4L2_MEMORY_DMABUF;
    qbuf.index = packet.buffer_index;
    qbuf.m.fd = b.fd();
    qbuf.length = static_cast<uint32_t>(b.size());

    if (ioctl(fd_, VIDIOC_QBUF, &qbuf) < 0) {
        perror("VIDIOC_QBUF(DMABUF recycle)");
        return false;
    }

    state.dequeued = false;
    return true;
}

bool V4L2Camera::stopStreaming() {
    if (fd_ < 0 || !is_streaming_) return true;

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) {
        perror("VIDIOC_STREAMOFF");
        return false;
    }

    is_streaming_ = false;
    return true;
}

void V4L2Camera::closeDevice() {
    if (fd_ >= 0) {
        // 先停流，再释放缓冲，最后关设备
        stopStreaming();

        cma_pool_.clear();
        buffer_states_.clear();

        close(fd_);
        fd_ = -1;
    }

    if (heap_fd_ >= 0) {
        close(heap_fd_);
        heap_fd_ = -1;
    }

    is_streaming_ = false;
}
