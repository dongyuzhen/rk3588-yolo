#include "v4l2_camera.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <sys/mman.h>

V4L2Camera::V4L2Camera() = default;

V4L2Camera::~V4L2Camera() {
    // 析构兜底：尽量优雅停流并释放资源
    stopStreaming();
    closeDevice();
}

bool V4L2Camera::init(const std::string& dev_path, int width, int height, int buffer_num, uint32_t pixfmt) {
    // 支持重复初始化：先清理旧状态
    closeDevice();

    this->width_ = width;
    this->height_ = height;
    this->buffer_num_ = buffer_num;
    this->pixfmt_ = pixfmt;

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

    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
        buf_type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    } else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        buf_type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    } else {
        std::cerr << "[V4L2] device doesn't support capture\n";
        closeDevice();
        return false;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        std::cerr << "[V4L2] device doesn't support streaming\n";
        closeDevice();
        return false;
    }

    // 配置采集格式
    v4l2_format fmt{};
    fmt.type = buf_type_;
    if (buf_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        fmt.fmt.pix_mp.width = this->width_;
        fmt.fmt.pix_mp.height = this->height_;
        fmt.fmt.pix_mp.pixelformat = this->pixfmt_;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;
    } else {
        fmt.fmt.pix.width = this->width_;
        fmt.fmt.pix.height = this->height_;
        fmt.fmt.pix.pixelformat = this->pixfmt_;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
    }

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        closeDevice();
        return false;
    }

    // 向驱动申请 MMAP 队列
    v4l2_requestbuffers req{};
    req.type = buf_type_;
    req.count = static_cast<uint32_t>(buffer_num_);
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS(MMAP)");
        closeDevice();
        return false;
    }

    // 过少缓冲容易流水线不足
    if (req.count < 2) {
        std::cerr << "[V4L2] insufficient MMAP count: " << req.count << "\n";
        closeDevice();
        return false;
    }

    buffer_num_ = static_cast<int>(req.count);

    // 建立内部缓冲，调用 mmap 并用 VIDIOC_EXPBUF 导出 DMABUF fd
    for (uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        v4l2_plane planes[1]{};
        buf.type = buf_type_;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (buf_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            buf.m.planes = planes;
            buf.length = 1;
        }

        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF");
            closeDevice();
            return false;
        }

        InternalBuffer ib{};
        if (buf_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            ib.length = buf.m.planes[0].length;
            ib.start = mmap(nullptr, ib.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.planes[0].m.mem_offset);
        } else {
            ib.length = buf.length;
            ib.start = mmap(nullptr, ib.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        }

        if (ib.start == MAP_FAILED) {
            perror("mmap");
            closeDevice();
            return false;
        }

        // 导出该 buffer 的 fd 给硬件零拷贝使用
        v4l2_exportbuffer expbuf{};
        expbuf.type = buf_type_;
        expbuf.index = i;
        if (ioctl(fd_, VIDIOC_EXPBUF, &expbuf) < 0) {
            perror("VIDIOC_EXPBUF");
            closeDevice();
            return false;
        }
        ib.dmabuf_fd = expbuf.fd;

        buffers_.push_back(ib);
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
        v4l2_buffer buf{};
        v4l2_plane planes[1]{};
        buf.type = buf_type_;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = static_cast<uint32_t>(i);
        if (buf_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            buf.m.planes = planes;
            buf.length = 1;
        }

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF(MMAP init)");
            return false;
        }

        buffer_states_[static_cast<size_t>(i)].dequeued = false;
    }

    return true;
}
bool V4L2Camera::startStreaming() {
    if (fd_ < 0) return false;
    if (is_streaming_) return true;

    v4l2_buf_type type = static_cast<v4l2_buf_type>(buf_type_);
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
    v4l2_plane planes[1]{};
    buf.type = buf_type_;
    buf.memory = V4L2_MEMORY_MMAP;
    if (buf_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        buf.m.planes = planes;
        buf.length = 1;
    }

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) return false;
        perror("VIDIOC_DQBUF");
        return false;
    }

    if (buf.index >= buffers_.size()) {
        std::cerr << "[V4L2] invalid buffer index: " << buf.index << "\n";
        return false;
    }

    buffer_states_[buf.index].dequeued = true;

    // 填充输出包：调用方只读使用
    out_packet.data = static_cast<const uint8_t*>(buffers_[buf.index].start);
    out_packet.bytes_used = static_cast<size_t>((buf_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) ? buf.m.planes[0].bytesused : buf.bytesused);
    out_packet.buffer_index = buf.index;
    out_packet.dmabuf_fd = buffers_[buf.index].dmabuf_fd;
    out_packet.valid = true;

    return true;
}

bool V4L2Camera::releaseFrame(const FramePacket& packet) {
    if (fd_ < 0 || !packet.valid) return false;
    if (packet.buffer_index >= buffers_.size()) return false;

    BufferState& state = this->buffer_states_[packet.buffer_index];

    // 必须是“已 DQ 未 Q”的帧才能释放
    if (!state.dequeued) return false;

    // 归还给驱动继续复用
    v4l2_buffer qbuf{};
    v4l2_plane planes[1]{};
    qbuf.type = buf_type_;
    qbuf.memory = V4L2_MEMORY_MMAP;
    qbuf.index = packet.buffer_index;
    if (buf_type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
        qbuf.m.planes = planes;
        qbuf.length = 1;
    }

    if (ioctl(fd_, VIDIOC_QBUF, &qbuf) < 0) {
        perror("VIDIOC_QBUF(MMAP recycle)");
        return false;
    }

    state.dequeued = false;
    return true;
}

bool V4L2Camera::stopStreaming() {
    if (fd_ < 0 || !is_streaming_) return true;

    v4l2_buf_type type = static_cast<v4l2_buf_type>(buf_type_);
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

        // 解除映射并关闭导出的 fd
        for (auto& b : buffers_) {
            if (b.start && b.start != MAP_FAILED) {
                munmap(b.start, b.length);
            }
            if (b.dmabuf_fd >= 0) {
                close(b.dmabuf_fd);
            }
        }
        buffers_.clear();
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
