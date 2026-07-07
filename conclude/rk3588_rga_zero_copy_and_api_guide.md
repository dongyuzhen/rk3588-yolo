# RK3588 零拷贝架构与 RGA API 核心机制开发指南

本文档总结了在 RK3588 平台上开发高性能视频/视觉流水线时，如何利用 DMA-BUF 和 RGA（2D Graphics Acceleration）硬件实现极致性能的零拷贝（Zero-Copy）架构，并提供核心 RGA API 的底层工作原理与通用代码示例。

## 一、 全链路零拷贝与内存分配策略

在视频流水线中，避免 CPU 参与像素搬运是性能优化的核心。通过精确规划不同硬件模块的内存分配策略，可以实现真正的端到端零拷贝。

### 1. 经典异构流水线中的内存数据源
在典型的“摄像头输入 -> AI/OSD 处理 -> 视频编码输出”流水线中，不同阶段的 DMA 内存通常有不同来源：

*   **硬件外设输入（如 V4L2 Camera）**：
    *   **分配方式**：由底层外设驱动在内核态分配（如 `V4L2_MEMORY_MMAP`），导出为 `dmabuf fd`。
    *   **特性**：通常为物理连续内存（CMA）。由于底层 DMA 控制器往往缺乏高级 IOMMU，交由驱动自行管理连续内存最为稳妥。
*   **中间计算与绘制缓冲（如 AI 预处理、OSD 图层）**：
    *   **分配方式**：由用户层程序通过 `open("/dev/dma_heap/system")` 或类似机制分配，导出为 `dmabuf fd`。
    *   **特性**：**物理离散（按页分配）**。对于这类大块的中间缓存，推荐使用 System Heap 而非连续的 CMA。
*   **硬件编码器缓冲（如 MPP 编码）**：
    *   **分配方式**：由底层硬件编码框架（如 MPP）的内部缓冲池分配。
    *   **特性**：满足特定硬件（如 H.265 编码器）对物理连续性、对齐和跨距（Stride）的严苛要求。

### 2. 高阶技巧：为什么推荐使用 System Heap 代替 CMA？
*   **CMA 的局限**：物理连续内存（CMA）是系统的稀缺资源。频繁申请和释放 1080P/4K 级别的大块内存极易导致 CMA 碎片化，最终引发系统崩溃（OOM）。
*   **IOMMU 的魔法**：RK3588 的 RGA、NPU 等现代硬件模块内置了 **IOMMU (Input-Output Memory Management Unit)**。它可以接收 System Heap 产生的物理散列页面（Scatter-Gather List），并在硬件内部将其映射为一段**连续的设备虚拟地址**。
*   **架构优势**：结合 System Heap 和 IOMMU，既摆脱了 CMA 碎片化和容量受限的死穴，又完美保持了 `dmabuf` 零拷贝特性。

---

## 二、 核心 RGA (librga) API 解析与实战

RGA 的核心 API (IM2D) 遵循特定的抽象逻辑：先将内存转化为 RGA 认识的“画板（Canvas）”，再定义操作的“画框（Frame）”。

### 1. 内存管理与句柄生命周期
*   **`importbuffer_fd(fd, ...)`**：将系统的 `dmabuf fd` 导入为 RGA 硬件句柄。在此过程中只发生 IOMMU 页表映射，没有内存拷贝。
    *   **优化建议**：对于生命周期长的缓冲（如背景板、输出目标池），应在初始化时**缓存 Handle**。在每帧的热路径中避免重复 import，以节省内核态切换开销。
*   **`releasebuffer_handle(handle)`**：释放 RGA 句柄。必须妥善管理，避免耗尽内核的 IOMMU 资源。

### 2. “画板”定义：`wrapbuffer_handle`
```cpp
rga_buffer_t dst_buf = wrapbuffer_handle(handle, width, height, format, w_stride, h_stride);
```
这个函数定义了目标内存的物理总容量和边界，相当于创建了一个**全局画板 (Canvas)**。RGA 会依据此处的 `w_stride` 计算内存偏移，并进行严格的越界检查。

### 3. “画框”定义与核心处理：`improcess` 与 `im_rect`
`improcess` 是多功能 API，可同时完成格式转换、缩放、裁剪和多图层叠加。其最关键的是三个 `im_rect` 操作画框：
`improcess(src, dst, pat, srect, drect, prect, ...)`

*   **`srect` (Source Rect)**：从源缓冲（`src`）提取数据的区域。
*   **`drect` (Destination Rect)**：写入目标缓冲（`dst`）的区域。RGA 自动根据 `srect` 和 `drect` 差异进行硬件缩放。
*   **`prect` (Pattern Rect)**：从第三辅助缓冲（`pat`，如 OSD 层）提取数据的区域。

#### 通用实战样例 A：硬件 Letterbox (AI 预处理)
目标：将不规则原图等比缩放并居中放置在方形 NPU 输入中，四周填充灰边。

```cpp
// 1. 准备画板
rga_buffer_t src = wrapbuffer_handle(src_handle, src_w, src_h, RK_FORMAT_YCbCr_420_SP);
rga_buffer_t dst = wrapbuffer_handle(dst_handle, npu_w, npu_h, RK_FORMAT_RGB_888);
rga_buffer_t empty_pat; memset(&empty_pat, 0, sizeof(rga_buffer_t)); // 不需要第三图层

// 2. CPU 提前绘制上下左右的灰边 (仅涂抹边缘，极少内存开销)
// ... memset(dst_ptr, 114, ...); 

// 3. 定义画框并执行
im_rect srect = {0, 0, src_w, src_h}; // 源图全图
im_rect drect = {pad_left, pad_top, resized_w, resized_h}; // 目标居中区域
im_rect prect = {0, 0, 0, 0}; // 空占位符

// 硬件一步完成：NV12转RGB + 缩放 + 定位居中写入
improcess(src, dst, empty_pat, srect, drect, prect, IM_SYNC);
```

#### 通用实战样例 B：OSD 透明叠加 (画中画/UI合成)
目标：将带有 Alpha 通道的 UI 图层叠加到视频帧上。

```cpp
// 1. 准备画板 (假设 OSD 也是全屏尺寸)
rga_buffer_t bg_src = wrapbuffer_handle(src_handle, w, h, RK_FORMAT_YCbCr_420_SP);
rga_buffer_t ui_pat = wrapbuffer_handle(ui_handle, w, h, RK_FORMAT_BGRA_8888); // 带有 Alpha
rga_buffer_t dst    = wrapbuffer_handle(dst_handle, w, h, RK_FORMAT_YCbCr_420_SP);

// 2. 定义画框
im_rect srect = {0, 0, w, h};
im_rect drect = {0, 0, w, h};
im_rect prect = {0, 0, w, h}; // 如果只需叠加小图标，可缩小 prect/drect 范围

// 硬件检测到 BGRA 格式，自动激活 Alpha Blending 叠加到 dst
improcess(bg_src, dst, ui_pat, srect, drect, prect, IM_SYNC);
```

---

## 三、 OSD 绘制的工程化极致优化最佳实践

将 OSD 拆分为**软件渲染**（如 OpenCV）和**硬件合成**（RGA）两步是极佳的架构选择。在软件渲染 DMA 画布阶段，建议遵循以下 6 个达到极致性能和效果的优化点：

### 1. 映射 DMA 内存为软画布
不要新建常规内存后拷贝给硬件，应直接将 DMA 的虚拟地址交给图形库。
```cpp
// 直接在 DMA 内存上绘图
cv::Mat bgra_canvas(height, width, CV_8UC4, dma_virtual_addr);
```

### 2. ARM64 硬件指令加速画布清空
在 `BGRA_8888` 格式中，全零即代表完全透明。相比使用图形库（如 `cv::Mat::setTo`）逐像素赋值，直接使用 C 函数 `memset` 在 ARM64 架构下会触发 `DC ZVA` 硬件指令，以 Cache line 为单位进行清零，速度快数倍。
```cpp
memset(dma_virtual_addr, 0, total_size);
```

### 3. CPU 强制 Cache 同步
CPU 操作映射的 DMA 虚拟地址时，数据停留在 L1/L2 缓存中。**绘制完成后，必须通过 `DMA_BUF_IOCTL_SYNC` 执行写后同步（Flush Cache）**。若无此步，硬件 RGA 读取物理内存时会读到残影或造成画面撕裂。

### 4. 硬件级色彩学避坑：坐标强制偶数对齐
若底层视频流为 `NV12 (YUV 4:2:0)` 格式，其 UV 色度分量是 `2x2` 像素块共享的。
在绘制 UI 边缘（如框、线条）时，应强行将坐标对齐到偶数：
```cpp
int xmin = x & ~1;
int ymin = y & ~1;
```
如果边缘处于奇数坐标，RGA 合成时会导致彩色像素的 UV 值和背景挤在同一个色度块内，引发边缘的彩色虚线或色度撕裂。

### 5. Alpha 通道的严谨控制
赋予颜色时必须显式指定 Alpha 通道。例如在 OpenCV 中必须传入 4 个参数 `cv::Scalar(B, G, R, 255)`。如果 Alpha 设为 0，尽管软绘制能看到像素，但 RGA 进行硬件 Alpha Blending 时该部分会完全消失。

### 6. 工业级高对比度字体：双层描边
在复杂背景下，单色字体易难以辨认。推荐采用双层绘制法：先画一层粗的黑底（如 `thickness=4`），再在同一坐标画一层细的彩色正文（如 `thickness=2`），形成高对比度描边效果，确保任何亮度下均清晰可见。
