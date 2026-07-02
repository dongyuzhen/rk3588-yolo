# 必知必会 02：MPP 与 FFmpeg 代码实战解析

在咱们的 `rk3588-yolo` 项目中，你需要接触到底层的 MPP（瑞芯微硬件编码）和 FFmpeg（网络推流）。它们俩虽然都是 C/C++ 库，但设计哲学完全不同。

## 1. 瑞芯微 MPP (Media Process Platform) 的使用套路

MPP 是一套非常底层的硬件调用接口，它的代码看起来极其繁琐（像你项目里的 `mpp.c` 有几百行），但其实它每次工作只遵循 4 个核心步骤：

1.  **上下文初始化 (Context Init)**
    *   告诉芯片：“我要创建一个编码器 (Encoder)，格式是 H.264，输入是 640x640 的 YUV 图像。”
    *   相关函数：`mpp_create`, `mpp_init`。
2.  **获取头部信息 (Get Header)**
    *   MPP 初始化好之后，它会立刻生成一份“说明书”（也就是上面提到的 SPS/PPS）。你需要把它拿出来存好。
3.  **循环编码 (Encode Loop) —— 这个最重要**
    *   你把 YOLO 处理完的图片包装成一个 `MppFrame` 送入芯片。
    *   芯片在后台“咔咔”运算，完成后，把结果打包成一个 `MppPacket`（这也就是 NALU 包）还给你。
    *   你从 `MppPacket` 里读出裸数据的指针 (`ptr`) 和大小 (`length`)。
4.  **资源释放 (Deinit)**
    *   结束时清理内存。

**关键点**：MPP 的输出仅仅是内存中的一块连续数据。它不管你是存到硬盘还是发到网线上，全靠你自己处理。

---

## 2. FFmpeg 推流的使用套路

FFmpeg 的核心在于**“封装”**。MPP 吐出来的东西叫“裸流”(Raw Stream)，不能直接在网络上传播。FFmpeg 需要给这些裸奔的数据穿上“网络协议的衣服”（比如 FLV, RTSP 等），这个过程叫 **Muxing (复用)**。

写 FFmpeg 推流代码，同样是 4 步走：

1.  **准备环境 (Format Context)**
    *   告诉 FFmpeg：“我要推流了，目标地址是 `rtsp://ip:8554/stream`，你要帮我准备好 RTSP 协议握手”。
    *   调用：`avformat_alloc_output_context2`, `avio_open`。
2.  **写入头部 (Write Header)**
    *   把刚才 MPP 生成的“说明书”(SPS/PPS) 告诉 FFmpeg，FFmpeg 会帮我们用网络协议发给客户端。
    *   调用：`avformat_write_header`。
3.  **循环写入数据帧 (Write Frame) —— 对接点！**
    *   这是把 MPP 和 FFmpeg 连起来的地方！
    *   你把 MPP 吐出来的裸数据，装进 FFmpeg 专属的一个信封（叫做 `AVPacket`）。
    *   给这个信封贴上邮票：写上展示时间 (`PTS`) 和 流索引 (`stream_index`)。
    *   把信封交给邮局：调用 `av_interleaved_write_frame` 发送到网络。
4.  **写入尾部并清理 (Write Trailer)**
    *   调用：`av_write_trailer` 告诉服务器结束推流。

## 3. 两者结合的“解耦”意义 (回应架构图)

在我们刚讨论的**观察者模式与队列架构**中：
*   **MPP 线程**负责不断执行上述的 MPP 套路 3（产生 `MppPacket`）。
*   它不直接交给 FFmpeg，而是把这个包扔进 **Queue（队列）**。
*   **RTSP 推流线程**专门执行 FFmpeg 套路 3。它从 Queue 里拿出数据，装进信封 (`AVPacket`)，调用 `av_interleaved_write_frame` 发走。

理解了这个工作流，你看任何音视频 C++ 代码，都会发现它们不过是这几个步骤的来回拼装！
