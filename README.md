# RK3588 智慧工地安全检测系统

> 基于 RK3588 NPU + YOLOv5s 的边缘实时安全帽/防护装备检测与告警系统
> 实现 **30FPS 推流、端到端延迟 <200ms、CPU 占用 <10%**

---

## 项目简介

本项目是一套完整的**边缘智能安全监控系统**，部署在 Rockchip RK3588 开发板上，通过 V4L2 摄像头实时采集工地画面，利用板载 NPU 进行 YOLOv5s 目标检测，识别工人是否佩戴安全帽、安全背心、手套、安全靴等防护装备，一旦发现违规行为立即触发告警并通过飞书 Webhook 推送通知，同时通过 RTSP 协议将带标注画面实时推流至监控端。

### 核心指标

| 指标 | 数值 |
|------|------|
| 推理帧率 | **30 FPS**（NPU 3核并行，8帧 in-flight） |
| 端到端延迟 | **< 200ms**（采集→NPU→画框→编码→推流） |
| CPU 占用率 | **< 10%**（硬件编码 + RGA 零拷贝转换） |
| 检测类别 | 12 类（安全帽颜色分类 + 违规状态） |
| 推流协议 | **RTSP**（H.264，MPP 硬件编码） |

---

## 系统架构

### 四线程流水线设计

`
V4L2摄像头 (YUYV DMABUF)
     |
     v
[Thread 1: 采集线程 captureT]
     | SafeQueue<CapturedFrame>(24)  <-- Drop-Oldest 零延迟策略
     v
[Thread 2: 处理线程 processT]  <-- 8帧 in-flight NPU并行
     | SafeQueue<YoloOutputFrame>(24)
     v
[Thread 3: 写出线程 writeT]  <-- RGA画框 + MPP编码
     |                    |
     | AlarmEvent         | H.264 NAL
     v                    v
[Thread 4: 报警引擎]   RTSP推流队列 --> RTSP Server --> 监控端
  漏桶防抖
  多类独立冷却
  异步存图+Webhook
`

### 五大关键设计

#### 1. Drop-Oldest 零延迟入队
队列满时不阻塞采集端，而是弹出最老帧（同时归还 V4L2 dmabuf）并压入最新帧，保证 NPU 永远处理最新鲜画面。enqueue_drop_oldest 在锁内完成原子替换，锁外调用回调，避免死锁。

#### 2. NPU 多帧 In-flight 并行
处理线程维护 map<int, future> inflight 表（最多8帧），通过 
ext_out 指针保证结果按序输出，充分利用 NPU 流水线，实现高吞吐同时保持帧序。

#### 3. RGA 硬件零拷贝 Letterbox 预处理
- YUYV DMABUF → RGB888（RGA imcopy，不经 CPU）
- RGB888 等比缩放 + 灰色填充（RGA imresize + imfill，保持长宽比）
- rknn_create_mem 分配模型输入缓冲，构造时 import_buffer 一次，推理全程复用
- 后处理坐标逆变换：ox = (output - pad) / scale，精确还原原图坐标

#### 4. AlarmEngine 独立线程（单一职责）
写线程只负责编码推流，告警逻辑完全剥离到专用线程，IO 阻塞不再影响视频流水线。通过 SafeQueue<AlarmEvent> 解耦，仅违规帧才 clone BGRMat，无违规帧零开销。

#### 5. 漏桶防抖 + 多类别独立冷却
- **漏桶防抖**：出现 +1，超过 300ms 未见 -2，score ≥ 15 触发（约 0.5s@30fps 持续违规）
- **多类独立冷却**：每类违规各自维护 30s 冷却期，
o helmet 和 
o vest 互不压制（解决原 clear() 全局清零导致的交叉压制问题）

---

## 快速开始

### 环境要求

| 依赖 | 版本 |
|------|------|
| 硬件 | Rockchip RK3588 |
| OS | Ubuntu 22.04 / Debian 11 (aarch64) |
| 编译器 | GCC 11+，C++17 |
| OpenCV | 4.x |
| RKNN Runtime | 1.5.2+ |
| RGA | librga 1.9+ |
| MPP | rockchip-mpp |

### 编译

`ash
git clone https://github.com/dongyuzhen/rk3588-yolo.git
cd rk3588-yolo
git checkout feature/alarm-engine

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j16
`

### 运行

`ash
# 基础运行（RTSP 推流）
./rk3588_yolo

# 开启本地视频保存
./rk3588_yolo --save-local

# 关闭 RTSP 推流
./rk3588_yolo --no-rtmp
`

RTSP 拉流地址：tsp://192.168.x.x:8554/live

---

## 检测类别

| 类别 | 标注颜色 | 含义 |
|------|---------|------|
| no helmet / no vest / no glove / no boots | 红色 | 违规：缺少防护装备 |
| helmet on / helmet_* / vest / gloves / boots | 绿色 | 合规：防护装备齐全 |
| person | 蓝色 | 人员检测 |

---

## 告警机制

违规持续 ≥15 帧 → 各类独立 30s 冷却 → 触发告警：
1. 异步存图：iolations/alarm_{label}_{时间戳}.jpg
2. 飞书 Webhook：推送告警文本（5s 超时保护）
3. CSV 日志：iolations/alarm_log.csv 追加写入

---

## 性能数据

| 阶段 | 耗时 |
|------|------|
| RGA Letterbox 预处理 | ~2ms |
| NPU 推理（YOLOv5s） | ~15ms |
| RGA BGR→NV12 转换 | ~1ms |
| MPP H.264 编码 | ~5ms |
| **端到端总延迟** | **< 200ms** |

---

## 分支说明

| 分支 | 内容 |
|------|------|
| rtmp | 初始版本，RTMP 推流 |
| switch-to-rtsp | 迁移至 RTSP 协议 + MPP 硬件编码 |
| yolo-add-letterbox | RGA Letterbox 预处理，修复长宽比变形 |
| **feature/alarm-engine** | **当前主线**：AlarmEngine 独立线程，多类别独立冷却，飞书告警 |

---

## 作者

**董玉振** | 嵌入式 Linux / 边缘 AI 开发
Email: 2790977664@qq.com | GitHub: [dongyuzhen](https://github.com/dongyuzhen)
