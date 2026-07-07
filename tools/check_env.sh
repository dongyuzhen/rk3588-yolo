#!/bin/bash
# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  RK3588 智慧工地安防系统 —— 运行环境检查脚本                                 ║
# ║  用法: bash tools/check_env.sh                                               ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

PASS=0
FAIL=0
WARN=0

pass() { echo -e "  ${GREEN}[✓]${NC} $1"; PASS=$((PASS+1)); }
fail() { echo -e "  ${RED}[✗]${NC} $1  ${YELLOW}<- 必须修复${NC}"; FAIL=$((FAIL+1)); }
warn() { echo -e "  ${YELLOW}[!]${NC} $1"; WARN=$((WARN+1)); }
info() { echo -e "  ${CYAN}[i]${NC} $1"; }

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  RK3588 Smart Site — Environment Check                     ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# ══════════════════════════════════════════════════════════════════════════════
# 1. 操作系统 & 架构
# ══════════════════════════════════════════════════════════════════════════════
echo "── 1. 系统 & 架构 ──"
ARCH=$(uname -m)
KERNEL=$(uname -r)
if [ "$ARCH" = "aarch64" ]; then
    pass "架构: $ARCH (ARM64, 正确)"
else
    fail "架构: $ARCH (需要 aarch64/ARM64, RK3588 是 ARM 平台)"
fi
info "内核版本: $KERNEL"

# ══════════════════════════════════════════════════════════════════════════════
# 2. 编译器工具链
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "── 2. 编译工具链 ──"

# g++
if command -v g++ &>/dev/null; then
    GXX_VER=$(g++ -dumpversion)
    GXX_MAJOR=$(echo "$GXX_VER" | cut -d. -f1)
    if [ "$GXX_MAJOR" -ge 9 ]; then
        pass "g++ $GXX_VER (C++17 可用)"
    else
        warn "g++ $GXX_VER (建议 ≥9.0, C++17 需要 ≥7.0)"
    fi
else
    fail "g++ 未安装 (sudo apt install g++)"
fi

# cmake
if command -v cmake &>/dev/null; then
    CMAKE_VER=$(cmake --version | head -1 | awk '{print $3}')
    CMAKE_MAJOR=$(echo "$CMAKE_VER" | cut -d. -f1)
    CMAKE_MINOR=$(echo "$CMAKE_VER" | cut -d. -f2)
    if [ "$CMAKE_MAJOR" -gt 3 ] || ([ "$CMAKE_MAJOR" -eq 3 ] && [ "$CMAKE_MINOR" -ge 16 ]); then
        pass "cmake $CMAKE_VER"
    else
        fail "cmake $CMAKE_VER (需要 ≥3.16)"
    fi
else
    fail "cmake 未安装 (sudo apt install cmake)"
fi

# pkg-config
if command -v pkg-config &>/dev/null; then
    pass "pkg-config 已安装"
else
    fail "pkg-config 未安装 (sudo apt install pkg-config)"
fi

# make
if command -v make &>/dev/null; then
    pass "make 已安装"
else
    fail "make 未安装 (sudo apt install make)"
fi

# ══════════════════════════════════════════════════════════════════════════════
# 3. 运行时库
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "── 3. 运行时依赖库 ──"

# OpenCV
if pkg-config --exists opencv4 2>/dev/null; then
    OCV_VER=$(pkg-config --modversion opencv4)
    pass "OpenCV4 $OCV_VER"
elif ldconfig -p 2>/dev/null | grep -q libopencv; then
    warn "OpenCV 已安装但无 pkg-config (可能影响 CMake 查找)"
else
    fail "OpenCV4 未安装 (sudo apt install libopencv-dev)"
fi

# FFmpeg
FFMPEG_OK=1
for lib in libavformat libavcodec libavutil; do
    if ! pkg-config --exists "$lib" 2>/dev/null; then
        FFMPEG_OK=0
        break
    fi
done
if [ "$FFMPEG_OK" -eq 1 ]; then
    FF_VER=$(pkg-config --modversion libavcodec)
    pass "FFmpeg libs $FF_VER (avformat/avcodec/avutil)"
else
    fail "FFmpeg 开发库缺失 (sudo apt install libavformat-dev libavcodec-dev libavutil-dev)"
fi

# pthread
if ldconfig -p 2>/dev/null | grep -q libpthread; then
    pass "pthread 可用"
else
    fail "pthread 不可用"
fi

# ══════════════════════════════════════════════════════════════════════════════
# 4. Rockchip 硬件驱动 & 库
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "── 4. Rockchip 硬件驱动 & 库 ──"

# MPP (librockchip_mpp)
if ldconfig -p 2>/dev/null | grep -q librockchip_mpp; then
    MPP_SO=$(ldconfig -p | grep librockchip_mpp | head -1 | awk '{print $NF}')
    pass "librockchip_mpp.so ($MPP_SO)"
elif [ -f /usr/lib/aarch64-linux-gnu/librockchip_mpp.so ]; then
    pass "librockchip_mpp.so (系统路径)"
else
    fail "librockchip_mpp.so 未找到 (需要安装 Rockchip MPP 库)"
fi

# RKNN Runtime (librknnrt.so)
RKNN_SO=""
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
for path in \
    "$PROJECT_DIR/3rdparty/librknn_api/aarch64/librknnrt.so" \
    /usr/lib/librknnrt.so \
    /usr/lib/aarch64-linux-gnu/librknnrt.so; do
    if [ -f "$path" ]; then
        RKNN_SO="$path"
        break
    fi
done
if [ -n "$RKNN_SO" ]; then
    pass "librknnrt.so ($RKNN_SO)"
else
    fail "librknnrt.so 未找到 (需要放到 3rdparty/librknn_api/aarch64/)"
fi

# RGA (librga.so)
RGA_SO=""
for path in \
    "$PROJECT_DIR/3rdparty/rga/RK3588/lib/Linux/aarch64/librga.so" \
    /usr/lib/librga.so \
    /usr/lib/aarch64-linux-gnu/librga.so; do
    if [ -f "$path" ]; then
        RGA_SO="$path"
        break
    fi
done
if [ -n "$RGA_SO" ]; then
    pass "librga.so ($RGA_SO)"
else
    fail "librga.so 未找到 (需要放到 3rdparty/rga/RK3588/lib/Linux/aarch64/)"
fi

# ══════════════════════════════════════════════════════════════════════════════
# 5. 内核驱动节点
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "── 5. 内核驱动节点 ──"

# RGA 设备
if [ -c /dev/rga ]; then
    pass "/dev/rga 设备节点存在"
elif [ -c /dev/rga3e ]; then
    pass "/dev/rga3e 设备节点存在"
else
    warn "/dev/rga 不存在 (RGA 硬件加速不可用, 检查内核配置)"
fi

# NPU 设备
NPU_COUNT=$(ls /dev/rknpu* 2>/dev/null | wc -l)
if [ "$NPU_COUNT" -ge 1 ]; then
    pass "NPU 设备节点: $(ls /dev/rknpu* 2>/dev/null | tr '\n' ' ')"
else
    fail "NPU 设备节点不存在 (/dev/rknpu* 缺失)"
fi

# MPP 服务
if [ -c /dev/mpp_service ]; then
    pass "/dev/mpp_service 设备节点存在"
else
    warn "/dev/mpp_service 不存在 (VPU 硬件编码可能不可用)"
fi

# DMA-BUF heap
DMA_HEAPS=$(ls /dev/dma_heap/ 2>/dev/null | tr '\n' ' ')
if echo "$DMA_HEAPS" | grep -q "system"; then
    pass "/dev/dma_heap/ 可用 (heaps: $DMA_HEAPS)"
else
    fail "/dev/dma_heap/ 不可用 (DMA-BUF 零拷贝需要 dma-heap 支持)"
fi

# ══════════════════════════════════════════════════════════════════════════════
# 6. 摄像头设备
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "── 6. 摄像头设备 ──"
CAM_DEV="/dev/video-camera0"
if [ -c "$CAM_DEV" ]; then
    # 尝试查询分辨率
    if command -v v4l2-ctl &>/dev/null; then
        CAM_FMT=$(v4l2-ctl -d "$CAM_DEV" --get-fmt-video 2>/dev/null | grep "Width/Height" || echo "无法读取")
        pass "$CAM_DEV 存在 ($CAM_FMT)"
    else
        pass "$CAM_DEV 设备节点存在 (安装 v4l2-ctl 可获取详细信息)"
    fi
elif [ -c /dev/video0 ]; then
    warn "$CAM_DEV 不存在，但 /dev/video0 可用 (可能需要创建软链接)"
elif [ -c /dev/video11 ]; then
    warn "$CAM_DEV 不存在，但 /dev/video11 可用 (可能是 MIPI CSI 摄像头)"
else
    fail "摄像头设备未找到 (/dev/video-camera0 或 /dev/video0)"
fi

# ══════════════════════════════════════════════════════════════════════════════
# 7. 项目文件完整性
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "── 7. 项目文件完整性 ──"

REQUIRED_FILES=(
    "CMakeLists.txt"
    "main.cpp"
    "rga_utils.h"
    "SafeQueue.h"
    "camera/v4l2_camera.cpp"
    "camera/cma_buffer.cpp"
    "yolo/yolov5s.cpp"
    "yolo/post_process.cpp"
    "mpp/mpp.c"
    "mpp/mpp_encoder.cpp"
    "pool/thread_poll.cpp"
    "streamer/rtsp.cpp"
    "3rdparty/librknn_api/include/rknn_api.h"
    "3rdparty/rga/RK3588/include/rga.h"
    "3rdparty/rga/RK3588/include/im2d.h"
)

MODEL_FILES=(
    "model/best.rknn"
    "model/helmet_labels.txt"
)

for f in "${REQUIRED_FILES[@]}"; do
    if [ -f "$PROJECT_DIR/$f" ]; then
        pass "$f"
    else
        fail "$f 缺失"
    fi
done

echo ""
for f in "${MODEL_FILES[@]}"; do
    if [ -f "$PROJECT_DIR/$f" ]; then
        pass "$f (模型文件)"
    else
        warn "$f 缺失 (模型文件，运行时必须)"
    fi
done

# ══════════════════════════════════════════════════════════════════════════════
# 8. 内存状态
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "── 8. 系统内存 ──"

# CMA 内存
CMA_TOTAL=$(grep -oP 'CmaTotal:\s+\K\d+' /proc/meminfo 2>/dev/null || echo 0)
CMA_FREE=$(grep -oP 'CmaFree:\s+\K\d+' /proc/meminfo 2>/dev/null || echo 0)
CMA_TOTAL_MB=$((CMA_TOTAL / 1024))
CMA_FREE_MB=$((CMA_FREE / 1024))
if [ "$CMA_TOTAL" -gt 0 ]; then
    if [ "$CMA_TOTAL_MB" -ge 16 ]; then
        pass "CMA: ${CMA_TOTAL_MB}MB 总量 / ${CMA_FREE_MB}MB 可用 (充裕)"
    else
        warn "CMA: ${CMA_TOTAL_MB}MB 总量 / ${CMA_FREE_MB}MB 可用 (建议 ≥16MB, 当前偏小)"
    fi
else
    warn "无法读取 CMA 内存状态"
fi

# 总内存
MEM_TOTAL=$(grep -oP 'MemTotal:\s+\K\d+' /proc/meminfo 2>/dev/null || echo 0)
MEM_FREE=$(grep -oP 'MemAvailable:\s+\K\d+' /proc/meminfo 2>/dev/null || echo 0)
MEM_TOTAL_MB=$((MEM_TOTAL / 1024))
MEM_FREE_MB=$((MEM_FREE / 1024))
info "系统内存: ${MEM_TOTAL_MB}MB 总量 / ${MEM_FREE_MB}MB 可用"

# ══════════════════════════════════════════════════════════════════════════════
# 9. NPU 状态 (可选)
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "── 9. NPU 状态 ──"

# 检查 NPU 频率 (需要 debugfs)
NPU_FREQ=""
for path in /sys/kernel/debug/clk/clk_summary /sys/class/devfreq/*/cur_freq; do
    if [ -f "$path" ] && grep -qi "npu\|rknpu" "$path" 2>/dev/null; then
        NPU_FREQ=$(grep -i "npu\|rknpu" "$path" 2>/dev/null | head -1)
        break
    fi
done
if [ -n "$NPU_FREQ" ]; then
    info "NPU 频率信息: $NPU_FREQ"
else
    info "无法读取 NPU 频率 (需要 root/debugfs 挂载)"
fi

# ══════════════════════════════════════════════════════════════════════════════
# 汇总
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  检查完毕                                                    ║"
printf "║  ${GREEN}通过: %2d${NC}  |  ${RED}失败: %2d${NC}  |  ${YELLOW}警告: %2d${NC}                             ║\n" "$PASS" "$FAIL" "$WARN"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

if [ "$FAIL" -gt 0 ]; then
    echo -e "${RED}存在必须修复的问题。请根据上面的 [✗] 提示逐一处理。${NC}"
    echo ""
    echo "常见修复命令:"
    echo "  sudo apt update"
    echo "  sudo apt install -y g++ cmake make pkg-config"
    echo "  sudo apt install -y libopencv-dev"
    echo "  sudo apt install -y libavformat-dev libavcodec-dev libavutil-dev"
    echo "  sudo apt install -y v4l-utils"
    echo ""
    exit 1
elif [ "$WARN" -gt 0 ]; then
    echo -e "${YELLOW}有警告项，建议处理后再编译运行。${NC}"
    echo ""
else
    echo -e "${GREEN}环境检查全部通过！可以编译运行:${NC}"
    echo "  cd build && cmake .. && make -j\$(nproc)"
    echo "  ./app --no-rtmp          # 无推流测试"
    echo "  ./app --bench-rga         # RGA vs OpenCV 对比"
    echo "  ./app                     # 完整推流"
    echo ""
fi
