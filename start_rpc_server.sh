#!/bin/bash
################################################################################
# ARX R5 双臂 RPC 服务端启动脚本
#
# 在机器人上位机上运行，启动双臂控制器和 RPC 服务端。
# 远程工作站通过 RPC 客户端 (arx_ros2_rpc_client.py) 连接。
#
# 启动内容:
#   1. R5 双臂控制器
#   2. ZeroRPC 服务端 (tcp://0.0.0.0:4242)
#
# 使用方法:
#   ./start_rpc_server.sh
#
# 停止: Ctrl+C (自动清理所有子进程)
#
# 远程连接示例 (工作站侧):
#   from ros2_bridge.arx_ros2_rpc_client import ArxROS2RPCClient
#   client = ArxROS2RPCClient(ip="<robot_ip>", port=4242)
#   client.system_connect()
#   state = client.get_full_state()
#
################################################################################

set -e

# ── 颜色 ──
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ── 路径 ──
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARX_WORKSPACE="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS2_BRIDGE_DIR="$ARX_WORKSPACE/ros2_bridge"

# ── 日志 ──
log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[OK]${NC}   $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERR]${NC}  $1"; }

# ── 子进程 PID 跟踪 ──
CHILD_PIDS=()

cleanup() {
    echo ""
    log_info "正在停止所有子进程..."

    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done

    sleep 1

    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    # 确保端口释放
    if lsof -Pi :4242 -sTCP:LISTEN -t >/dev/null 2>&1; then
        fuser -k 4242/tcp 2>/dev/null || true
    fi

    log_success "清理完成"
}

trap cleanup EXIT INT TERM

# ── 显示信息 ──
echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║          ARX R5 双臂 RPC Server Launcher                ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
log_info "启动组件: R5 双臂控制器 + RPC 服务端"
log_info "RPC 地址: tcp://0.0.0.0:4242"
echo ""

# ── 检查前提条件 ──

# ROS 2 环境
if [ -z "$ROS_DISTRO" ]; then
    log_info "加载 ROS 2 环境..."
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    else
        log_error "未找到 ROS 2 Jazzy，请先安装"
        exit 1
    fi
fi
log_success "ROS 2: $ROS_DISTRO"

# 工作空间
if [ ! -f "$ARX_WORKSPACE/ros2_ws/install/setup.bash" ]; then
    log_error "ROS 2 工作空间未编译: $ARX_WORKSPACE/ros2_ws"
    log_info "请运行: cd $ARX_WORKSPACE/ros2_ws && colcon build"
    exit 1
fi
source "$ARX_WORKSPACE/ros2_ws/install/setup.bash"
log_success "工作空间: $ARX_WORKSPACE/ros2_ws"

# ZeroRPC 依赖
if ! python3 -c "import zerorpc" >/dev/null 2>&1; then
    log_error "缺少 Python 模块: zerorpc"
    log_info "请安装: pip install zerorpc gevent"
    exit 1
fi

if ! python3 -c "import gevent" >/dev/null 2>&1; then
    log_error "缺少 Python 模块: gevent"
    log_info "请安装: pip install gevent"
    exit 1
fi
log_success "ZeroRPC 依赖检查通过 (zerorpc/gevent)"

# RPC 服务端脚本
if [ ! -f "$ROS2_BRIDGE_DIR/arx_ros2_rpc_server.py" ]; then
    log_error "未找到 RPC 服务端: $ROS2_BRIDGE_DIR/arx_ros2_rpc_server.py"
    exit 1
fi

# 端口占用检查
if ss -tuln 2>/dev/null | grep -q ':4242 '; then
    log_warn "端口 4242 已被占用"
    log_info "可能 RPC 服务端已在运行，或运行 stop_all.sh --rpc 释放端口"
    exit 1
fi

log_success "前提条件检查通过"
echo ""

# ── 检查 / 初始化 CAN 接口 ──

source "$SCRIPT_DIR/scripts/can_ensure.sh"

CAN_OK=true
ensure_can_interface /dev/arxcan1 can1 "左臂" || CAN_OK=false
ensure_can_interface /dev/arxcan3 can3 "右臂" || CAN_OK=false

if [ "$CAN_OK" = false ]; then
    log_error "CAN 接口初始化失败，无法启动双臂控制器"
    log_info "请检查 USB CAN 适配器是否连接"
    exit 1
fi
echo ""

# ── 启动双臂控制器 ──

if ros2 node list 2>/dev/null | grep -qE '/arm_l|/arm_r'; then
    log_warn "双臂控制器已在运行，跳过"
else
    log_info "启动 R5 双臂控制器..."
    ros2 launch arx_r5_controller open_double_arm_normal.launch.py >/dev/null 2>&1 &
    CHILD_PIDS+=($!)
    sleep 5
    log_success "双臂控制器已启动 (PID: ${CHILD_PIDS[-1]})"
fi

# ── 启动 RPC 服务端 (前台运行) ──

echo ""
log_info "启动 RPC 服务端 (前台运行, Ctrl+C 退出)..."
log_info "等待远程客户端连接 tcp://0.0.0.0:4242 ..."
echo ""
echo "────────────────────────────────────────────────────────────"

cd "$ROS2_BRIDGE_DIR"
python3 arx_ros2_rpc_server.py --arms-only
