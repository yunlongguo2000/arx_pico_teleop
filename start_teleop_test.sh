#!/bin/bash
################################################################################
# ARX 纯遥操测试启动器 (无相机、无录制)
#
# 自动启动两个终端:
#   终端1: CAN 初始化 + R5 双臂控制器 + RPC 服务端
#   终端2: VR 遥操测试 (纯控制循环, 无相机/无数据集)
#
# 使用方法:
#   ./start_teleop_test.sh
#
# 停止: 按 VR X 按钮 或 Ctrl+C
################################################################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
ARX_WORKSPACE="${ARX_WORKSPACE:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
ARX_ROS2_WS="${ARX_ROS2_WS:-$ARX_WORKSPACE/ros2_ws}"
RPC_BRIDGE_DIR="$ARX_WORKSPACE/services/rpc"

LOG_DIR="$PROJECT_ROOT/.log"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
CTRL_LOG="$LOG_DIR/teleop_ctrl_${TIMESTAMP}.log"
TELEOP_LOG="$LOG_DIR/teleop_test_${TIMESTAMP}.log"

log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[OK]${NC}   $1"; }
log_error()   { echo -e "${RED}[ERR]${NC}  $1"; }

################################################################################
# 前提条件检查
################################################################################

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║     ARX VR 纯遥操测试 (无相机, 无录制)                ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# gnome-terminal
if ! which gnome-terminal >/dev/null 2>&1; then
    log_error "未找到 gnome-terminal"
    exit 1
fi

# ROS 2
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || { log_error "未找到 ROS 2 Jazzy"; exit 1; }
fi
log_success "ROS 2: $ROS_DISTRO"

# Workspace
if [ ! -f "$ARX_ROS2_WS/install/setup.bash" ]; then
    log_error "ROS 2 工作空间未编译: $ARX_ROS2_WS"
    exit 1
fi
log_success "工作空间: $ARX_ROS2_WS"

# conda
if ! conda info --envs 2>/dev/null | grep -q "ur_data"; then
    log_error "ur_data conda 环境不存在"
    exit 1
fi
log_success "conda: ur_data"

# Log dir
mkdir -p "$LOG_DIR"

echo ""
log_info "终端1: CAN 初始化 + R5 控制器 + RPC 服务端"
log_info "终端2: VR 遥操测试循环"
echo ""
log_info "日志: $CTRL_LOG"
log_info "日志: $TELEOP_LOG"
echo ""

################################################################################
# 终端1: 控制器 + RPC 服务端
################################################################################

gnome-terminal --tab --title="ARX-Controllers" -- bash -c "
    echo '=== 终端1: 控制器 + RPC 服务端 ===' | tee '$CTRL_LOG'
    source /opt/ros/jazzy/setup.bash 2>&1 | tee -a '$CTRL_LOG'
    source $ARX_ROS2_WS/install/setup.bash 2>&1 | tee -a '$CTRL_LOG'
    cd $PROJECT_ROOT

    # CAN 初始化
    source $PROJECT_ROOT/scripts/can_ensure.sh
    CAN_OK=true
    ensure_can_interface /dev/arxcan1 can1 '左臂' 2>&1 | tee -a '$CTRL_LOG' || CAN_OK=false
    ensure_can_interface /dev/arxcan3 can3 '右臂' 2>&1 | tee -a '$CTRL_LOG' || CAN_OK=false
    if [ \"\$CAN_OK\" = false ]; then
        echo '❌ CAN 接口初始化失败' | tee -a '$CTRL_LOG'
        read -p '按Enter键关闭终端'
        exit 1
    fi
    echo '✅ CAN 接口就绪' | tee -a '$CTRL_LOG'

    # R5 双臂控制器
    if ! ros2 node list 2>/dev/null | grep -q -E '/arm_l|/arm_r'; then
        echo '启动 R5 双臂控制器...' | tee -a '$CTRL_LOG'
        ros2 launch arx_r5_controller open_double_arm_normal.launch.py >/dev/null 2>&1 &
        sleep 5
        echo '✅ 双臂控制器已启动' | tee -a '$CTRL_LOG'
    else
        echo '⚠️  双臂控制器已在运行' | tee -a '$CTRL_LOG'
    fi

    # RPC 服务端
    if ! ss -tuln 2>/dev/null | grep -q ':4242 '; then
        echo '启动 RPC 服务端 (arms-only)...' | tee -a '$CTRL_LOG'
        cd $RPC_BRIDGE_DIR
        python3 -m services.rpc.server --arms-only 2>&1 | tee -a '$CTRL_LOG'
    else
        echo '⚠️  RPC 服务端已在运行' | tee -a '$CTRL_LOG'
        read -p '按Enter键关闭终端'
    fi
" &

# 等控制器启动
log_info "等待控制器启动 (8s)..."
sleep 8

################################################################################
# 终端2: VR 遥操测试
################################################################################

gnome-terminal --tab --title="ARX-TeleopTest" -- bash -c "
    echo '=== 终端2: VR 纯遥操测试 ===' | tee '$TELEOP_LOG'

    cd $PROJECT_ROOT
    export PYTHONPATH=\"$PROJECT_ROOT:\$PYTHONPATH\"

    conda run -n ur_data --no-capture-output bash -c \"
        source /opt/ros/jazzy/setup.bash
        source $ARX_ROS2_WS/install/setup.bash
        cd $PROJECT_ROOT
        export PYTHONPATH=\\\"$PROJECT_ROOT:\\\$PYTHONPATH\\\"
        python3 scripts/core/run_teleop_test.py
    \" 2>&1 | tee -a '$TELEOP_LOG'

    read -p '按Enter键关闭终端'
" &

log_success "两个终端窗口已打开"
echo ""
echo "操作说明:"
echo "  握住 Grip 按钮 = 激活手臂 IK 控制"
echo "  扣扳机 = 夹爪开合"
echo "  X 按钮 或 Ctrl+C = 退出"
