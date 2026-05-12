#!/bin/bash
################################################################################
# ARX LIFT2 VR 遥操作全流程录制启动脚本
# 自动分两个终端启动，输出保存到.log文件夹：
#  终端1 (系统Python 3.12): 启动LIFT控制器 + 双臂控制器 + RPC服务端
#  终端2 (ur_data conda 3.10): 启动数据采集程序
#
# 使用方法:
#   bash start_recording.sh [debug|record]
#
# 模式说明:
#   debug  - 调试模式 (不发送动作，只读取状态)
#   record - 正式录制
#
################################################################################

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
ARX_WORKSPACE="$(cd "$SCRIPT_DIR/../.." && pwd)"
ZERORPC_DIR="$ARX_WORKSPACE/ros2_bridge"

# 录制模式 (默认: debug)
RUN_MODE="${1:-debug}"
CONFIG_FILE="$PROJECT_ROOT/scripts/config/cfg_arx.yaml"

# 日志目录和日志文件名（带时间戳）
LOG_DIR="$PROJECT_ROOT/.log"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
CONTROLLERS_LOG="$LOG_DIR/controllers_${TIMESTAMP}.log"
DATACOL_LOG="$LOG_DIR/data_collection_${TIMESTAMP}.log"

# 验证模式
if [[ ! "$RUN_MODE" =~ ^(debug|record)$ ]]; then
    echo -e "${RED}错误: 无效的运行模式 '$RUN_MODE'${NC}"
    echo "用法: $0 [debug|record]"
    echo ""
    echo "  debug  - 调试模式 (不发送动作)"
    echo "  record - 正式录制"
    exit 1
fi

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[⚠]${NC} $1"
}

log_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# 清理函数
cleanup() {
    log_info "清理资源..."

    # 查找所有与ARX相关的进程
    for PID in $(pgrep -f "arx_ros2_rpc_server.py\|arx_lift_controller\|arx_r5_controller"); do
        log_info "停止进程 (PID: $PID)..."
        kill $PID 2>/dev/null || true
        sleep 0.2
        if ps -p $PID > /dev/null 2>&1; then
            kill -9 $PID 2>/dev/null || true
        fi
        wait $PID 2>/dev/null || true
    done

    log_success "清理完成"
}

# 注册清理函数
trap cleanup EXIT INT TERM

################################################################################
# 显示运行模式信息
################################################################################

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║      ARX LIFT2 VR 遥操作数据采集启动器                       ║"
echo "║    Auto-Split-Terminal: Controllers + Data Collection         ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

case "$RUN_MODE" in
    debug)
        log_info "运行模式: 调试模式 (不发送动作)"
        ;;
    record)
        log_info "运行模式: 正式录制"
        log_warning "注意: 此模式会移动机器人!"
        ;;
esac
echo ""

################################################################################
# 检查前提条件
################################################################################

log_info "检查前提条件..."

# 检查终端工具是否存在
if ! which gnome-terminal >/dev/null 2>&1; then
    log_error "未找到gnome-terminal，请安装它"
    log_info "请运行: sudo apt install gnome-terminal"
    exit 1
fi
log_success "终端工具: gnome-terminal"

# 检查ROS 2 环境
if [ -z "$ROS_DISTRO" ]; then
    log_error "ROS 2 环境未配置"
    log_info "请运行: source /opt/ros/jazzy/setup.bash"
    exit 1
fi
log_success "ROS 2 环境: $ROS_DISTRO"

# 检查工作空间
if [ ! -f "$ARX_WORKSPACE/ros2_ws/install/setup.bash" ]; then
    log_error "ROS 2 工作空间未编译"
    log_info "请运行: cd $ARX_WORKSPACE/ros2_ws && colcon build"
    exit 1
fi
log_success "工作空间已编译: $ARX_WORKSPACE"

# 检查配置文件
if [ ! -f "$CONFIG_FILE" ]; then
    log_error "配置文件未找到"
    log_info "期望路径: $CONFIG_FILE"
    exit 1
fi
log_success "配置文件已找到"

# 检查URDF文件
URDF_PATH=$(grep 'robot_urdf_path:' "$CONFIG_FILE" | head -1 | awk '{print $2}' | tr -d '"')
if [ ! -f "$PROJECT_ROOT/$URDF_PATH" ]; then
    log_error "URDF文件未找到"
    log_info "期望路径: $PROJECT_ROOT/$URDF_PATH"
    exit 1
fi
log_success "URDF文件已找到: $URDF_PATH"

# 检查conda环境
if ! conda info --envs 2>/dev/null | grep -q "ur_data"; then
    log_error "ur_data conda环境不存在"
    log_info "请运行: conda create -n ur_data python=3.10"
    exit 1
fi
log_success "conda环境: ur_data (Python 3.10)"

# 获取conda环境里的Python路径
UR_DATA_PYTHON=$(conda run -n ur_data which python3)
log_success "ur_data Python: $UR_DATA_PYTHON"

# 创建日志目录
if [ ! -d "$LOG_DIR" ]; then
    mkdir -p "$LOG_DIR"
    log_success "日志目录已创建: $LOG_DIR"
else
    log_info "日志目录已存在: $LOG_DIR"
fi

################################################################################
# 验证 debug 配置与运行模式是否匹配
################################################################################

log_info "验证 debug 配置..."

# 用 Python 解析 YAML 文件的 debug 值（支持锚点引用）
DEBUG_VALUE=$(python3 - <<END
import yaml
with open("$CONFIG_FILE", 'r', encoding='utf-8') as f:
    config = yaml.safe_load(f)
print(config.get('record', {}).get('debug', False))
END
)

# 根据运行模式检查配置
if [ "$RUN_MODE" = "debug" ]; then
    if [ "$DEBUG_VALUE" != "True" ] && [ "$DEBUG_VALUE" != "true" ]; then
        log_error "配置不匹配！"
        echo ""
        echo "  当前运行模式: ${YELLOW}debug${NC} (不发送动作)"
        echo "  配置文件中的 debug: ${RED}$DEBUG_VALUE${NC}"
        echo ""
        echo "  请修改 $CONFIG_FILE:"
        echo "    设置 ${GREEN}debug: true${NC}"
        exit 1
    fi
    log_success "配置验证通过: debug=true (调试模式)"
else
    if [ "$DEBUG_VALUE" = "True" ] || [ "$DEBUG_VALUE" = "true" ]; then
        log_error "配置不匹配！"
        echo ""
        echo "  当前运行模式: ${YELLOW}record${NC} (正式录制)"
        echo "  配置文件中的 debug: ${RED}$DEBUG_VALUE${NC}"
        echo ""
        echo "  请修改 $CONFIG_FILE:"
        echo "    设置 ${GREEN}debug: false${NC}"
        exit 1
    fi
    log_success "配置验证通过: debug=False (录制模式)"
fi

################################################################################
# 自动分终端启动
################################################################################

log_info "准备启动分终端程序..."
echo ""
echo "将启动两个终端窗口:"
echo "  终端1: 系统Python 3.12 - 启动机器人控制器 + RPC服务端"
echo "  终端2: ur_data conda 3.10 - 启动数据采集程序"
echo ""
echo "日志文件位置:"
echo "  控制器日志: ${BLUE}$CONTROLLERS_LOG${NC}"
echo "  数据采集日志: ${BLUE}$DATACOL_LOG${NC}"
echo ""

# 等待用户确认
read -p "是否继续？(y/N): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    log_info "操作取消"
    exit 0
fi

# 1. 在新终端启动机器人控制器和RPC服务端 (系统Python 3.12)
log_info "启动终端1: 机器人控制器 + RPC服务端"
gnome-terminal --tab --title="ARX-Controllers" -- bash -c "
    echo '=== 终端1: 机器人控制器 + RPC服务端 ===' | tee '$CONTROLLERS_LOG'
    echo '日志文件: $CONTROLLERS_LOG' | tee -a '$CONTROLLERS_LOG'
    source /opt/ros/jazzy/setup.bash 2>&1 | tee -a '$CONTROLLERS_LOG'
    source $ARX_WORKSPACE/ros2_ws/install/setup.bash 2>&1 | tee -a '$CONTROLLERS_LOG'
    cd $PROJECT_ROOT
    echo '✅ ROS 2 环境已加载' | tee -a '$CONTROLLERS_LOG'

    # 检查LIFT控制器是否已运行
    if ! ros2 node list 2>/dev/null | grep -q '/lift'; then
        echo '启动 LIFT 控制器...' | tee -a '$CONTROLLERS_LOG'
        ros2 run arx_lift_controller lift_controller >/dev/null 2>&1 &
        LIFT_PID=\$!
        sleep 3
        echo '✅ LIFT 控制器已启动 (PID: \$LIFT_PID)' | tee -a '$CONTROLLERS_LOG'
    else
        echo '⚠️  LIFT 控制器已在运行' | tee -a '$CONTROLLERS_LOG'
    fi

    # 检查双臂控制器是否已运行
    if ! ros2 node list 2>/dev/null | grep -q -E '/arm_l|/arm_r'; then
        echo '启动 双臂控制器...' | tee -a '$CONTROLLERS_LOG'
        ros2 launch arx_r5_controller open_double_arm.launch.py >/dev/null 2>&1 &
        ARMS_PID=\$!
        sleep 5
        echo '✅ 双臂控制器已启动 (PID: \$ARMS_PID)' | tee -a '$CONTROLLERS_LOG'
    else
        echo '⚠️  双臂控制器已在运行' | tee -a '$CONTROLLERS_LOG'
    fi

    # 检查RPC服务端是否已运行
    if ! ss -tuln 2>/dev/null | grep -q ':4242 '; then
        echo '启动 RPC 服务端...' | tee -a '$CONTROLLERS_LOG'
        cd $ZERORPC_DIR
        # 同时输出到终端和日志文件
        python3 arx_ros2_rpc_server.py 2>&1 | tee -a '$CONTROLLERS_LOG'
    else
        echo '⚠️  RPC 服务端已在运行' | tee -a '$CONTROLLERS_LOG'
        read -p '按Enter键关闭终端'
    fi
" &

# 等待控制器启动
log_info "等待控制器启动..."
sleep 3

# 2. 在新终端启动数据采集程序 (ur_data conda 3.10)
log_info "启动终端2: 数据采集程序"
gnome-terminal --tab --title="ARX-DataCollection" -- bash -c "
    echo '=== 终端2: 数据采集程序 ===' | tee '$DATACOL_LOG'
    echo '日志文件: $DATACOL_LOG' | tee -a '$DATACOL_LOG'
    echo '使用ur_data环境的Python: $UR_DATA_PYTHON' | tee -a '$DATACOL_LOG'

    # 先验证conda环境里的Python是否能导入lerobot
    echo '检查conda环境里的lerobot...' | tee -a '$DATACOL_LOG'
    if ! conda run -n data_collection python3 -c 'import lerobot; print(\"✅ lerobot已安装:\", lerobot.__file__)' 2>&1 | tee -a '$DATACOL_LOG'; then
        echo '❌ data_collection环境里没有安装lerobot！' | tee -a '$DATACOL_LOG'
        echo '请在激活data_collection环境后运行: pip install lerobot' | tee -a '$DATACOL_LOG'
        read -p '按Enter键关闭终端'
        exit 1
    fi

    # 检查其他依赖
    for MOD in numpy scipy yaml xrobotoolkit_teleop; do
        if ! conda run -n data_collection python3 -c \"import \$MOD\" 2>/dev/null; then
            echo '❌ 模块' \$MOD '未安装' | tee -a '$DATACOL_LOG'
            read -p '按Enter键关闭终端'
            exit 1
        fi
    done
    echo '✅ 所有 Python 依赖已安装' | tee -a '$DATACOL_LOG'

    # 用conda run直接运行，确保在data_collection环境里
    cd $PROJECT_ROOT
    export PYTHONPATH=\"$PROJECT_ROOT:\$PYTHONPATH\"
    echo '✅ 准备运行数据采集程序' | tee -a '$DATACOL_LOG'
    echo '' | tee -a '$DATACOL_LOG'

    # 用conda run -n data_collection运行，确保在正确的环境里
    conda run -n data_collection --no-capture-output bash -c \"
        source /opt/ros/jazzy/setup.bash
        source $ARX_WORKSPACE/ros2_ws/install/setup.bash
        cd $PROJECT_ROOT
        export PYTHONPATH=\\\"$PROJECT_ROOT:\\\$PYTHONPATH\\\"
        python3 scripts/core/run_record_arx.py
    \" 2>&1 | tee -a '$DATACOL_LOG'

    read -p '按Enter键关闭终端'
" &

log_success "分终端启动完成！"
echo "两个终端窗口已打开，请检查它们的输出"
echo ""
echo "日志文件位置:"
echo "  控制器日志: ${BLUE}$CONTROLLERS_LOG${NC}"
echo "  数据采集日志: ${BLUE}$DATACOL_LOG${NC}"
