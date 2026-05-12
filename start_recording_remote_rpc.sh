#!/bin/bash
################################################################################
# 仅在「数采电脑」上启动数据采集：通过 TCP 连接「控制电脑」上已运行的 RPC 服务端
# （控制端需已启动：双臂控制器 + arx_ros2_rpc_server.py，默认监听 tcp://*:4242）
#
# 本机需要：conda 环境 data_collection、RealSense 相机、VR/遥操作相关依赖；不需要本机 CAN/双臂 ROS 节点。
#
# 用法:
#   export ARX_RPC_HOST=192.168.x.x   # 可选，也可作为第一个参数传入
#   bash start_recording_remote_rpc.sh [控制机地址] [debug|record] [RPC端口]
#
# 示例:
#   bash start_recording_remote_rpc.sh 192.168.1.100 record
#   bash start_recording_remote_rpc.sh 192.168.1.100 debug 4242
#   ARX_RPC_HOST=10.0.0.5 ARX_RPC_PORT=4242 bash start_recording_remote_rpc.sh record
#
# 网络: 控制机防火墙需放行 RPC 端口（默认 4242/TCP），且 ZeroRPC 服务端需绑定为 0.0.0.0（见 ros2_bridge 服务端）。
################################################################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
# 与 start_recording.sh 一致；若你的工程不在默认路径，请 export ARX_WORKSPACE
ARX_WORKSPACE="${ARX_WORKSPACE:-/home/arx/ARX_new}"
ARX_ROS2_WS="${ARX_ROS2_WS:-$ARX_WORKSPACE/ros2_ws}"
CONFIG_FILE="$PROJECT_ROOT/scripts/config/cfg_arx.yaml"
LOG_DIR="$PROJECT_ROOT/.log"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
DATACOL_LOG="$LOG_DIR/data_collection_remote_${TIMESTAMP}.log"

# xrobotoolkit_sdk.so 的 RUNPATH 硬编码到 ARX_new/.../dependencies/.../lib，
# 但该构建产物目录通常不在仓库内。按候选顺序补上 LD_LIBRARY_PATH，
# 让动态链接器能找到 libPXREARobotSDK.so（可用 XR_SDK_LIB_DIR 覆盖）。
if [ -z "${XR_SDK_LIB_DIR:-}" ]; then
    for _candidate in \
        "$PROJECT_ROOT/xrobotoolkit_teleop/dependencies/XRoboToolkit-PC-Service-Pybind/lib" \
        "/opt/apps/roboticsservice/SDK/x64"; do
        if [ -f "$_candidate/libPXREARobotSDK.so" ]; then
            XR_SDK_LIB_DIR="$(cd "$_candidate" && pwd)"
            break
        fi
    done
fi
if [ -n "${XR_SDK_LIB_DIR:-}" ]; then
    export LD_LIBRARY_PATH="$XR_SDK_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_ok() { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[⚠]${NC} $1"; }
log_err() { echo -e "${RED}[✗]${NC} $1"; }

usage() {
    echo "用法:"
    echo "  $0 <控制机IP或主机名> [debug|record] [RPC端口]"
    echo "  或: export ARX_RPC_HOST=<控制机> [ARX_RPC_PORT=4242] 后"
    echo "      $0 [debug|record] [RPC端口]"
    echo ""
    echo "  debug  - 不发送动作（与 cfg 中 debug:true 一致）"
    echo "  record - 正式录制（与 cfg 中 debug:false 一致）"
    echo "  默认端口: 4242"
}

# ---------- 解析参数 ----------
if [ -z "${ARX_RPC_HOST:-}" ]; then
    if [ $# -lt 1 ]; then
        log_err "未设置 ARX_RPC_HOST，且未提供控制机地址"
        usage
        exit 1
    fi
    export ARX_RPC_HOST="$1"
    if [[ "$ARX_RPC_HOST" =~ ^(debug|record)$ ]]; then
        log_err "第一个参数应为控制机 IP 或主机名，不能是 debug/record"
        usage
        exit 1
    fi
    shift
fi

RUN_MODE="${1:-debug}"
if [[ "$RUN_MODE" =~ ^(debug|record)$ ]]; then
    shift || true
else
    RUN_MODE="debug"
fi

if [ -n "${1:-}" ]; then
    export ARX_RPC_PORT="$1"
else
    export ARX_RPC_PORT="${ARX_RPC_PORT:-4242}"
fi

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  ARX 数采（远程 RPC）— 仅本机 run_record_arx                  ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
log_info "控制机 RPC: tcp://${ARX_RPC_HOST}:${ARX_RPC_PORT}"
log_info "运行模式: $RUN_MODE"
echo ""

# ---------- 前提检查 ----------
log_info "检查前提条件..."

if [ ! -f "$CONFIG_FILE" ]; then
    log_err "配置文件不存在: $CONFIG_FILE"
    exit 1
fi
log_ok "配置文件: $CONFIG_FILE"

if ! command -v conda >/dev/null 2>&1; then
    log_err "未找到 conda"
    exit 1
fi
if ! conda info --envs 2>/dev/null | grep -q "data_collection"; then
    log_err "conda 环境 data_collection 不存在"
    exit 1
fi
log_ok "conda 环境: data_collection"

UR_DATA_PYTHON=$(conda run -n data_collection which python3)
log_ok "Python: $UR_DATA_PYTHON"

if [ -n "${XR_SDK_LIB_DIR:-}" ]; then
    log_ok "XR SDK lib: $XR_SDK_LIB_DIR"
else
    log_warn "未找到 libPXREARobotSDK.so，xrobotoolkit_sdk 可能 import 失败；可手动 export XR_SDK_LIB_DIR=<lib 目录>"
fi

# 与 start_recording.sh 一致：校验 cfg 中 debug 与命令行模式一致
DEBUG_VALUE=$(python3 - <<END
import yaml
with open("$CONFIG_FILE", "r", encoding="utf-8") as f:
    config = yaml.safe_load(f)
print(config.get("record", {}).get("debug", False))
END
)

if [ "$RUN_MODE" = "debug" ]; then
    if [ "$DEBUG_VALUE" != "True" ] && [ "$DEBUG_VALUE" != "true" ]; then
        log_err "配置不匹配：当前为 debug 模式，但 $CONFIG_FILE 中 record.debug 不是 true"
        exit 1
    fi
else
    if [ "$DEBUG_VALUE" = "True" ] || [ "$DEBUG_VALUE" = "true" ]; then
        log_err "配置不匹配：当前为 record 模式，但 $CONFIG_FILE 中 record.debug 为 true"
        exit 1
    fi
fi
log_ok "debug 配置与运行模式一致"

# 探测 RPC 端口是否可达（不保证 ZeroRPC 应用层握手，仅 TCP）
if command -v nc >/dev/null 2>&1; then
    if nc -z -w 2 "$ARX_RPC_HOST" "$ARX_RPC_PORT" 2>/dev/null; then
        log_ok "TCP 可达: ${ARX_RPC_HOST}:${ARX_RPC_PORT}"
    else
        log_warn "无法用 nc 连通 ${ARX_RPC_HOST}:${ARX_RPC_PORT}，请检查网络/防火墙/控制端 RPC 是否已启动"
    fi
elif timeout 1 bash -c "echo >/dev/tcp/${ARX_RPC_HOST}/${ARX_RPC_PORT}" 2>/dev/null; then
    log_ok "TCP 可达: ${ARX_RPC_HOST}:${ARX_RPC_PORT}"
else
    log_warn "未安装 nc 且无法探测 /dev/tcp；请自行确认控制端 ${ARX_RPC_PORT}/TCP 已开放"
fi

mkdir -p "$LOG_DIR"
log_info "日志: $DATACOL_LOG"
echo ""

read -p "是否启动数据采集？(y/N): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    log_info "已取消"
    exit 0
fi

# ---------- 依赖（在 tee 之前检查，便于看到彩色日志）----------
if ! conda run -n data_collection python3 -c 'import lerobot' 2>/dev/null; then
    log_err "data_collection 中无法 import lerobot，请先 pip install"
    exit 1
fi

for MOD in numpy scipy yaml xrobotoolkit_teleop; do
    if ! conda run -n data_collection python3 -c "import $MOD" 2>/dev/null; then
        log_err "缺少 Python 模块: $MOD"
        exit 1
    fi
done

for MOD in zerorpc gevent; do
    if ! conda run -n data_collection python3 -c "import $MOD" 2>/dev/null; then
        log_err "缺少 ZeroRPC 依赖模块: $MOD"
        log_info "请在 data_collection 环境安装: conda run -n data_collection pip install zerorpc gevent"
        exit 1
    fi
done

# ---------- 清理残留进程 ----------
_stale=$(pgrep -f "run_record_arx.py" 2>/dev/null || true)
if [ -n "$_stale" ]; then
    log_warn "发现残留 run_record_arx.py 进程 (PID: $_stale)，自动清理..."
    kill $_stale 2>/dev/null || true
    # 等待内核释放 /dev/video* 设备，最多 10s
    for _i in $(seq 1 10); do
        sleep 1
        if ! lsof /dev/video* 2>/dev/null | grep -q "python"; then
            break
        fi
        log_warn "等待设备释放... (${_i}s)"
    done
    log_ok "残留进程已清理"
fi

# ---------- 启动（当前终端前台，同时写入日志）----------
log_info "启动 run_record_arx.py（ARX_RPC_HOST / ARX_RPC_PORT 将传入子进程）..."

cd "$PROJECT_ROOT"
export PYTHONPATH="$PROJECT_ROOT:${PYTHONPATH:-}"

(
    echo "=== 远程 RPC 数采 ==="
    echo "ARX_RPC_HOST=$ARX_RPC_HOST ARX_RPC_PORT=$ARX_RPC_PORT"
    echo "日志: $DATACOL_LOG"
    echo "录制按键说明（仅键盘）:"
    echo "  → : 结束当前阶段并进入下一步"
    echo "  ← : 重录当前 episode（仅录制阶段有效）"
    echo "  Esc : 结束录制并保存"
    echo "  Ctrl+C : 立即退出（进入异常清理）"
    echo "手柄快捷键:"
    echo "  A : 复位右臂（当前episode重录）"
    echo "  X : 复位左臂（当前episode重录）"
    echo ""

    # 数采机若无本地 ROS2 工作空间，可跳过；有则与原版一致供可能的依赖使用
    conda run -n data_collection --no-capture-output bash -c "
        set -e
        if [ -f /opt/ros/jazzy/setup.bash ] && [ -f \"$ARX_ROS2_WS/install/setup.bash\" ]; then
            source /opt/ros/jazzy/setup.bash
            source \"$ARX_ROS2_WS/install/setup.bash\"
        elif [ -f /opt/ros/jazzy/setup.bash ]; then
            source /opt/ros/jazzy/setup.bash
        fi
        cd \"$PROJECT_ROOT\"
        export PYTHONPATH=\"$PROJECT_ROOT:\${PYTHONPATH:-}\"
        export LD_LIBRARY_PATH=\"${LD_LIBRARY_PATH:-}\"
        export ARX_RPC_HOST=\"$ARX_RPC_HOST\"
        export ARX_RPC_PORT=\"$ARX_RPC_PORT\"
        python3 scripts/core/run_record_arx.py
    "
) 2>&1 | tee -a "$DATACOL_LOG"
