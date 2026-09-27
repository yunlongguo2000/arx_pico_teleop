#!/bin/bash
################################################################################
# 仅在「控制电脑」上启动回放：通过 TCP 连接「机器人上位机」上已运行的 RPC 服务端
# （机器人端需已启动：双臂控制器 + server.py，默认监听 tcp://*:4242）
#
# 本机需要：conda 环境 ur_data，不需要本机 CAN/双臂 ROS 节点。
# 数据集从 huggingface 缓存读取 (~/.cache/huggingface/lerobot/)。
#
# 用法:
#   export ARX_RPC_HOST=10.10.10.2   # 可选，也可作为第一个参数传入
#   bash start_replay_remote_rpc.sh [机器人IP] [episode_idx] [RPC端口]
#
# 示例:
#   bash start_replay_remote_rpc.sh 10.10.10.2 0
#   bash start_replay_remote_rpc.sh 10.10.10.2 5 4242
#   ARX_RPC_HOST=10.10.10.2 ARX_RPC_PORT=4242 bash start_replay_remote_rpc.sh 0
#
# 配置:
#   数据集和episode相关配置来自 scripts/config/cfg_arx.yaml
#   若指定了命令行的 episode_idx，会覆盖配置文件中的设置。
#
# 网络: 机器人端防火墙需放行 RPC 端口（默认 4242/TCP）。
################################################################################

# set -e  # Keep error output visible

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Get script directory (where this file is located)
# This script is at PROJECT_ROOT, so SCRIPT_DIR = PROJECT_ROOT
SCRIPT_DIR=$(cd "$(dirname "$BASH_SOURCE")" && pwd)
PROJECT_ROOT="$SCRIPT_DIR"
ARX_WORKSPACE="${ARX_WORKSPACE:-/home/arx/ARX_new}"
CONFIG_FILE="$PROJECT_ROOT/scripts/config/cfg_arx.yaml"

# Debug: print paths
# echo "DEBUG: PROJECT_ROOT = $PROJECT_ROOT"
# echo "DEBUG: CONFIG_FILE = $CONFIG_FILE"
# echo "DEBUG: python path will be $PROJECT_ROOT/scripts/core/run_replay.py"

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_ok() { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[⚠]${NC} $1"; }
log_err() { echo -e "${RED}[✗]${NC} $1"; }

usage() {
    echo "用法:"
    echo "  $0 <机器人IP或主机名> [episode_idx] [RPC端口]"
    echo "  或: export ARX_RPC_HOST=<机器人> [ARX_RPC_PORT=4242] 后"
    echo "      $0 [episode_idx] [RPC端口]"
    echo ""
    echo "  episode_idx - 要回放的episode索引 (默认使用配置文件中的设置)"
    echo "  默认端口: 4242"
}

# ---------- 解析参数 ----------
if [ -z "${ARX_RPC_HOST:-}" ]; then
    if [ $# -lt 1 ]; then
        log_err "未设置 ARX_RPC_HOST，且未提供机器人地址"
        usage
        exit 1
    fi
    export ARX_RPC_HOST="$1"
    if [[ "$ARX_RPC_HOST" =~ ^[0-9]+$ ]]; then
        log_err "第一个参数应为机器人 IP 或主机名，不能是 episode 索引"
        usage
        exit 1
    fi
    shift
fi

EPISODE_IDX="$1"
if [ -n "${EPISODE_IDX:-}" ] && [[ "$EPISODE_IDX" =~ ^[0-9]+$ ]]; then
    shift || true
else
    EPISODE_IDX=""  # use config file default
fi

if [ -n "${1:-}" ]; then
    export ARX_RPC_PORT="$1"
else
    export ARX_RPC_PORT="${ARX_RPC_PORT:-4242}"
fi

# ---------- 检查环境 ----------
log_info "ARX RPC Host: $ARX_RPC_HOST"
log_info "ARX RPC Port: $ARX_RPC_PORT"
if [ -n "$EPISODE_IDX" ]; then
    log_info "Episode index: $EPISODE_IDX (overriding config file)"
else
    log_info "Episode index: using default from config file"
fi
echo ""

# 检查conda环境
if ! python -c "import lerobot" >/dev/null 2>&1; then
    log_err "请先激活 conda 环境: conda activate ur_data"
    exit 1
fi
log_ok "Conda环境检查通过 (lerobot已安装)"

# 检查配置文件
if [ ! -f "$CONFIG_FILE" ]; then
    log_err "配置文件不存在: $CONFIG_FILE"
    exit 1
fi
log_ok "配置文件: $CONFIG_FILE"

# ---------- 启动回放 ----------
echo ""
log_info "启动回放..."
echo "────────────────────────────────────────────────────────────"

cd "$PROJECT_ROOT/scripts/core"

# 如果命令行指定了episode_idx，通过环境变量传给python
if [ -n "$EPISODE_IDX" ]; then
    export REPLAY_EPISODE_INDEX="$EPISODE_IDX"
fi

python run_replay.py
