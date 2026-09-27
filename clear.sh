#!/bin/bash
# 清理所有 ARX 机器人相关后台进程和端口
#
# 覆盖范围:
#   - RPC 服务端 (server.py)
#   - 数据采集 (run_record_arx.py)
#   - R5 双臂控制器 (R5Controller / launch)
#   - LIFT 底盘控制器
#   - 端口 4242 (ZMQ RPC)

set -u

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}清理 ARX 机器人相关进程...${NC}"

# 按依赖顺序: 先上层应用, 再底层控制器
TARGETS=(
    "server.py"
    "run_record_arx.py"
    "R5Controller"
    "open_double_arm"
    "arx_lift_controller"
    "lift_controller"
)

killed=0
for pat in "${TARGETS[@]}"; do
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -v "$$" || true)
    if [ -n "$pids" ]; then
        kill $pids 2>/dev/null
        echo -e "${GREEN}  [killed]${NC} $pat ($(echo $pids | tr '\n' ' '))"
        ((killed++))
    fi
done

if [ "$killed" -eq 0 ]; then
    echo "  没有运行中的进程"
fi

# 等待进程退出, 强杀残留
sleep 1
for pat in "${TARGETS[@]}"; do
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -v "$$" || true)
    if [ -n "$pids" ]; then
        kill -9 $pids 2>/dev/null
        echo -e "${YELLOW}  [force]${NC} $pat"
    fi
done

# 释放 RPC 端口
if lsof -Pi :4242 -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo -e "${YELLOW}端口 4242 仍被占用，强制释放...${NC}"
    fuser -k 4242/tcp 2>/dev/null
    sleep 1
fi

if lsof -Pi :4242 -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo -e "${RED}[FAIL] 端口 4242 未释放，请手动检查: lsof -i :4242${NC}"
else
    echo -e "${GREEN}端口 4242 OK${NC}"
fi

echo -e "${GREEN}清理完成${NC}"
