#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="${ARX_SDK_ROOT:-${ARX_EXTERNAL_ROOT:-$HOME/.cache/arx}/python_sdk}"
ARX_SDK_DIR="$SDK_ROOT/arx_r5_sdk"
if [ ! -d "$ARX_SDK_DIR" ]; then
    echo "SDK absent: $ARX_SDK_DIR"
    printf 'Run arx-restore python-sdk --destination "%s" in the ARX workspace.\n' "$SDK_ROOT"
    exit 1
fi
export ARX_SDK_ROOT="$SDK_ROOT"
export LD_LIBRARY_PATH="$ARX_SDK_DIR/bimanual/api/arx_r5_src:$ARX_SDK_DIR/bimanual/api:$ARX_SDK_DIR/bimanual/api/arx_r5_python:/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"
exec python3 "$SCRIPT_DIR/test_arx_modules.py"
