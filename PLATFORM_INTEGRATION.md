# ARX 平台集成

安装 ARX_new/packages/arx_client 和 ARX_new 平台工具后再安装本包。机器人抽象使用 arx_client；可选标定使用 arx_platform.calibration，不再通过固定父目录导入平台模块。

完整平台目录可由 ARX_WORKSPACE 指定。旧 SDK 测试从 ARX_SDK_ROOT 或 ARX_EXTERNAL_ROOT/python_sdk 加载固定快照恢复的厂商 SDK。Pico 和 Quest 的既有通用 Python 包名可能冲突，当前使用分离环境。

SDK 测试前运行 `arx-restore python-sdk --destination "${ARX_EXTERNAL_ROOT:-$HOME/.cache/arx}/python_sdk"`；
若使用自定义位置，将 `ARX_SDK_ROOT` 指向恢复出的整个 SDK 根目录。
