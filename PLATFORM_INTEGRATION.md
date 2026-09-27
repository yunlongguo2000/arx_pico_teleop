# ARX 平台集成

安装 ARX_new/packages/arx_client 和 ARX_new 平台工具后再安装本包。机器人抽象使用 arx_client；可选标定使用 arx_platform.calibration，不再通过固定父目录导入平台模块。

完整平台目录可由 ARX_WORKSPACE 指定。旧 SDK 测试仍依赖平台 legacy/python_sdk。Pico 和 Quest 的既有通用 Python 包名可能冲突，当前使用分离环境。
