from setuptools import setup, find_packages
from pathlib import Path
# ====== Project root ======
ROOT = Path(__file__).parent.resolve()
setup(
    name="arx_pico_teleop",
    version="0.1.0",
    description="Pico 4 VR teleoperation and dataset collection for ARX robots",
    python_requires=">=3.10",
    packages=find_packages(where=".", include=[
        "scripts", "scripts.*",
        "robots", "robots.*",
        "teleoperators", "teleoperators.*",
        "xrobotoolkit_teleop", "xrobotoolkit_teleop.*",
    ]),
    include_package_data=True,
    install_requires=[
    "send2trash",
    "zerorpc",
    "arx-client>=0.1.0,<0.2"
    ],
    scripts=[
        "scripts/tools/map_gripper.sh",
    ],
    entry_points={
        "console_scripts": [
            # === 主要命令 (pico 遥操) ===
            "pico-record = scripts.launcher:launch_remote_recording",
            "pico-replay = scripts.launcher:launch_remote_replay",
            "pico-reset = scripts.launcher:launch_robot_reset",
            "pico-rpc = scripts.launcher:launch_rpc_server",

            # === 通用别名 ===
            "robot-reset = scripts.launcher:launch_robot_reset",

            # === 工具 ===
            "tools-check-dataset = scripts.tools.check_dataset_info:main",
            "tools-check-rs = scripts.tools.rs_devices:main",

            # === 测试 ===
            "test-gripper-ctrl = scripts.tests.gripper_ctrl:main",
            "test-arx-connection = scripts.tests.test_arx_modules:main",

            # === 帮助 ===
            "vr-help = scripts.help.help_info:main",
        ]
    },
)
