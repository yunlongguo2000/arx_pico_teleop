"""
ARX LIFT VR Pure Teleoperation Script.
Only runs VR teleoperation, no data recording.
Adapted from run_record_arx.py with all dataset recording removed.
"""

import yaml
from pathlib import Path
from typing import Dict, Any
from robots.arx import ARXLift2Config, ARXLift2
from teleoperators.arx import ARXVRTeleopConfig, ARXVRTeleop
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.visualization_utils import init_rerun
import time
import signal
import os
import threading
import sys
import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")


class ARXTeleopConfig:
    """Configuration class for ARX LIFT pure teleoperation."""

    def __init__(self, cfg: Dict[str, Any]):
        robot = cfg["robot"]
        teleop = cfg["teleop"]

        # Global config
        self.debug: bool = cfg.get("debug", False)

        # Robot config
        robot_gripper = robot["gripper"]
        self.left_can: str = robot["left_can"]
        self.right_can: str = robot["right_can"]
        self.lift_can: str = robot["lift_can"]
        self.arm_type: int = robot["arm_type"]
        self.left_init_joints: list = robot["left_init_joints"]
        self.right_init_joints: list = robot["right_init_joints"]
        self.init_height: float = robot["init_height"]
        self.dt: float = robot.get("dt", 0.05)
        self.chassis_mode: int = robot.get("chassis_mode", 2)
        self.use_gripper: bool = robot_gripper["use_gripper"]
        self.gripper_close: float = robot_gripper["close_position"]
        self.gripper_open: float = robot_gripper["open_position"]

        # Teleop config
        teleop_chassis = teleop["chassis"]
        teleop_gripper = teleop["gripper"]
        teleop_placo = teleop["placo"]
        self.scale_factor: float = teleop["scale_factor"]
        self.control_mode: str = teleop.get("control_mode", "vrteleop")
        self.R_headset_world: list = teleop["R_headset_world"]
        self.trigger_reverse: bool = teleop_gripper["trigger_reverse"]
        self.trigger_threshold: float = teleop_gripper["trigger_threshold"]
        self.teleop_close_position: float = teleop_gripper["close_position"]
        self.teleop_open_position: float = teleop_gripper["open_position"]
        self.chassis_vx_scale: float = teleop_chassis["vx_scale"]
        self.chassis_vy_scale: float = teleop_chassis["vy_scale"]
        self.chassis_wz_scale: float = teleop_chassis["wz_scale"]
        self.chassis_height_scale: float = teleop_chassis["height_scale"]
        # Placo IK config
        self.robot_urdf_path: str = teleop_placo["robot_urdf_path"]
        self.servo_time: float = teleop_placo["servo_time"]
        self.visualize_placo: bool = teleop_placo["visualize_placo"]


def check_keyboard_interrupt(events):
    """Check for keyboard interrupt."""
    if events["keyboard_interrupt"]:
        raise KeyboardInterrupt


def listen_xrclient(xr_client, events, stop_signal):
    """XR client listening thread for exit button."""
    while not stop_signal.is_set():
        try:
            # Check if X button is pressed to exit
            if xr_client.get_button_state_by_name("X"):
                logging.info("X按钮被按下，准备退出程序")
                events["stop_recording"] = True
                break
        except Exception as e:
            logging.warning(f"XR client监听错误: {e}")
        time.sleep(0.1)


def reset_to_init_position(teleop_cfg, robot, teleop, first_time: bool = True, duration: float = 3.0):
    """Reset robot to initial joint positions with smooth interpolation.

    Args:
        duration: 插值运动时间 (秒), 越大越平滑
    """
    import numpy as np

    logging.info(f"Resetting to initial joint positions (duration={duration}s)...")
    logging.info(f"  Left target:  {teleop_cfg.left_init_joints}")
    logging.info(f"  Right target: {teleop_cfg.right_init_joints}")
    logging.info(f"  Height target: {teleop_cfg.init_height}")

    target_left = np.array(teleop_cfg.left_init_joints, dtype=np.float64)
    target_right = np.array(teleop_cfg.right_init_joints, dtype=np.float64)
    target_height = float(teleop_cfg.init_height)

    # 读取当前状态作为插值起点
    try:
        state = robot.bridge.get_full_state()
        current_left = np.array(state["left_arm"]["joint_positions"], dtype=np.float64)
        current_right = np.array(state["right_arm"]["joint_positions"], dtype=np.float64)
        current_height = float(state["chassis"]["height"])
    except Exception as e:
        logging.warning(f"无法读取当前状态，直接跳转: {e}")
        robot.bridge.set_dual_joint_positions(target_left, target_right)
        robot.bridge.set_chassis_height(target_height)
        time.sleep(1.0)
        if not first_time:
            teleop.reset_tracking()
        return

    # 平滑插值: 以 50Hz 发送插值命令
    hz = 50
    steps = int(duration * hz)
    dt = 1.0 / hz

    logging.info(f"  开始平滑移动 ({steps} steps @ {hz}Hz)...")
    for i in range(1, steps + 1):
        # 使用 smoothstep (3t^2 - 2t^3) 实现加减速
        t = i / steps
        alpha = t * t * (3.0 - 2.0 * t)

        interp_left = current_left + alpha * (target_left - current_left)
        interp_right = current_right + alpha * (target_right - current_right)
        interp_height = current_height + alpha * (target_height - current_height)

        robot.bridge.set_full_command(
            interp_left, interp_right,
            0.0, 0.0, 0.0, interp_height
        )
        time.sleep(dt)

    # 最终确认到达目标
    robot.bridge.set_full_command(
        target_left, target_right,
        0.0, 0.0, 0.0, target_height
    )

    # Reset teleop tracking state
    if not first_time:
        teleop.reset_tracking()

    logging.info("✓ 已平滑到达初始位置")


def teleop_loop(robot, events, teleop,
                control_time_s=6000):
    """Main teleoperation loop - continuously runs, no data recording."""
    start_time = time.time()

    while time.time() - start_time < control_time_s and not events["stop_recording"] and not events["exit_early"]:
        check_keyboard_interrupt(events)

        # Get VR teleop action - already processed by teleop.get_action()
        action = teleop.get_action()

        # Send action directly to robot - send_action already handles command sending
        robot.send_action(action)

        # Read current observation (updates cached state in robot)
        robot.get_observation()

        # Print teleop status periodically
        if int((time.time() - start_time) * 10) % 10 == 0:
            logging.debug(f"Teleop running - time: {time.time() - start_time:.1f}s")

        # Sleep to maintain control rate
        if hasattr(robot, 'cfg') and robot.cfg.dt > 0:
            time.sleep(robot.cfg.dt)

        check_keyboard_interrupt(events)


def run_teleop(teleop_cfg):
    """Main teleoperation function without data recording."""
    from xrobotoolkit_teleop.common.xr_client import XrClient

    logging.info("====== ARX LIFT VR 纯遥操作启动 ======")
    logging.info("模式: 只遥操作，不录制数据")
    logging.info("按 X 按钮退出程序")
    logging.info("")

    try:
        # Initialize XR client
        xr_client = XrClient()

        # Create configs
        robot_config = ARXLift2Config(
            left_can=teleop_cfg.left_can,
            right_can=teleop_cfg.right_can,
            lift_can=teleop_cfg.lift_can,
            arm_type=teleop_cfg.arm_type,
            num_joints=7,  # R5 has 7 joints
            dt=teleop_cfg.dt,
            left_init_joints=teleop_cfg.left_init_joints,
            right_init_joints=teleop_cfg.right_init_joints,
            init_height=teleop_cfg.init_height,
            chassis_mode=teleop_cfg.chassis_mode,
            use_gripper=teleop_cfg.use_gripper,
            gripper_close=teleop_cfg.gripper_close,
            gripper_open=teleop_cfg.gripper_open,
            debug=teleop_cfg.debug,
            cameras={}  # No cameras needed for pure teleop
        )
        teleop_config = ARXVRTeleopConfig(
            xr_client=xr_client,
            fps=30,
            scale_factor=teleop_cfg.scale_factor,
            R_headset_world=teleop_cfg.R_headset_world,
            trigger_reverse=teleop_cfg.trigger_reverse,
            trigger_threshold=teleop_cfg.trigger_threshold,
            close_position=teleop_cfg.teleop_close_position,
            open_position=teleop_cfg.teleop_open_position,
            chassis_vx_scale=teleop_cfg.chassis_vx_scale,
            chassis_vy_scale=teleop_cfg.chassis_vy_scale,
            chassis_wz_scale=teleop_cfg.chassis_wz_scale,
            chassis_height_scale=teleop_cfg.chassis_height_scale,
            chassis_mode=teleop_cfg.chassis_mode,
            control_mode=teleop_cfg.control_mode,
            robot_urdf_path=teleop_cfg.robot_urdf_path,
            servo_time=teleop_cfg.servo_time,
            visualize_placo=teleop_cfg.visualize_placo
        )

        # Initialize the robot and teleoperator
        robot = ARXLift2(robot_config)
        teleop = ARXVRTeleop(teleop_config)

        # Initialize keyboard listener and visualization
        _, events = init_keyboard_listener()
        init_rerun(session_name="teleoperation")

        # Connect robot and teleop
        # Order is important:
        # 1. robot.connect() - creates ROS2 Bridge and connects to hardware
        # 2. teleop.set_robot_reference(robot) - shares Bridge reference for state reading
        # 3. reset_to_init_position() - move to initial position
        # 4. teleop.connect() - starts VR update thread
        robot.connect()
        teleop.set_robot_reference(robot)  # Share Bridge reference
        reset_to_init_position(teleop_cfg, robot, teleop, first_time=True)
        teleop.connect()

        # Start VR input listener thread
        stop_signal = threading.Event()
        listener_thread = threading.Thread(target=listen_xrclient, args=(xr_client, events, stop_signal), daemon=True)
        listener_thread.start()

        events["stop_recording"] = False
        events["keyboard_interrupt"] = False
        events["exit_early"] = False

        logging.info("")
        logging.info("✅ 遥操作已启动，可以开始控制机器人了")
        logging.info("🎮 使用VR控制器控制机械臂和底盘")
        logging.info("❌ 按下X按钮退出程序")
        logging.info("")

        # Main teleoperation loop - runs until X is pressed or Ctrl+C
        while not events["stop_recording"]:
            check_keyboard_interrupt(events)

            # Run continuous teleop - no processors needed for pure teleop
            teleop_loop(
                robot=robot,
                events=events,
                teleop=teleop,
                control_time_s=3600,  # 1 hour max per loop, will restart if not stopped
            )

            if not events["stop_recording"]:
                logging.info("计时到，重新启动遥操作循环")

        # Clean up
        logging.info("停止遥操作，清理资源...")
        robot.disconnect()
        teleop.disconnect()
        stop_signal.set()
        # Wait for listener thread to exit cleanly before Python exit
        # This prevents C++ SDK from crashing during interpreter shutdown
        listener_thread.join(timeout=1.0)
        # Manually delete XrClient before exit to ensure C++ destructor runs
        # while Python interpreter is still active
        del xr_client

        logging.info("✅ 遥操作已安全退出")

    except Exception as e:
        logging.error(f"====== [ERROR] {e} ======")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    except KeyboardInterrupt:
        logging.info("\n====== [INFO] Ctrl+C 检测到，正在退出... ======")
        sys.exit(0)


def main():
    """Entry point for ARX LIFT pure teleoperation."""
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "cfg_arx.yaml"
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)

    teleop_cfg = ARXTeleopConfig(cfg["record"])
    run_teleop(teleop_cfg)


if __name__ == "__main__":
    main()
