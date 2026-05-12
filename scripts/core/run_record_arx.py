"""
ARX LIFT VR Teleoperation Data Recording Script.
Adapted from run_record.py for ARX LIFT platform (dual R5 arms + LIFT chassis).

R5 arms have 7 joints each. Data is recorded in LeRobot format.
"""

import yaml
from pathlib import Path
from typing import Dict, Any
from scripts.utils.dataset_utils import generate_dataset_name, update_dataset_info
from robots.arx import ARXLift2Config, ARXLift2, ARXR5Config, ARXR5
from teleoperators.arx import ARXVRTeleopConfig, ARXVRTeleop
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras.realsense.camera_realsense import RealSenseCameraConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.processor import make_default_processors
from lerobot.utils.visualization_utils import init_rerun
from lerobot.utils.control_utils import init_keyboard_listener
from send2trash import send2trash
import time
import os
import select
import threading
import termios
import tty
import sys
from xrobotoolkit_teleop.common.xr_client import XrClient
from lerobot.utils.constants import HF_LEROBOT_HOME
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.utils.control_utils import sanity_check_dataset_robot_compatibility
import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")


class ARXRecordConfig:
    """Configuration class for ARX LIFT recording."""

    def __init__(self, cfg: Dict[str, Any]):
        storage = cfg["storage"]
        task = cfg["task"]
        time_cfg = cfg["time"]
        cam = cfg["cameras"]
        robot = cfg["robot"]
        teleop = cfg["teleop"]

        # Global config
        self.repo_id: str = cfg["repo_id"]
        self.debug: bool = cfg.get("debug", False)
        self.fps: int = cfg.get("fps", 20)
        self.dataset_path: str = HF_LEROBOT_HOME / self.repo_id
        self.user_info: str = cfg.get("user_notes", None)

        # Robot config
        robot_gripper = robot["gripper"]
        self.left_can: str = robot["left_can"]
        self.right_can: str = robot["right_can"]
        self.lift_can: str = robot.get("lift_can", "can5")
        self.arm_type: int = robot["arm_type"]
        self.robot_type: str = robot.get("robot_type", "arx_r5")
        self.left_init_joints: list = robot["left_init_joints"]
        self.right_init_joints: list = robot["right_init_joints"]
        self.init_height: float = robot.get("init_height", 0.0)
        self.dt: float = robot.get("dt", 0.05)
        self.chassis_mode: int = robot.get("chassis_mode", 2)
        self.use_gripper: bool = robot_gripper["use_gripper"]
        self.gripper_close: float = robot_gripper["close_position"]
        self.gripper_open: float = robot_gripper["open_position"]

        # Teleop config
        # Note: CAN config removed from teleop - teleop uses robot ref instead
        teleop_chassis = teleop["chassis"]
        teleop_gripper = teleop["gripper"]
        teleop_placo = teleop["placo"]
        self.scale_factor: float = teleop["scale_factor"]
        self.control_mode: str = teleop.get("control_mode", "vrteleop")
        self.R_headset_world: list = teleop["R_headset_world"]
        self.left_arm_yaw_comp_deg: float = teleop.get("left_arm_yaw_comp_deg", 0.0)
        self.right_arm_yaw_comp_deg: float = teleop.get("right_arm_yaw_comp_deg", 0.0)
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

        # Smoothing config (optional; defaults match ARXVRTeleopConfig).
        smoothing = teleop.get("smoothing", {})
        self.enable_pose_filter: bool = smoothing.get("enable_pose_filter", True)
        self.pose_min_cutoff: float = smoothing.get("pose_min_cutoff", 1.0)
        self.pose_beta: float = smoothing.get("pose_beta", 0.02)
        self.pose_d_cutoff: float = smoothing.get("pose_d_cutoff", 1.0)
        self.quat_slerp_alpha: float = smoothing.get("quat_slerp_alpha", 0.5)
        self.enable_joint_filter: bool = smoothing.get("enable_joint_filter", True)
        self.joint_ema_alpha: float = smoothing.get("joint_ema_alpha", 0.6)
        self.joint_max_velocity: float = smoothing.get("joint_max_velocity", 3.0)

        # Task config
        self.num_episodes: int = task.get("num_episodes", 1)
        self.display: bool = task.get("display", True)
        self.task_description: str = task.get("description", "ARX LIFT task")
        self.resume: bool = task.get("resume", False)
        self.resume_dataset: str = task.get("resume_dataset", "")

        # Time config
        self.episode_time_sec: int = time_cfg.get("episode_time_sec", 6000)
        self.reset_time_sec: int = time_cfg.get("reset_time_sec", 6000)
        self.save_meta_period: int = time_cfg.get("save_meta_period", 1)

        # Camera config
        self.left_wrist_cam_serial: str = cam["left_wrist_cam_serial"]
        self.right_wrist_cam_serial: str = cam["right_wrist_cam_serial"]
        self.exterior_cam_serial: str = cam["exterior_cam_serial"]
        self.width: int = cam["width"]
        self.height: int = cam["height"]

        # Storage config
        self.push_to_hub: bool = storage.get("push_to_hub", False)


def handle_incomplete_dataset(dataset_path):
    """Handle incomplete dataset by prompting user for deletion."""
    if dataset_path.exists():
        logging.info(f"====== [WARNING] Detected a potentially incomplete dataset folder: {dataset_path} ======")
        if not sys.stdin.isatty():
            logging.warning("stdin is not a TTY, skipping deletion prompt.")
            return
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        ans = input("Do you want to delete it? (y/n): ").strip().lower()
        if ans == "y":
            logging.info(f"====== [DELETE] Removing folder: {dataset_path} ======")
            send2trash(str(dataset_path))
            logging.info("====== [DONE] Incomplete dataset folder deleted successfully. ======")
        else:
            logging.info("====== [KEEP] Incomplete dataset folder retained, please check manually. ======")


def ensure_events_flag(events: Dict[str, Any], flag: bool = False):
    """Reset event flags."""
    events["rerecord_episode"] = flag
    events["exit_early"] = flag


class LocalTerminalKeyListener:
    """
    Terminal-local key listener fallback (doesn't rely on pynput global hook).
    Maps:
      Right Arrow -> exit_early
      Left Arrow  -> rerecord_episode + exit_early
      Esc         -> stop_recording + exit_early
    """

    def __init__(self, events: Dict[str, Any]):
        self.events = events
        self._stop = threading.Event()
        self._pause = threading.Event()
        self._thread = None
        self._fd = None
        self._old_termios = None
        self.started = False

    def start(self):
        if self.started:
            return
        if not sys.stdin.isatty():
            logging.warning("stdin is not a TTY, local keyboard fallback is disabled.")
            return

        self._fd = sys.stdin.fileno()
        self._old_termios = termios.tcgetattr(self._fd)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.started = True
        logging.info("Local terminal keyboard fallback enabled (←/→/Esc).")

    def pause(self):
        self._pause.set()

    def resume(self):
        self._pause.clear()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._fd is not None and self._old_termios is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_termios)
            except Exception:
                pass

    def _run(self):
        try:
            tty.setcbreak(self._fd)
            while not self._stop.is_set():
                if self._pause.is_set():
                    time.sleep(0.05)
                    continue

                ready, _, _ = select.select([self._fd], [], [], 0.1)
                if not ready:
                    continue

                key = os.read(self._fd, 1).decode("utf-8", errors="ignore")
                if not key:
                    continue

                if key == "\x1b":
                    seq = key
                    for _ in range(2):
                        ready2, _, _ = select.select([self._fd], [], [], 0.005)
                        if ready2:
                            seq += os.read(self._fd, 1).decode("utf-8", errors="ignore")

                    if seq == "\x1b[C":
                        self.events["exit_early"] = True
                    elif seq == "\x1b[D":
                        self.events["rerecord_episode"] = True
                        self.events["exit_early"] = True
                    else:
                        self.events["stop_recording"] = True
                        self.events["exit_early"] = True
        except Exception as e:
            logging.warning(f"Local terminal key listener stopped: {e}")


def listen_xrclient_reset_buttons(xr_client: XrClient, events: Dict[str, Any], stop_signal: threading.Event):
    """Listen XR buttons for single-arm reset shortcuts: A=right arm, X=left arm."""
    prev_a = False
    prev_x = False

    while not stop_signal.is_set():
        try:
            a_pressed = bool(xr_client.get_button_state_by_name("A"))
            x_pressed = bool(xr_client.get_button_state_by_name("X"))

            if a_pressed and not prev_a:
                events["reset_right_arm"] = True
                events["exit_early"] = True
                logging.info("检测到手柄 A：请求复位右臂。")

            if x_pressed and not prev_x:
                events["reset_left_arm"] = True
                events["exit_early"] = True
                logging.info("检测到手柄 X：请求复位左臂。")

            prev_a = a_pressed
            prev_x = x_pressed
        except Exception as e:
            logging.warning(f"读取XR按键失败: {e}")

        time.sleep(0.05)


def reset_to_init_position(record_cfg: ARXRecordConfig, robot: ARXLift2, teleop: ARXVRTeleop, first_time: bool = True, duration: float = 3.0):
    """Reset robot to initial joint positions with smooth interpolation.

    Args:
        duration: 插值运动时间 (秒), 越大越平滑
    """
    import numpy as np

    logging.info(f"Resetting to initial joint positions (duration={duration}s)...")
    logging.info(f"  Left target:  {record_cfg.left_init_joints}")
    logging.info(f"  Right target: {record_cfg.right_init_joints}")
    if record_cfg.robot_type == "arx_lift2":
        logging.info(f"  Height target: {record_cfg.init_height}")

    target_left = np.array(record_cfg.left_init_joints, dtype=np.float64)
    target_right = np.array(record_cfg.right_init_joints, dtype=np.float64)
    target_height = float(record_cfg.init_height)

    # 读取当前状态作为插值起点
    try:
        state = robot.bridge.get_full_state()
        current_left = np.array(state["left_arm"]["joint_positions"], dtype=np.float64)
        current_right = np.array(state["right_arm"]["joint_positions"], dtype=np.float64)
        current_height = float(state["chassis"]["height"]) if record_cfg.robot_type == "arx_lift2" else 0.0
    except Exception as e:
        logging.warning(f"无法读取当前状态，直接跳转: {e}")
        robot.bridge.set_dual_joint_positions(target_left, target_right)
        if record_cfg.robot_type == "arx_lift2":
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

        if record_cfg.robot_type == "arx_lift2":
            interp_height = current_height + alpha * (target_height - current_height)
            robot.bridge.set_full_command(
                interp_left, interp_right,
                0.0, 0.0, 0.0, interp_height
            )
        else:
            robot.bridge.set_dual_joint_positions(interp_left, interp_right)
        time.sleep(dt)

    # 最终确认到达目标
    if record_cfg.robot_type == "arx_lift2":
        robot.bridge.set_full_command(
            target_left, target_right,
            0.0, 0.0, 0.0, target_height
        )
    else:
        robot.bridge.set_dual_joint_positions(target_left, target_right)

    # Reset teleop tracking state
    if not first_time:
        teleop.reset_tracking()

    logging.info("✓ 已平滑到达初始位置")


def reset_single_arm_to_init_position(
    record_cfg: ARXRecordConfig,
    robot: ARXLift2,
    teleop: ARXVRTeleop,
    arm_side: str,
    duration: float = 2.0,
):
    """Smoothly reset one arm to init joints while keeping the other arm unchanged."""
    import numpy as np

    if arm_side not in ("left", "right"):
        raise ValueError(f"Invalid arm_side: {arm_side}")

    logging.info(f"Resetting {arm_side} arm to init position (duration={duration}s)...")

    target_left = np.array(record_cfg.left_init_joints, dtype=np.float64)
    target_right = np.array(record_cfg.right_init_joints, dtype=np.float64)

    try:
        state = robot.bridge.get_full_state()
        current_left = np.array(state["left_arm"]["joint_positions"], dtype=np.float64)
        current_right = np.array(state["right_arm"]["joint_positions"], dtype=np.float64)
        current_height = float(state["chassis"]["height"]) if record_cfg.robot_type == "arx_lift2" else 0.0
    except Exception as e:
        logging.warning(f"无法读取当前状态，单臂复位退化为直接指令: {e}")
        try:
            if arm_side == "left":
                robot.bridge.set_left_joint_positions(target_left)
            else:
                robot.bridge.set_right_joint_positions(target_right)
        except Exception:
            robot.bridge.set_dual_joint_positions(target_left, target_right)
        if not record_cfg.debug:
            time.sleep(0.5)
        teleop.reset_tracking()
        return

    hz = 50
    steps = max(1, int(duration * hz))
    dt = 1.0 / hz

    for i in range(1, steps + 1):
        t = i / steps
        alpha = t * t * (3.0 - 2.0 * t)

        interp_left = current_left.copy()
        interp_right = current_right.copy()
        if arm_side == "left":
            interp_left = current_left + alpha * (target_left - current_left)
        else:
            interp_right = current_right + alpha * (target_right - current_right)

        if record_cfg.robot_type == "arx_lift2":
            robot.bridge.set_full_command(interp_left, interp_right, 0.0, 0.0, 0.0, current_height)
        else:
            robot.bridge.set_dual_joint_positions(interp_left, interp_right)
        time.sleep(dt)

    if arm_side == "left":
        final_left = target_left
        final_right = current_right
    else:
        final_left = current_left
        final_right = target_right

    if record_cfg.robot_type == "arx_lift2":
        robot.bridge.set_full_command(final_left, final_right, 0.0, 0.0, 0.0, current_height)
    else:
        robot.bridge.set_dual_joint_positions(final_left, final_right)

    teleop.reset_tracking()
    logging.info(f"✓ {arm_side} arm reset completed.")


def handle_single_arm_reset_requests(
    events: Dict[str, Any],
    record_cfg: ARXRecordConfig,
    robot: ARXLift2,
    teleop: ARXVRTeleop,
    clear_episode_buffer_cb=None,
) -> bool:
    """Consume pending single-arm reset requests from XR buttons."""
    reset_left = bool(events.get("reset_left_arm", False))
    reset_right = bool(events.get("reset_right_arm", False))
    if not reset_left and not reset_right:
        return False

    if clear_episode_buffer_cb is not None:
        clear_episode_buffer_cb()

    if reset_right:
        reset_single_arm_to_init_position(record_cfg, robot, teleop, arm_side="right")
    if reset_left:
        reset_single_arm_to_init_position(record_cfg, robot, teleop, arm_side="left")

    events["reset_left_arm"] = False
    events["reset_right_arm"] = False
    ensure_events_flag(events, False)
    return True


def clear_rerun_namespaces() -> None:
    """Clear stale entities from previous runs in the same Rerun viewer."""
    try:
        import rerun as rr

        rr.log("observation", rr.Clear(recursive=True))
        rr.log("action", rr.Clear(recursive=True))
    except Exception as e:
        logging.debug(f"Skip rerun namespace clear: {e}")


def run_record(record_cfg: ARXRecordConfig):
    """Main recording function for ARX LIFT."""
    dataset_name = None
    dataset = None
    robot = None
    teleop = None
    xr_client = None
    listener = None
    local_key_listener = None
    xr_stop_signal = None

    try:
        dataset_name, data_version = generate_dataset_name(record_cfg)

        # Create RealSense camera configurations
        left_wrist_image_cfg = RealSenseCameraConfig(
            serial_number_or_name=record_cfg.left_wrist_cam_serial,
            fps=record_cfg.fps,
            width=record_cfg.width,
            height=record_cfg.height,
            color_mode=ColorMode.RGB,
            use_depth=False,
            rotation=Cv2Rotation.NO_ROTATION
        )

        right_wrist_image_cfg = RealSenseCameraConfig(
            serial_number_or_name=record_cfg.right_wrist_cam_serial,
            fps=record_cfg.fps,
            width=record_cfg.width,
            height=record_cfg.height,
            color_mode=ColorMode.RGB,
            use_depth=False,
            rotation=Cv2Rotation.NO_ROTATION
        )

        exterior_image_cfg = RealSenseCameraConfig(
            serial_number_or_name=record_cfg.exterior_cam_serial,
            fps=record_cfg.fps,
            width=record_cfg.width,
            height=record_cfg.height,
            color_mode=ColorMode.RGB,
            use_depth=False,
            rotation=Cv2Rotation.NO_ROTATION
        )

        camera_config = {
            "left_wrist_image": left_wrist_image_cfg,
            "right_wrist_image": right_wrist_image_cfg,
            "head_image": exterior_image_cfg,
        }

        # Create ARX robot configuration based on robot type
        if record_cfg.robot_type == "arx_r5":
            robot_config = ARXR5Config(
                left_can=record_cfg.left_can,
                right_can=record_cfg.right_can,
                arm_type=record_cfg.arm_type,
                num_joints=7,
                dt=record_cfg.dt,
                left_init_joints=record_cfg.left_init_joints,
                right_init_joints=record_cfg.right_init_joints,
                use_gripper=record_cfg.use_gripper,
                gripper_close=record_cfg.gripper_close,
                gripper_open=record_cfg.gripper_open,
                debug=record_cfg.debug,
                cameras=camera_config,
            )
            robot = ARXR5(robot_config)
        else:
            robot_config = ARXLift2Config(
                left_can=record_cfg.left_can,
                right_can=record_cfg.right_can,
                lift_can=record_cfg.lift_can,
                arm_type=record_cfg.arm_type,
                num_joints=7,
                dt=record_cfg.dt,
                left_init_joints=record_cfg.left_init_joints,
                right_init_joints=record_cfg.right_init_joints,
                init_height=record_cfg.init_height,
                chassis_mode=record_cfg.chassis_mode,
                use_gripper=record_cfg.use_gripper,
                gripper_close=record_cfg.gripper_close,
                gripper_open=record_cfg.gripper_open,
                debug=record_cfg.debug,
                cameras=camera_config,
            )
            robot = ARXLift2(robot_config)

        # Configure dataset features
        action_features = hw_to_dataset_features(robot.action_features, "action")
        obs_features = hw_to_dataset_features(robot.observation_features, "observation", use_video=True)
        dataset_features = {**action_features, **obs_features}

        if record_cfg.resume:
            dataset = LeRobotDataset(dataset_name)
            if hasattr(robot, "cameras") and len(robot.cameras) > 0:
                dataset.start_image_writer()
            sanity_check_dataset_robot_compatibility(dataset, robot, record_cfg.fps, dataset_features)
        else:
            dataset = LeRobotDataset.create(
                repo_id=dataset_name,
                fps=record_cfg.fps,
                features=dataset_features,
                robot_type=robot.name,
                use_videos=True,
                image_writer_threads=4,
            )

        dataset.meta.metadata_buffer_size = record_cfg.save_meta_period

        # Initialize XR teleop only after dataset is successfully prepared.
        xr_client = XrClient()
        teleop_config = ARXVRTeleopConfig(
            xr_client=xr_client,
            fps=record_cfg.fps,
            scale_factor=record_cfg.scale_factor,
            R_headset_world=record_cfg.R_headset_world,
            left_arm_yaw_comp_deg=record_cfg.left_arm_yaw_comp_deg,
            right_arm_yaw_comp_deg=record_cfg.right_arm_yaw_comp_deg,
            trigger_reverse=record_cfg.trigger_reverse,
            trigger_threshold=record_cfg.trigger_threshold,
            close_position=record_cfg.teleop_close_position,
            open_position=record_cfg.teleop_open_position,
            enable_lift=record_cfg.robot_type == "arx_lift2",
            chassis_vx_scale=record_cfg.chassis_vx_scale,
            chassis_vy_scale=record_cfg.chassis_vy_scale,
            chassis_wz_scale=record_cfg.chassis_wz_scale,
            chassis_height_scale=record_cfg.chassis_height_scale,
            chassis_mode=record_cfg.chassis_mode,
            control_mode=record_cfg.control_mode,
            robot_urdf_path=record_cfg.robot_urdf_path,
            servo_time=record_cfg.servo_time,
            visualize_placo=record_cfg.visualize_placo,
            enable_pose_filter=record_cfg.enable_pose_filter,
            pose_min_cutoff=record_cfg.pose_min_cutoff,
            pose_beta=record_cfg.pose_beta,
            pose_d_cutoff=record_cfg.pose_d_cutoff,
            quat_slerp_alpha=record_cfg.quat_slerp_alpha,
            enable_joint_filter=record_cfg.enable_joint_filter,
            joint_ema_alpha=record_cfg.joint_ema_alpha,
            joint_max_velocity=record_cfg.joint_max_velocity,
        )
        teleop = ARXVRTeleop(teleop_config)

        # Initialize keyboard listener and visualization
        listener, events = init_keyboard_listener()
        events["reset_left_arm"] = False
        events["reset_right_arm"] = False
        local_key_listener = LocalTerminalKeyListener(events)
        local_key_listener.start()
        init_rerun(session_name="recording")
        clear_rerun_namespaces()

        # Create processors
        teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

        # Connect robot and teleop
        # Order is important:
        # 1. robot.connect() - creates ROS2 Bridge and connects to hardware
        # 2. teleop.set_robot_reference(robot) - shares Bridge reference for state reading
        # 3. reset_to_init_position() - move to initial position
        # 4. teleop.connect() - starts VR update thread
        robot.connect()
        teleop.set_robot_reference(robot)  # Share Bridge reference
        reset_to_init_position(record_cfg, robot, teleop, first_time=True)
        teleop.connect()

        xr_stop_signal = threading.Event()
        xr_listener_thread = threading.Thread(
            target=listen_xrclient_reset_buttons,
            args=(xr_client, events, xr_stop_signal),
            daemon=True,
        )
        xr_listener_thread.start()

        episode_idx = 0
        events["stop_recording"] = False
        ensure_events_flag(events, False)
        logging.info("控制按键：→=下一步，←=重录当前episode，Esc=停止录制，Ctrl+C=退出。")
        logging.info("手柄按键：A=复位右臂，X=复位左臂（触发后当前episode会重录）。")

        while episode_idx < record_cfg.num_episodes and not events["stop_recording"]:
            logging.info(f"====== [RECORD] Recording episode {episode_idx + 1} of {record_cfg.num_episodes} ======")

            record_loop(
                robot=robot,
                events=events,
                fps=record_cfg.fps,
                teleop=teleop,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                dataset=dataset,
                control_time_s=record_cfg.episode_time_sec,
                single_task=record_cfg.task_description,
                display_data=record_cfg.display,
            )

            if handle_single_arm_reset_requests(
                events, record_cfg, robot, teleop, clear_episode_buffer_cb=dataset.clear_episode_buffer
            ):
                logging.info("Single-arm reset done. Re-recording current episode.")
                continue

            if events["rerecord_episode"]:
                logging.info("Re-recording episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                reset_to_init_position(record_cfg, robot, teleop, first_time=False)
                continue

            dataset.save_episode()
            ensure_events_flag(events, False)

            # Reset environment if not stopping
            if not events["stop_recording"] and episode_idx < record_cfg.num_episodes - 1:
                while True:
                    if handle_single_arm_reset_requests(events, record_cfg, robot, teleop):
                        continue
                    termios.tcflush(sys.stdin, termios.TCIFLUSH)
                    local_key_listener.pause()
                    try:
                        user_input = input("====== [WAIT] Press Enter to reset the environment ======")
                    finally:
                        local_key_listener.resume()
                    if user_input == "":
                        break
                    logging.info("====== [WARNING] Please press only Enter to continue ======")

                logging.info("====== [RESET] Resetting the environment ======")
                logging.info("在 reset 阶段按右方向键(→)可提前结束并进入下一集。")

                record_loop(
                    robot=robot,
                    events=events,
                    fps=record_cfg.fps,
                    teleop=teleop,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    control_time_s=record_cfg.reset_time_sec,
                    single_task=record_cfg.task_description,
                    display_data=record_cfg.display,
                )
                handle_single_arm_reset_requests(events, record_cfg, robot, teleop)
                ensure_events_flag(events, False)

                reset_to_init_position(record_cfg, robot, teleop, first_time=False)

            ensure_events_flag(events, False)
            episode_idx += 1

        logging.info("✅ 录制流程结束，准备清理资源。")

        # 真正的 Clean up
        logging.info("Stop recording and cleaning up...")
        if xr_stop_signal is not None:
            xr_stop_signal.set()
        if local_key_listener is not None:
            local_key_listener.stop()
        if listener is not None:
            listener.stop()
        if robot is not None:
            robot.disconnect()
        if teleop is not None:
            teleop.disconnect()
        if dataset is not None:
            dataset.finalize()

        update_dataset_info(record_cfg, dataset_name, data_version)
        if record_cfg.push_to_hub:
            dataset.push_to_hub()

    except Exception as e:
        try:
            if xr_stop_signal is not None:
                xr_stop_signal.set()
            if local_key_listener is not None:
                local_key_listener.stop()
            if listener is not None:
                listener.stop()
            if teleop is not None:
                teleop.disconnect()
            if robot is not None:
                robot.disconnect()
        except Exception:
            pass
        logging.error(f"====== [ERROR] {e} ======")
        import traceback
        traceback.print_exc()
        if isinstance(e, FileExistsError):
            logging.error(
                "Dataset folder already exists. "
                "This is a naming collision, not an incomplete dataset."
            )
            logging.error(
                "Please re-run after updating naming (or changing repo_id), "
                "so a new version folder is generated."
            )
        elif dataset_name and dataset is not None:
            dataset_path = Path(HF_LEROBOT_HOME) / dataset_name
            handle_incomplete_dataset(dataset_path)
        sys.exit(1)

    except KeyboardInterrupt:
        try:
            if xr_stop_signal is not None:
                xr_stop_signal.set()
            if local_key_listener is not None:
                local_key_listener.stop()
            if listener is not None:
                listener.stop()
            if teleop is not None:
                teleop.disconnect()
            if robot is not None:
                robot.disconnect()
        except Exception:
            pass
        logging.info("\n====== [INFO] Ctrl+C detected, cleaning up incomplete dataset... ======")
        if dataset_name and dataset is not None:
            dataset_path = Path(HF_LEROBOT_HOME) / dataset_name
            handle_incomplete_dataset(dataset_path)
        sys.exit(1)


def main():
    """Entry point for ARX LIFT recording."""
    parent_path = Path(__file__).resolve().parent
    default_cfg = parent_path.parent / "config" / "cfg_arx.yaml"
    cfg_path = Path(os.environ.get("ARX_CONFIG", str(default_cfg)))
    if not cfg_path.is_absolute():
        cfg_path = (parent_path.parent / cfg_path).resolve()
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)

    logging.info(f"Loading config: {cfg_path}")
    record_cfg = ARXRecordConfig(cfg["record"])
    run_record(record_cfg)


if __name__ == "__main__":
    main()
