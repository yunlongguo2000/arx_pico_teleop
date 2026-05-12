import logging
import os
import time
from typing import Any
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np
import sys

from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from ros2_bridge.arx_ros2_rpc_client import ArxROS2RPCClient

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot
from .config_arx_lift2 import ARXLift2Config

try:
    from algorithms.calibration import load_calibration_params
    from algorithms.calibration.board_utils import create_gridboard, create_detector
    from algorithms.calibration.pose_estimation import (
        compute_head_camera_pose, compose_head_camera_pose, pose_to_7dof,
    )

    _CALIB_AVAILABLE = True
except ImportError:
    _CALIB_AVAILABLE = False

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

_EE_AXES = ["x", "y", "z", "roll", "pitch", "yaw"]


def _angle_diff(a: float, b: float) -> float:
    """Compute shortest angular difference (a - b), handling ±π wrapping."""
    d = a - b
    return (d + np.pi) % (2 * np.pi) - np.pi


class _PerfStats:
    """Lightweight rolling performance stats for periodic logging."""
    def __init__(self, name: str, report_every: int = 100):
        self.name = name
        self.report_every = report_every
        self._count = 0
        self._sums = {}
        self._maxs = {}

    def record(self, **kwargs):
        self._count += 1
        for k, v in kwargs.items():
            self._sums[k] = self._sums.get(k, 0.0) + v
            self._maxs[k] = max(self._maxs.get(k, 0.0), v)
        if self._count % self.report_every == 0:
            parts = []
            for k in kwargs:
                avg = self._sums[k] / self.report_every * 1000
                mx = self._maxs[k] * 1000
                parts.append(f"{k}={avg:.1f}/{mx:.1f}ms")
            logger.info(f"[PERF {self.name}] {self._count} frames | " + " | ".join(parts))
            self._sums.clear()
            self._maxs.clear()


class ARXLift2(Robot):
    """
    ARX LIFT2 robot class using ROS2 Bridge for hardware abstraction.
    Extends LeRobot's Robot base class.

    R5/X5lite arms have 6 arm joints + 1 gripper (7 total from SDK).
    Connected via ROS2 Bridge which manages CAN bus communication.
    LIFT chassis connected via can5.

    Joint structure per arm:
        - Joints 1-6: Arm joints (URDF: joint1-joint6)
        - Joint 7: Gripper position (from SDK, separate control via set_catch_pos)

    Architecture:
        - Uses ARXLift2Bridge for all hardware communication
        - Bridge handles ROS2 message processing in background threads
        - No direct CAN or SDK calls - all abstracted through Bridge
    """

    config_class = ARXLift2Config
    name = "arx_lift2"

    def __init__(self, config: ARXLift2Config):
        super().__init__(config)
        self.cameras = make_cameras_from_configs(config.cameras)

        self.cfg = config
        self._is_connected = False
        self._num_joints = config.num_joints

        # ROS2 Bridge - replaces direct BimanualArm and LiftHeadControlLoop
        # Will be initialized in connect()
        self.bridge = None

        # Gripper state tracking (Bridge reads actual position from SDK joint 7)
        self._left_gripper_position = config.gripper_open
        self._right_gripper_position = config.gripper_open

        # Cached state for avoiding redundant RPC calls within same frame
        self._last_state = None

        # Previous ee_pose tracking for observation delta computation
        self._prev_obs_left_ee = np.zeros(6)
        self._prev_obs_right_ee = np.zeros(6)
        self._obs_initialized = False

        # Performance monitoring
        self._perf = _PerfStats("main_loop", report_every=100)

        # Head camera extrinsic calibration
        self._calib_enabled = config.enable_calibration and _CALIB_AVAILABLE
        self._calib_detector = None
        self._calib_board = None
        self._calib_camera_matrix = None
        self._calib_dist_coeffs = None
        self._calib_T_board_to_world = np.eye(4)
        self._calib_head_image_key = "head_image"  # from camera config naming
        self._calib_pose_7d = np.zeros(7, dtype=np.float32)
        if self._calib_enabled:
            try:
                params = load_calibration_params(config.calibration_params_path)
                bc = params["calibration_board"]
                self._calib_board = create_gridboard(
                    markers_x=bc["markers_x"], markers_y=bc["markers_y"],
                    marker_length_m=bc["marker_length_m"],
                    marker_separation_m=bc["marker_separation_m"],
                    dictionary_name=bc["dictionary"],
                )
                self._calib_detector = create_detector(bc["dictionary"])
                cc = params["cameras"]["head"]["intrinsics"]
                self._calib_camera_matrix = np.array(
                    [[cc["fx"], 0, cc["cx"]], [0, cc["fy"], cc["cy"]], [0, 0, 1]],
                    dtype=np.float64,
                )
                self._calib_dist_coeffs = np.array(cc["dist_coeffs"], dtype=np.float64)
                tw = params["T_board_to_world"]
                if tw["translation_m"] != [0.0, 0.0, 0.0] or tw["quaternion_xyzw"] != [0.0, 0.0, 0.0, 1.0]:
                    t = tw["translation_m"]
                    q = tw["quaternion_xyzw"]
                    qw, qx, qy, qz = q[3], q[0], q[1], q[2]
                    R = np.array([
                        [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                        [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
                        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy],
                    ], dtype=np.float64)
                    self._calib_T_board_to_world[:3, :3] = R
                    self._calib_T_board_to_world[:3, 3] = t
                logger.info(f"[CALIB] Calibration loaded (board: {bc['markers_x']}x{bc['markers_y']})")
            except Exception as e:
                logger.warning(f"[CALIB] Failed to load calibration: {e}")
                self._calib_enabled = False

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self.name} is already connected.")

        rpc_host = os.environ.get("ARX_RPC_HOST", "localhost")
        rpc_port = int(os.environ.get("ARX_RPC_PORT", "4242"))
        logger.info(
            "\n===== [ROBOT] Initializing ARX LIFT2 System via RPC (tcp://%s:%s) =====",
            rpc_host,
            rpc_port,
        )

        try:
            # ZMQ RPC client (local or remote control PC via ARX_RPC_HOST / ARX_RPC_PORT)
            self.bridge = ArxROS2RPCClient(ip=rpc_host, port=rpc_port)

            # Connect to server
            if not self.bridge.system_connect(timeout=10.0):
                raise Exception("Failed to connect to ZeroRPC server")

            # Display current state
            left_pos = self.bridge.get_left_joint_positions()
            right_pos = self.bridge.get_right_joint_positions()

            logger.info(f"[ROBOT] Left R5 arm joint positions ({len(left_pos)} joints): {[round(p, 4) for p in left_pos]}")
            logger.info(f"[ROBOT] Right R5 arm joint positions ({len(right_pos)} joints): {[round(p, 4) for p in right_pos]}")
            chassis_height = self.bridge.get_chassis_height()
            logger.info(f"[CHASSIS] Current height: {chassis_height:.4f} m")
            logger.info("===== [ROBOT] System connected successfully =====\n")

        except Exception as e:
            logger.error(f"===== [ERROR] Failed to connect to ARX LIFT2 system: {e} =====")
            raise

        # Connect cameras
        logger.info("\n===== [CAM] Initializing Cameras =====")
        for cam_name, cam in self.cameras.items():
            cam.connect()
            logger.info(f"[CAM] {cam_name} connected successfully.")
        logger.info("===== [CAM] Cameras Initialized Successfully =====\n")

        self.is_connected = True
        logger.info(f"[INFO] {self.name} env initialization completed successfully.\n")

    @property
    def _motors_ft(self) -> dict[str, type]:
        """Motor state features for observation. R5 has 6 arm joints + 1 gripper per arm."""
        ft = {
            # Left arm joints (position, velocity, current) - 6 arm joints + 1 gripper
            **{f"left_joint_{i+1}.pos": float for i in range(self._num_joints)},
            **{f"left_joint_{i+1}.vel": float for i in range(self._num_joints)},
            **{f"left_joint_{i+1}.cur": float for i in range(self._num_joints)},
            # Right arm joints - 6 arm joints + 1 gripper
            **{f"right_joint_{i+1}.pos": float for i in range(self._num_joints)},
            **{f"right_joint_{i+1}.vel": float for i in range(self._num_joints)},
            **{f"right_joint_{i+1}.cur": float for i in range(self._num_joints)},
            # TCP poses (from SDK get_ee_pose) - RPY format [x,y,z,roll,pitch,yaw]
            **{f"left_tcp_pose.{axis}": float for axis in _EE_AXES},
            **{f"right_tcp_pose.{axis}": float for axis in _EE_AXES},
            # Delta TCP poses (frame-to-frame change in ee_pose)
            **{f"left_delta_tcp_pose.{axis}": float for axis in _EE_AXES},
            **{f"right_delta_tcp_pose.{axis}": float for axis in _EE_AXES},
            # Gripper state
            "left_gripper_position": float,
            "right_gripper_position": float,
            # Chassis state
            "chassis_height": float,
            "chassis_head_yaw": float,
            "chassis_head_pitch": float,
        }
        if self._calib_enabled:
            ft.update({
                **{f"head_camera_pose.{axis}": float for axis in ["x", "y", "z", "qx", "qy", "qz", "qw"]},
            })
        return ft

    @property
    def action_features(self) -> dict[str, type]:
        """Action features for control. R5 has 6 arm joints + 1 gripper per arm."""
        ft = {
            # Joint position commands - 6 arm joints + 1 gripper
            **{f"left_joint_{i+1}.pos": float for i in range(self._num_joints)},
            **{f"right_joint_{i+1}.pos": float for i in range(self._num_joints)},
            # EE pose targets (commanded target from Placo IK) - RPY format [x,y,z,roll,pitch,yaw]
            **{f"left_tcp_pose.{axis}": float for axis in _EE_AXES},
            **{f"right_tcp_pose.{axis}": float for axis in _EE_AXES},
            # Delta EE pose targets (frame-to-frame change in target ee_pose)
            **{f"left_delta_tcp_pose.{axis}": float for axis in _EE_AXES},
            **{f"right_delta_tcp_pose.{axis}": float for axis in _EE_AXES},
            # Gripper commands
            "left_gripper_position": float,
            "right_gripper_position": float,
            # Chassis commands
            "chassis_vx": float,
            "chassis_vy": float,
            "chassis_wz": float,
            "chassis_height": float,
        }
        return ft

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send action commands to robot via ROS2 Bridge.

        Supports two action formats (auto-detected from action keys):
        - Joint position control: keys like ``left_joint_1.pos``  (pick_handover)
        - Delta EE pose control:  keys like ``left_delta_ee_pose.x`` (fold_cloth)
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        _t0 = time.perf_counter()
        if not self.cfg.debug:
            if "left_joint_1.pos" in action:
                # ── 关节角模式 (pick_handover) ──────────────────────────────
                left_pos = np.array([action[f"left_joint_{i+1}.pos"] for i in range(self._num_joints)])
                right_pos = np.array([action[f"right_joint_{i+1}.pos"] for i in range(self._num_joints)])

                # gripper_position 是归一化值 (-0.2~0.2), 需要 * 5.0 转为 SDK 原始值
                if "left_gripper_position" in action:
                    self._left_gripper_position = action["left_gripper_position"]
                    left_pos[6] = self._left_gripper_position * 5.0
                if "right_gripper_position" in action:
                    self._right_gripper_position = action["right_gripper_position"]
                    right_pos[6] = self._right_gripper_position * 5.0

                vx = action.get("chassis_vx", 0.0)
                vy = action.get("chassis_vy", 0.0)
                wz = action.get("chassis_wz", 0.0)
                height = action.get("chassis_height", 0.0)

                self.bridge.set_full_command(left_pos, right_pos, vx, vy, wz, height)

            elif "left_delta_ee_pose.x" in action:
                # ── Delta EE 位姿模式 (fold_cloth) ──────────────────────────
                # 数据集 action key 轴名: x/y/z/rx/ry/rz (rx=roll, ry=pitch, rz=yaw)
                _AXES = ["x", "y", "z", "rx", "ry", "rz"]
                left_delta  = np.array([action[f"left_delta_ee_pose.{ax}"]  for ax in _AXES])
                right_delta = np.array([action[f"right_delta_ee_pose.{ax}"] for ax in _AXES])

                # 从上一帧缓存的状态里读当前末端位姿 [x, y, z, roll, pitch, yaw]
                # infer.py 每步都先调 get_observation()，所以 _last_state 是最新的
                left_ee  = np.asarray(self._last_state["left_arm"]["end_pose"],  dtype=np.float64)
                right_ee = np.asarray(self._last_state["right_arm"]["end_pose"], dtype=np.float64)

                new_left  = (left_ee  + left_delta).tolist()
                new_right = (right_ee + right_delta).tolist()

                # 二值夹爪指令: 训练值为 0/1，但模型输出可能超出范围，阈值化到 0/1
                # > 0.5 → 1.0（关）；<= 0.5 → 0.0（开）
                left_gripper  = 1.0 if action.get("left_gripper_cmd_bin",  0.0) > 0.5 else 0.0
                right_gripper = 1.0 if action.get("right_gripper_cmd_bin", 0.0) > 0.5 else 0.0

                self.bridge.set_dual_ee_poses(new_left, new_right, left_gripper, right_gripper)

            else:
                logger.warning("send_action: 未知 action 格式，忽略。keys=%s", list(action.keys())[:4])

        _t_send = time.perf_counter() - _t0

        return action

    def get_observation(self) -> dict[str, Any]:
        """Get current observation from robot via ROS2 Bridge."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        _t0 = time.perf_counter()
        obs_dict = {}

        # Read full state from Bridge (also cached for teleop reuse)
        state = self.bridge.get_full_state()
        self._last_state = state
        _t_rpc = time.perf_counter()

        # Extract arm joint states
        left_pos = state["left_arm"]["joint_positions"]
        right_pos = state["right_arm"]["joint_positions"]
        left_vel = state["left_arm"]["joint_velocities"]
        right_vel = state["right_arm"]["joint_velocities"]
        left_cur = state["left_arm"]["joint_currents"]
        right_cur = state["right_arm"]["joint_currents"]

        for i in range(self._num_joints):
            obs_dict[f"left_joint_{i+1}.pos"] = float(left_pos[i])
            obs_dict[f"left_joint_{i+1}.vel"] = float(left_vel[i])
            obs_dict[f"left_joint_{i+1}.cur"] = float(left_cur[i])
            obs_dict[f"right_joint_{i+1}.pos"] = float(right_pos[i])
            obs_dict[f"right_joint_{i+1}.vel"] = float(right_vel[i])
            obs_dict[f"right_joint_{i+1}.cur"] = float(right_cur[i])

        # Read TCP poses (Bridge returns [x,y,z,roll,pitch,yaw])
        left_ee = np.asarray(state["left_arm"]["end_pose"], dtype=np.float64)
        right_ee = np.asarray(state["right_arm"]["end_pose"], dtype=np.float64)

        for i, axis in enumerate(_EE_AXES):
            obs_dict[f"left_tcp_pose.{axis}"] = float(left_ee[i])
            obs_dict[f"right_tcp_pose.{axis}"] = float(right_ee[i])

        # Compute delta ee_pose (frame-to-frame change)
        if not self._obs_initialized:
            # First frame: delta is zero
            delta_left = np.zeros(6)
            delta_right = np.zeros(6)
            self._obs_initialized = True
        else:
            delta_left = left_ee - self._prev_obs_left_ee
            delta_right = right_ee - self._prev_obs_right_ee
            # Handle angle wrapping for roll/pitch/yaw (indices 3,4,5)
            for j in range(3, 6):
                delta_left[j] = _angle_diff(left_ee[j], self._prev_obs_left_ee[j])
                delta_right[j] = _angle_diff(right_ee[j], self._prev_obs_right_ee[j])

        self._prev_obs_left_ee = left_ee.copy()
        self._prev_obs_right_ee = right_ee.copy()

        for i, axis in enumerate(_EE_AXES):
            obs_dict[f"left_delta_tcp_pose.{axis}"] = float(delta_left[i])
            obs_dict[f"right_delta_tcp_pose.{axis}"] = float(delta_right[i])

        # Gripper state (from Bridge - reads joint 7)
        obs_dict["left_gripper_position"] = float(state["left_arm"]["gripper"])
        obs_dict["right_gripper_position"] = float(state["right_arm"]["gripper"])

        # Chassis state
        obs_dict["chassis_height"] = float(state["chassis"]["height"])
        obs_dict["chassis_head_yaw"] = float(state["chassis"]["head_yaw"])
        obs_dict["chassis_head_pitch"] = float(state["chassis"]["head_pitch"])

        # Capture images from cameras (parallel to reduce latency)
        _t_cam_start = time.perf_counter()
        if self.cameras:
            with ThreadPoolExecutor(max_workers=len(self.cameras)) as pool:
                futures = {pool.submit(cam.read): key for key, cam in self.cameras.items()}
                for future in as_completed(futures):
                    obs_dict[futures[future]] = future.result()
        _t_cam = time.perf_counter()

        # Head camera extrinsic calibration (if enabled)
        if self._calib_enabled:
            head_img = obs_dict.get(self._calib_head_image_key)
            if head_img is not None:
                T_cam_world, num_markers = compute_head_camera_pose(
                    head_img,
                    self._calib_detector,
                    self._calib_board,
                    self._calib_camera_matrix,
                    self._calib_dist_coeffs,
                    self._calib_T_board_to_world,
                )
                if T_cam_world is not None:
                    self._calib_pose_7d = pose_to_7dof(T_cam_world)
            # Write 7-DoF pose to obs_dict
            for i, axis in enumerate(["x", "y", "z", "qx", "qy", "qz", "qw"]):
                obs_dict[f"head_camera_pose.{axis}"] = float(self._calib_pose_7d[i])

        # Record observation timing (will be combined with send_action in perf report)
        self._perf_obs_rpc = _t_rpc - _t0
        self._perf_obs_cam = _t_cam - _t_cam_start

        return obs_dict

    def disconnect(self) -> None:
        """Disconnect from robot."""
        if not self.is_connected:
            return

        # Stop robot movement via Bridge
        if self.bridge is not None:
            logger.info("Stopping robot movement...")
            try:
                self.bridge.set_chassis_velocity(0.0, 0.0, 0.0)
                # Hold arms at current position
                left_pos = self.bridge.get_left_joint_positions()
                right_pos = self.bridge.get_right_joint_positions()
                self.bridge.set_dual_joint_positions(left_pos, right_pos)
                time.sleep(0.2)
            except Exception as e:
                logger.warning(f"Error stopping robot: {e}")

            # Disconnect Bridge
            self.bridge.disconnect()
            self.bridge = None

        # Disconnect cameras
        for cam in self.cameras.values():
            cam.disconnect()

        self.is_connected = False
        logger.info(f"[INFO] ===== All {self.name} connections have been closed =====")

    def calibrate(self) -> None:
        """Calibrate robot (not implemented)."""
        pass

    def is_calibrated(self) -> bool:
        """Check if robot is calibrated."""
        return self.is_connected

    def configure(self) -> None:
        """Configure robot (not implemented)."""
        pass

    def go_home(self, steps: int = 50, delay_sec: float = 0.05) -> None:
        """Move arms to home position with smooth linear interpolation.

        Args:
            steps: Number of interpolation steps (more = slower/smoother)
            delay_sec: Delay between each step in seconds, total time ≈ steps * delay_sec
        """
        if self.bridge is not None and self.is_connected:
            # Get current joint positions from robot
            left_current = np.array(self.bridge.get_left_joint_positions())
            right_current = np.array(self.bridge.get_right_joint_positions())

            # Target home positions from config
            left_target = np.array(self.cfg.left_init_joints)
            right_target = np.array(self.cfg.right_init_joints)

            logger.info(f"[go_home] Smooth interpolation: {steps} steps, total {steps * delay_sec:.1f}s")
            logger.info(f"[go_home] Left current → target: {[round(x, 4) for x in left_current]} → {[round(x, 4) for x in left_target]}")
            logger.info(f"[go_home] Right current → target: {[round(x, 4) for x in right_current]} → {[round(x, 4) for x in right_target]}")

            # Linear interpolation from current to target
            for step in range(1, steps + 1):
                alpha = step / steps  # interpolation factor 0 → 1
                left_interp = left_current * (1 - alpha) + left_target * alpha
                right_interp = right_current * (1 - alpha) + right_target * alpha

                # Send interpolated joint positions
                self.bridge.set_dual_joint_positions(left_interp, right_interp)

                # Wait before next step
                time.sleep(delay_sec)

            # Final step - ensure we reach exact target
            self.bridge.set_dual_joint_positions(left_target, right_target)
            logger.info("[go_home] Done - arms reached home position")

    def move_to_action(self, target_action: dict[str, Any],
                       steps: int = 50, delay_sec: float = 0.05) -> None:
        """Smoothly interpolate arms from current pose to the joint state in ``target_action``.

        For joint position actions (pick_handover): interpolates from current joint
        positions to the target joint positions over ``steps`` steps.

        For delta EE pose actions (fold_cloth): delta has no absolute target, so
        this method skips interpolation and returns immediately.
        """
        if self.bridge is None or not self.is_connected:
            return

        # delta EE pose 没有绝对目标位姿，无法插值，直接跳过
        if "left_joint_1.pos" not in target_action:
            logger.info("[move_to_action] delta EE pose 模式，跳过平滑过渡，从当前位姿直接开始推理")
            return

        # Current joint positions from robot (7 joints per arm, last is SDK-scale gripper)
        left_current = np.array(self.bridge.get_left_joint_positions())
        right_current = np.array(self.bridge.get_right_joint_positions())

        # Target joints from action dict. Joint 7 in the dict may be stale/raw; the
        # normalized gripper_position is the authoritative source - recompute joint 7
        # as gripper_position * 5.0 to match send_action's scaling convention.
        left_target = np.array([target_action[f"left_joint_{i+1}.pos"] for i in range(self._num_joints)])
        right_target = np.array([target_action[f"right_joint_{i+1}.pos"] for i in range(self._num_joints)])
        if "left_gripper_position" in target_action:
            left_target[6] = target_action["left_gripper_position"] * 5.0
        if "right_gripper_position" in target_action:
            right_target[6] = target_action["right_gripper_position"] * 5.0

        logger.info(f"[move_to_action] Smooth interpolation: {steps} steps, total {steps * delay_sec:.1f}s")
        logger.info(f"[move_to_action] Left current → target: {[round(x, 4) for x in left_current]} → {[round(x, 4) for x in left_target]}")
        logger.info(f"[move_to_action] Right current → target: {[round(x, 4) for x in right_current]} → {[round(x, 4) for x in right_target]}")

        # Linear interpolation from current to target
        for step in range(1, steps + 1):
            alpha = step / steps                                                # 0 → 1
            left_interp = left_current * (1 - alpha) + left_target * alpha
            right_interp = right_current * (1 - alpha) + right_target * alpha
            self.bridge.set_dual_joint_positions(left_interp, right_interp)
            time.sleep(delay_sec)

        # Final step - ensure we reach exact target
        self.bridge.set_dual_joint_positions(left_target, right_target)

        # Sync internal gripper tracker so the upcoming send_action's override uses the right value
        if "left_gripper_position" in target_action:
            self._left_gripper_position = target_action["left_gripper_position"]
        if "right_gripper_position" in target_action:
            self._right_gripper_position = target_action["right_gripper_position"]

        logger.info("[move_to_action] Done - arms at first-frame pose")

    def gravity_compensation(self) -> None:
        """
        Enable gravity compensation mode.
        Note: Not directly supported in ROS2 Bridge API.
        Would need to be implemented in the underlying controller nodes.
        """
        logger.warning("Gravity compensation not implemented in ROS2 Bridge interface")

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras}

    @property
    def observation_features(self) -> dict[str, Any]:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def cameras(self):
        return self._cameras

    @cameras.setter
    def cameras(self, value):
        self._cameras = value

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        self._config = value
