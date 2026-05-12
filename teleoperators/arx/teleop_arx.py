#!/usr/bin/env python

"""
ARX VR Teleoperator for LIFT platform with dual R5 arms.
Uses ZeroRPC bridge for all robot communication.

R5/X5lite arms have 6 arm joints + 1 gripper (7 total from SDK).
CAN bus: can1 for left, can3 for right.

Architecture Note:
    This teleoperator receives a robot reference from the main recording
    script via set_robot_reference(). This ensures:
    1. Only one ZeroRPC connection to the robot (in ARXLift2)
    2. Teleop reads current state via robot.bridge
    3. Commands are sent only via robot.send_action() in the record loop

Control Strategy:
    Uses Placo IK solver with dual_R5a.urdf (dual-arm URDF) to convert VR
    controller delta poses into joint angles. Gripper controlled separately
    via VR trigger toggle.
"""

import logging
import time
from pathlib import Path
from typing import Any, Dict, TYPE_CHECKING
import threading
import numpy as np
import placo
from scipy.spatial.transform import Rotation as R

from lerobot.teleoperators.teleoperator import Teleoperator
from .config_teleop_arx import ARXVRTeleopConfig
from .filters import JointRateLimiter, OneEuroFilter, SlerpEMA
from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.hardware.interface.universal_robots import CONTROLLER_DEADZONE
from xrobotoolkit_teleop.utils.geometry import apply_delta_pose, quat_diff_as_angle_axis
import meshcat.transformations as tf

if TYPE_CHECKING:
    from robots.arx import ARXLift2

_EE_AXES = ["x", "y", "z", "roll", "pitch", "yaw"]


def _angle_diff(a: float, b: float) -> float:
    """Compute shortest angular difference (a - b), handling +/- pi wrapping."""
    d = a - b
    return (d + np.pi) % (2 * np.pi) - np.pi

logger = logging.getLogger(__file__)
logger.setLevel(logging.INFO)


DEFAULT_MANIPULATOR_CONFIG = {
    "left_arm": {
        "link_name": "left_link6",
        "pose_source": "left_controller",
        "control_trigger": "left_grip",
        "gripper_trigger": "left_trigger",
    },
    "right_arm": {
        "link_name": "right_link6",
        "pose_source": "right_controller",
        "control_trigger": "right_grip",
        "gripper_trigger": "right_trigger",
    },
}

ARM_MAP = {
    "left_arm": {
        "last": "_last_left_trigger_val",
        "pos": "left_gripper_pos",
    },
    "right_arm": {
        "last": "_last_right_trigger_val",
        "pos": "right_gripper_pos",
    },
}


class ARXVRTeleop(Teleoperator):
    """
    ARX VR Teleop class for controlling dual R5 arms + LIFT chassis.
    Uses ZeroRPC bridge for all robot communication.

    R5/X5lite arms have 6 arm joints + 1 gripper (7 total from SDK).

    Joint structure:
        - Joints 1-6: Arm joints
        - Joint 7: Gripper position (controlled separately via VR trigger)

    Architecture:
        - Receives robot reference via set_robot_reference()
        - Reads current state via robot.bridge (ZeroRPC client)
        - Computes target joint positions
        - Returns action via get_action() - actual send happens in record loop
    """

    config_class = ARXVRTeleopConfig
    name = "ARXVRTeleop"

    def __init__(self, config: ARXVRTeleopConfig):
        super().__init__(config)
        self.xr_client = config.xr_client
        self.cfg = config
        self._is_connected = False
        self._stop_event = threading.Event()
        # R5 has 6 arm joints + 1 gripper (7 total)
        self._num_arm_joints = 6
        self._num_joints = 7

        # Robot reference (set via set_robot_reference before connect)
        self._robot: "ARXLift2" = None
        self.manipulator_config = DEFAULT_MANIPULATOR_CONFIG

        # Placo IK state
        self.placo_robot = None
        self.solver = None
        self.effector_task = {}

        # VR tracking state
        self.init_ee_xyz = {"left_arm": None, "right_arm": None}
        self.init_ee_quat = {"left_arm": None, "right_arm": None}
        self.init_controller_xyz = {"left_arm": None, "right_arm": None}
        self.init_controller_quat = {"left_arm": None, "right_arm": None}

        # Target joint positions
        self.target_left_q = np.zeros(self._num_joints)
        self.target_right_q = np.zeros(self._num_joints)

        # Gripper state
        self._last_left_trigger_val = 1.0
        self._last_right_trigger_val = 1.0
        self.left_gripper_pos = config.open_position
        self.right_gripper_pos = config.open_position

        # Chassis state
        self.target_chassis_vx = 0.0
        self.target_chassis_vy = 0.0
        self.target_chassis_wz = 0.0
        self.target_chassis_height = 0.0
        self._current_height = 0.0

        # Delta tcp_pose tracking (frame-to-frame change in IK target)
        self._prev_left_tcp_pose = np.zeros(6)
        self._prev_right_tcp_pose = np.zeros(6)
        self._tcp_pose_initialized = False

        # Coordinate transformation
        self.R_headset_world = R.from_euler(
            "ZYX", self.cfg.R_headset_world, degrees=True
        ).as_matrix()

        # Per-arm 安装朝向补偿旋转 (绕 Z 轴)
        # 两臂相对安装时需要; 同向安装时 comp_deg=0, 旋转为单位阵, 无影响
        self._left_arm_comp = R.from_euler("z", self.cfg.left_arm_yaw_comp_deg, degrees=True)
        self._right_arm_comp = R.from_euler("z", self.cfg.right_arm_yaw_comp_deg, degrees=True)

        # Smoothing filters (see filters.py for tuning notes).
        # Pose filter: 3 OneEuro per arm for xyz, 1 SlerpEMA per arm for quat.
        self._pose_xyz_filter = {
            arm: [
                OneEuroFilter(
                    min_cutoff=self.cfg.pose_min_cutoff,
                    beta=self.cfg.pose_beta,
                    d_cutoff=self.cfg.pose_d_cutoff,
                )
                for _ in range(3)
            ]
            for arm in self.manipulator_config
        }
        self._pose_quat_filter = {arm: SlerpEMA() for arm in self.manipulator_config}

        # Joint rate limiter: 6 arm joints per arm; dt from teleop fps.
        _dt = 1.0 / max(int(self.cfg.fps), 1)
        self._joint_limiter_left = JointRateLimiter(
            dim=self._num_arm_joints,
            alpha=self.cfg.joint_ema_alpha,
            max_velocity=self.cfg.joint_max_velocity,
            dt=_dt,
        )
        self._joint_limiter_right = JointRateLimiter(
            dim=self._num_arm_joints,
            alpha=self.cfg.joint_ema_alpha,
            max_velocity=self.cfg.joint_max_velocity,
            dt=_dt,
        )

    @property
    def action_features(self) -> dict:
        return {}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        pass

    def set_robot_reference(self, robot: "ARXLift2") -> None:
        """
        Set robot reference for reading current state.

        This method must be called AFTER robot.connect() and BEFORE teleop.connect().
        The teleoperator uses the robot's Bridge to read current state.

        Args:
            robot: The ARXLift2 robot instance (must be connected)
        """
        if not robot.is_connected:
            raise RuntimeError("Robot must be connected before setting reference")
        if robot.bridge is None:
            raise RuntimeError("Robot Bridge not initialized")
        self._robot = robot
        logger.info("[TELEOP] Robot reference set successfully")

    def _check_placo_setup(self):
        """Initialize Placo robot model and IK solver."""
        urdf_path = Path(__file__).parents[2] / self.cfg.robot_urdf_path
        logger.info(f"[TELEOP] Loading URDF from: {urdf_path}")
        self.placo_robot = placo.RobotWrapper(str(urdf_path))
        self.solver = placo.KinematicsSolver(self.placo_robot)
        self.solver.dt = self.cfg.servo_time
        self.solver.mask_fbase(True)
        self.solver.add_kinetic_energy_regularization_task(1e-6)

        # Build state.q index map for revolute joints (single slot per joint)
        self._q_indices = {}  # joint_name -> index in state.q
        for jname in self.placo_robot.joint_names():
            q_before = self.placo_robot.state.q.copy()
            self.placo_robot.set_joint(jname, 0.777)
            q_after = self.placo_robot.state.q.copy()
            self.placo_robot.set_joint(jname, 0.0)
            changed = np.where(np.abs(q_after - q_before) > 1e-10)[0]
            if len(changed) > 0:
                self._q_indices[jname] = int(changed[0])
        self.placo_robot.update_kinematics()

    def _check_endeffector_setup(self):
        """Set up IK frame tasks and manipulability for each arm."""
        for name, config in self.manipulator_config.items():
            initial_pose = np.eye(4)
            self.effector_task[name] = self.solver.add_frame_task(
                config["link_name"], initial_pose
            )
            self.effector_task[name].configure(f"{name}_frame", "soft", 1.0)
            manipulability = self.solver.add_manipulability_task(
                config["link_name"], "both", 1.0
            )
            manipulability.configure(f"{name}_manipulability", "soft", 1e-3)

    def _setup_joints_regularization(self):
        """Lock non-arm joints using soft joints regularization task."""
        joints_task = self.solver.add_joints_task()
        arm_joint_names = set()
        for i in range(1, 7):
            arm_joint_names.add(f"left_joint{i}")
            arm_joint_names.add(f"right_joint{i}")

        q0 = self.placo_robot.state.q
        non_arm_joints = {}
        for joint_name in self.placo_robot.joint_names():
            if joint_name not in arm_joint_names:
                # Revolute/prismatic joint: use zero-config value
                non_arm_joints[joint_name] = q0[self._q_indices[joint_name]]

        joints_task.set_joints(non_arm_joints)
        joints_task.configure("non_arm_regularization", "soft", 1e-4)

    def _set_placo_joint(self, joint_name: str, angle: float):
        """Set a revolute joint in Placo."""
        idx = self._q_indices[joint_name]
        self.placo_robot.state.q[idx] = angle

    def _get_placo_joint(self, joint_name: str) -> float:
        """Get angle of a revolute joint from Placo."""
        idx = self._q_indices[joint_name]
        return self.placo_robot.state.q[idx]

    def _sync_placo_from_bridge(self):
        """Sync Placo model joint state from Bridge (real robot)."""
        # 使用get_full_state一次获取所有状态，避免多次调用导致协程错误
        try:
            state = self._robot.bridge.get_full_state()
            if state is not None:
                left_pos = state["left_arm"]["joint_positions"]
                right_pos = state["right_arm"]["joint_positions"]
                for i in range(6):
                    self._set_placo_joint(f"left_joint{i+1}", left_pos[i])
                    self._set_placo_joint(f"right_joint{i+1}", right_pos[i])
                self.placo_robot.update_kinematics()
                return left_pos, right_pos
        except Exception as e:
            logger.warning(f"Failed to sync from bridge: {e}")

        # 如果获取失败，返回零位置
        return np.zeros(7), np.zeros(7)

    def _sync_placo_from_state(self, state_dict):
        """Sync Placo model from a pre-fetched state dict (avoids redundant RPC call).

        Same logic as _sync_placo_from_bridge but uses an already-fetched state dict
        instead of making another ZeroRPC call.
        """
        try:
            if state_dict is not None:
                left_pos = state_dict["left_arm"]["joint_positions"]
                right_pos = state_dict["right_arm"]["joint_positions"]
                for i in range(6):
                    self._set_placo_joint(f"left_joint{i+1}", left_pos[i])
                    self._set_placo_joint(f"right_joint{i+1}", right_pos[i])
                self.placo_robot.update_kinematics()
                return left_pos, right_pos
        except Exception as e:
            logger.warning(f"Failed to sync from cached state: {e}")

        return np.zeros(7), np.zeros(7)

    def _read_placo_arm_joints(self):
        """Read solved arm joint angles from Placo model."""
        left_q = np.array([
            self._get_placo_joint(f"left_joint{i+1}") for i in range(6)
        ])
        right_q = np.array([
            self._get_placo_joint(f"right_joint{i+1}") for i in range(6)
        ])
        return left_q, right_q

    def connect(self) -> None:
        """
        Start VR update thread.

        Requires set_robot_reference() to be called first.
        Uses robot's ZeroRPC bridge for state reading.
        """
        if self._robot is None:
            raise RuntimeError(
                "Robot reference not set. Call set_robot_reference(robot) before connect()"
            )

        logger.info("\n===== [TELEOP] Starting VR Teleoperator =====")
        logger.info("[TELEOP] Using robot's ZeroRPC Bridge for state reading")

        # Initialize Placo IK solver
        self._check_placo_setup()
        self._check_endeffector_setup()
        self._setup_joints_regularization()

        # Initialize target joint positions from current state
        self._init_qpos()

        # Start VR update thread
        threading.Thread(target=self._vr_update_loop, daemon=True).start()

        self._is_connected = True
        logger.info(f"[INFO] {self.name} initialization completed successfully.\n")

    def _init_qpos(self):
        """Initialize target joint positions from current robot state via Bridge."""
        left_pos, right_pos = self._sync_placo_from_bridge()
        self.target_left_q = np.array(left_pos)   # 7 values (6 arm + 1 gripper)
        self.target_right_q = np.array(right_pos)

        # Seed joint rate limiters with current state so the first commanded
        # frame doesn't get clamped/lagged from a zero initial value.
        self._joint_limiter_left.reset(self.target_left_q[:self._num_arm_joints])
        self._joint_limiter_right.reset(self.target_right_q[:self._num_arm_joints])

        # Set initial IK targets to current EE poses
        for name, config in self.manipulator_config.items():
            T_current = self.placo_robot.get_T_world_frame(config["link_name"])
            self.effector_task[name].T_world_frame = T_current

        # Initialize chassis height (only when LIFT is enabled)
        if self.cfg.enable_lift:
            try:
                state = self._robot.bridge.get_full_state()
                if state is not None and "chassis" in state:
                    self._current_height = state["chassis"]["height"]
                    self.target_chassis_height = self._current_height
            except Exception as e:
                logger.warning(f"Failed to get chassis height: {e}")
                self._current_height = 0.24
                self.target_chassis_height = 0.24

    def _vr_update_loop(self):
        """Background thread: read VR input and update targets.

        VR thread only computes target joint positions (via Placo IK).
        All RPC communication happens in the main thread via send_action().
        """
        _vr_count = 0
        _vr_sums = {"vr_input": 0.0, "ik_solve": 0.0, "total": 0.0}
        _vr_maxs = {"vr_input": 0.0, "ik_solve": 0.0, "total": 0.0}

        while not self._stop_event.is_set():
            try:
                t0 = time.perf_counter()
                self._update_from_vr()
                t1 = time.perf_counter()

                elapsed = t1 - t0
                time.sleep(max(0, 1 / self.cfg.fps - elapsed))
                total = time.perf_counter() - t0

                # Collect timing
                _vr_count += 1
                vr_in = getattr(self, '_perf_vr_input', 0.0)
                ik = getattr(self, '_perf_ik_solve', 0.0)
                for k, v in [("vr_input", vr_in), ("ik_solve", ik), ("total", total)]:
                    _vr_sums[k] += v
                    _vr_maxs[k] = max(_vr_maxs[k], v)

                if _vr_count % 200 == 0:
                    actual_hz = 200 / _vr_sums["total"] if _vr_sums["total"] > 0 else 0
                    parts = []
                    for k in ("vr_input", "ik_solve"):
                        avg = _vr_sums[k] / 200 * 1000
                        mx = _vr_maxs[k] * 1000
                        parts.append(f"{k}={avg:.1f}/{mx:.1f}ms")
                    logger.info(
                        f"[PERF vr_thread] {_vr_count} frames | "
                        f"actual {actual_hz:.1f}Hz | " + " | ".join(parts)
                    )
                    _vr_sums = {k: 0.0 for k in _vr_sums}
                    _vr_maxs = {k: 0.0 for k in _vr_maxs}

            except Exception as e:
                logger.error(f"Error in VR update thread: {e}")

    def _update_from_vr(self):
        """Process VR input and update robot targets via Placo IK.

        VR thread only reads VR data, runs IK, and writes target variables.
        No ZeroRPC calls here — all RPC happens in the main thread.
        """
        _t0 = time.perf_counter()

        # 1. Process each arm (set IK targets from VR input)
        for arm_name, config in self.manipulator_config.items():
            self._process_arm_vr(arm_name, config)

        # 2. Process chassis (only when LIFT is enabled)
        if self.cfg.enable_lift:
            self._process_chassis_vr()

        _t1 = time.perf_counter()
        self._perf_vr_input = _t1 - _t0

        # 3. Solve IK (one solve for both arms)
        try:
            self.solver.solve(True)
            self.placo_robot.update_kinematics()
            left_solved, right_solved = self._read_placo_arm_joints()

            # Joint-space smoothing (EMA + per-joint velocity clamp) to
            # absorb single-frame IK jumps before commanding the robot.
            if self.cfg.enable_joint_filter:
                left_solved = self._joint_limiter_left.step(left_solved)
                right_solved = self._joint_limiter_right.step(right_solved)

            # Update arm joints (indices 0-5), keep gripper at index 6
            self.target_left_q[:6] = left_solved
            self.target_right_q[:6] = right_solved
        except RuntimeError as e:
            logger.warning(f"IK solver failed: {e}. Keeping last good positions.")

        self._perf_ik_solve = time.perf_counter() - _t1

    def _process_arm_vr(self, arm_name: str, config: dict):
        """
        Process VR input for one arm: gripper toggle + IK target setting.

        Sets IK targets on self.effector_task[arm_name]. The actual IK solve
        happens in _update_from_vr() after all arms are processed.
        """
        # Check grip trigger for arm activation
        xr_grip_val = self.xr_client.get_key_value_by_name(config["control_trigger"])
        active = xr_grip_val > (1.0 - CONTROLLER_DEADZONE)

        # Process gripper trigger — 直接映射: 按住=闭合, 松开=张开
        # raw: 0=松开, 1=按下
        raw_trigger = self.xr_client.get_key_value_by_name(config["gripper_trigger"])
        is_pressed = raw_trigger > self.cfg.trigger_threshold

        pos_attr = ARM_MAP[arm_name]["pos"]
        if self.cfg.trigger_reverse:
            # reverse: 按下=张开, 松开=闭合
            setattr(self, pos_attr, self.cfg.open_position if is_pressed else self.cfg.close_position)
        else:
            # normal: 按下=闭合, 松开=张开
            setattr(self, pos_attr, self.cfg.close_position if is_pressed else self.cfg.open_position)

        # Set IK target from VR input
        arm_side = "left" if arm_name == "left_arm" else "right"

        if active:
            if self.init_ee_xyz[arm_name] is None:
                # First activation: get initial EE pose from Placo FK
                T_world_ee = self.placo_robot.get_T_world_frame(config["link_name"])
                self.init_ee_xyz[arm_name] = T_world_ee[:3, 3].copy()
                self.init_ee_quat[arm_name] = tf.quaternion_from_matrix(T_world_ee)
                logger.info(f"[{arm_name}] 激活IK控制，初始位置: {self.init_ee_xyz[arm_name]}")

            # Get VR controller pose delta
            xr_pose = self.xr_client.get_pose_by_name(config["pose_source"])
            delta_xyz, delta_rot = self._process_xr_pose(xr_pose, arm_name)

            # 臂安装朝向补偿: 绕 Z 轴旋转 delta (相对安装时生效, 同向安装时为单位旋转)
            comp = self._left_arm_comp if arm_name == "left_arm" else self._right_arm_comp
            delta_xyz = comp.apply(delta_xyz)
            delta_rot = comp.apply(delta_rot)

            # Apply delta to get target EE pose
            target_xyz, target_quat = apply_delta_pose(
                self.init_ee_xyz[arm_name],
                self.init_ee_quat[arm_name],
                delta_xyz,
                delta_rot,
            )

            # Set IK target
            target_transform = tf.quaternion_matrix(target_quat)
            target_transform[:3, 3] = target_xyz
            self.effector_task[arm_name].T_world_frame = target_transform

        else:
            # Not active: reset tracking state
            if self.init_ee_xyz[arm_name] is not None:
                self.init_ee_xyz[arm_name] = None
                self.init_ee_quat[arm_name] = None
                self.init_controller_xyz[arm_name] = None
                self.init_controller_quat[arm_name] = None
                logger.info(f"[{arm_name}] 退出IK控制，保持当前位置")
                # Hold current pose as IK target
                T_current = self.placo_robot.get_T_world_frame(config["link_name"])
                self.effector_task[arm_name].T_world_frame = T_current

            # Update target from real state (for gripper value at index 6)
            # 注释掉这部分，避免VR线程直接调用ZeroRPC
            # try:
            #     state = self._robot.bridge.get_full_state()
            #     if state is not None:
            #         if arm_side == "left":
            #             self.target_left_q = np.array(
            #                 state["left_arm"]["joint_positions"]
            #             )
            #         else:
            #             self.target_right_q = np.array(
            #                 state["right_arm"]["joint_positions"]
            #             )
            # except Exception as e:
            #     logger.warning(f"Failed to update from real state: {e}")
            # 保持当前的target_q不变，不调用ZeroRPC

    def _process_xr_pose(self, xr_pose, arm_name: str):
        """Process VR controller pose to get delta movement."""
        controller_xyz = np.array([xr_pose[0], xr_pose[1], xr_pose[2]])
        controller_quat = np.array(
            [
                xr_pose[6],
                xr_pose[3],
                xr_pose[4],
                xr_pose[5],
            ]
        )

        controller_xyz = self.R_headset_world @ controller_xyz

        R_transform = np.eye(4)
        R_transform[:3, :3] = self.R_headset_world
        R_quat = tf.quaternion_from_matrix(R_transform)
        controller_quat = tf.quaternion_multiply(
            tf.quaternion_multiply(R_quat, controller_quat),
            tf.quaternion_conjugate(R_quat),
        )

        # Pose smoothing: One-Euro on xyz, slerp-EMA on quat. Reset filters
        # on activation so the init pose matches the smoothed stream and
        # there's no cold-start step.
        if self.cfg.enable_pose_filter:
            xyz_filters = self._pose_xyz_filter[arm_name]
            quat_filter = self._pose_quat_filter[arm_name]
            if self.init_controller_xyz[arm_name] is None:
                t_now = time.perf_counter()
                for i, f in enumerate(xyz_filters):
                    f.reset(float(controller_xyz[i]), t_now)
                quat_filter.reset(controller_quat)
            else:
                t_now = time.perf_counter()
                controller_xyz = np.array(
                    [xyz_filters[i](float(controller_xyz[i]), t_now) for i in range(3)]
                )
                controller_quat = quat_filter.step(
                    controller_quat, self.cfg.quat_slerp_alpha
                )

        if self.init_controller_xyz[arm_name] is None:
            self.init_controller_xyz[arm_name] = controller_xyz.copy()
            self.init_controller_quat[arm_name] = controller_quat.copy()
            delta_xyz = np.zeros(3)
            delta_rot = np.array([0.0, 0.0, 0.0])
        else:
            delta_xyz = (
                controller_xyz - self.init_controller_xyz[arm_name]
            ) * self.cfg.scale_factor
            delta_rot = quat_diff_as_angle_axis(
                self.init_controller_quat[arm_name], controller_quat
            )

        return delta_xyz, delta_rot

    def _process_chassis_vr(self):
        """Process VR joystick input for chassis control."""
        try:
            # 使用正确的get_joystick_state方法，需要传入"left"或"right"参数
            if hasattr(self.xr_client, 'get_joystick_state'):
                # 分别获取左、右摇杆的状态
                left_joy = self.xr_client.get_joystick_state("left")
                right_joy = self.xr_client.get_joystick_state("right")

                left_joy_x = left_joy[0]
                left_joy_y = left_joy[1]
                right_joy_x = right_joy[0]
                right_joy_y = right_joy[1]
            else:
                left_joy_x = left_joy_y = right_joy_x = right_joy_y = 0.0

            # Apply deadzone
            deadzone = 0.1
            if abs(left_joy_x) < deadzone:
                left_joy_x = 0.0
            if abs(left_joy_y) < deadzone:
                left_joy_y = 0.0
            if abs(right_joy_x) < deadzone:
                right_joy_x = 0.0
            if abs(right_joy_y) < deadzone:
                right_joy_y = 0.0

            # Map to chassis commands
            self.target_chassis_vx = left_joy_y * self.cfg.chassis_vx_scale
            self.target_chassis_vy = left_joy_x * self.cfg.chassis_vy_scale
            self.target_chassis_wz = right_joy_x * self.cfg.chassis_wz_scale

            # Height control (incremental)
            height_delta = right_joy_y * self.cfg.chassis_height_scale
            self.target_chassis_height = max(
                0.0, min(1.0, self._current_height + height_delta)
            )
            self._current_height = self.target_chassis_height
        except Exception as e:
            # 如果控制器不支持摇杆，跳过底盘控制
            logger.warning(f"Chassis control error: {e}")
            self.target_chassis_vx = 0.0
            self.target_chassis_vy = 0.0
            self.target_chassis_wz = 0.0
            # 保持当前高度不变
            self.target_chassis_height = self._current_height

    def reset_tracking(self):
        """Reset VR tracking state and sync targets from current robot position."""
        for arm_name in self.manipulator_config.keys():
            self.init_ee_xyz[arm_name] = None
            self.init_ee_quat[arm_name] = None
            self.init_controller_xyz[arm_name] = None
            self.init_controller_quat[arm_name] = None
            # Pose filters get re-seeded on next activation; clearing here
            # avoids stale state if config flags get toggled at runtime.
            for f in self._pose_xyz_filter[arm_name]:
                f.reset()
            self._pose_quat_filter[arm_name].reset()

        # Reset delta tcp_pose tracking
        self._prev_left_tcp_pose = np.zeros(6)
        self._prev_right_tcp_pose = np.zeros(6)
        self._tcp_pose_initialized = False

        # 同步目标关节到当前机器人位置，防止新 episode 第一帧跳变
        # (also re-seeds joint rate limiters to current state)
        self._init_qpos()

    def calibrate(self) -> None:
        pass

    def configure(self):
        pass

    def get_action(self) -> dict[str, Any]:
        """Return current action targets.

        注意: get_action()在主线程中调用，所以可以安全地调用ZeroRPC方法！
        我们在这里同步Placo模型和真实机器人的状态。
        """
        # 同步Placo模型：优先使用get_observation()缓存的状态，避免重复RPC调用
        try:
            cached = getattr(self._robot, '_last_state', None)
            if cached is not None:
                self._sync_placo_from_state(cached)
            else:
                self._sync_placo_from_bridge()
        except Exception as e:
            logger.warning(f"Failed to sync Placo model: {e}")

        action = {}

        # Arm joint positions
        for i in range(self._num_joints):
            action[f"left_joint_{i+1}.pos"] = self.target_left_q[i]
            action[f"right_joint_{i+1}.pos"] = self.target_right_q[i]

        # Gripper positions
        action["left_gripper_position"] = self.left_gripper_pos
        action["right_gripper_position"] = self.right_gripper_pos

        # EE pose targets (commanded target from Placo IK) + delta computation
        for arm_name in self.manipulator_config:
            prefix = "left" if arm_name == "left_arm" else "right"
            T = self.effector_task[arm_name].T_world_frame
            xyz = T[:3, 3]
            rpy = R.from_matrix(T[:3, :3]).as_euler('xyz')
            current_tcp = np.array([*xyz, *rpy])

            for i, axis in enumerate(_EE_AXES):
                action[f"{prefix}_tcp_pose.{axis}"] = float(current_tcp[i])

            # Delta tcp_pose (frame-to-frame change in IK target)
            prev_tcp = self._prev_left_tcp_pose if prefix == "left" else self._prev_right_tcp_pose
            if not self._tcp_pose_initialized:
                delta = np.zeros(6)
            else:
                delta = current_tcp - prev_tcp
                for j in range(3, 6):  # angle wrapping for roll/pitch/yaw
                    delta[j] = _angle_diff(current_tcp[j], prev_tcp[j])

            for i, axis in enumerate(_EE_AXES):
                action[f"{prefix}_delta_tcp_pose.{axis}"] = float(delta[i])

            if prefix == "left":
                self._prev_left_tcp_pose = current_tcp.copy()
            else:
                self._prev_right_tcp_pose = current_tcp.copy()

        self._tcp_pose_initialized = True

        # Chassis commands (only when LIFT is enabled)
        if self.cfg.enable_lift:
            action["chassis_vx"] = self.target_chassis_vx
            action["chassis_vy"] = self.target_chassis_vy
            action["chassis_wz"] = self.target_chassis_wz
            action["chassis_height"] = self.target_chassis_height

        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        """Disconnect teleoperator."""
        if not self.is_connected:
            return

        self._stop_event.set()
        self._is_connected = False
        logger.info(f"[INFO] ===== All {self.name} connections have been closed =====")
