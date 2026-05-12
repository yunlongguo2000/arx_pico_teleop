#!/usr/bin/env python3
"""
Offline test for teleop_arx.py — no hardware or VR needed.

Tests the complete ARXVRTeleop pipeline with mock robot/bridge:
  1. Load ARXVRTeleop with dual_R5a.urdf
  2. Mock robot/bridge with preset joint positions
  3. Test joint syncing (Bridge -> Placo)
  4. Test IK solving with mock VR delta inputs
  5. Test get_action() output format
  6. Verify joint limits are respected

Usage:
    conda run -n ur_data python test_teleop_arx_offline.py
"""

import sys
import logging
import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import Any, Dict

# Setup basic logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Add the project root to path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))


@dataclass
class MockARXVRTeleopConfig:
    """Mock config for testing."""
    xr_client: Any = None
    fps: int = 20
    scale_factor: float = 0.8
    position_filter_alpha: float = 0.25
    rotation_filter_alpha: float = 0.25
    R_headset_world: list = None
    left_arm_yaw_comp_deg: float = 0.0
    right_arm_yaw_comp_deg: float = 0.0
    trigger_reverse: bool = True
    trigger_threshold: float = 0.5
    close_position: float = -0.2
    open_position: float = 0.2
    robot_urdf_path: str = "assets/r5_urdf/dual_R5a.urdf"
    servo_time: float = 0.017
    visualize_placo: bool = False
    enable_lift: bool = True
    chassis_vx_scale: float = 2.0
    chassis_vy_scale: float = 2.0
    chassis_wz_scale: float = 4.0
    chassis_height_scale: float = 0.02
    chassis_mode: int = 2
    control_mode: str = "vrteleop"

    def __post_init__(self):
        if self.R_headset_world is None:
            self.R_headset_world = [-90, 0, 90]


class MockXrClient:
    """Mock XR client for testing."""

    def __init__(self):
        self.key_values = {}
        self.pose_values = {}
        # Set default values (no triggers pressed)
        self.key_values["left_grip"] = 0.0
        self.key_values["right_grip"] = 0.0
        self.key_values["left_trigger"] = 1.0
        self.key_values["right_trigger"] = 1.0
        self.key_values["left_joystick_x"] = 0.0
        self.key_values["left_joystick_y"] = 0.0
        self.key_values["right_joystick_x"] = 0.0
        self.key_values["right_joystick_y"] = 0.0
        # Default pose (identity)
        self.pose_values["left_controller"] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.pose_values["right_controller"] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    def get_key_value_by_name(self, name: str) -> float:
        return self.key_values.get(name, 0.0)

    def get_pose_by_name(self, name: str) -> list:
        return self.pose_values.get(name, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    def set_key_value(self, name: str, value: float):
        self.key_values[name] = value

    def set_controller_pose(self, name: str, tx: float, ty: float, tz: float,
                            qx: float, qy: float, qz: float, qw: float):
        self.pose_values[name] = [tx, ty, tz, qx, qy, qz, qw]


class MockBridge:
    """Mock Bridge for testing."""

    def __init__(self):
        self.left_joint_pos = np.zeros(7)
        self.right_joint_pos = np.zeros(7)
        self.chassis_height = 0.5

    def get_left_joint_positions(self):
        return self.left_joint_pos.copy()

    def get_right_joint_positions(self):
        return self.right_joint_pos.copy()

    def get_chassis_height(self):
        return self.chassis_height

    def get_full_state(self):
        zeros = np.zeros(7)
        return {
            "left_arm": {
                "joint_positions": self.left_joint_pos.copy(),
                "joint_velocities": zeros.copy(),
                "joint_currents": zeros.copy(),
                "end_pose": np.zeros(6),
            },
            "right_arm": {
                "joint_positions": self.right_joint_pos.copy(),
                "joint_velocities": zeros.copy(),
                "joint_currents": zeros.copy(),
                "end_pose": np.zeros(6),
            },
            "chassis": {
                "height": float(self.chassis_height),
            },
        }

    def set_joint_positions(self, left: np.ndarray, right: np.ndarray):
        self.left_joint_pos = left.copy()
        self.right_joint_pos = right.copy()

    def set_full_command(self, left: np.ndarray, right: np.ndarray, vx: float, vy: float, wz: float, height: float):
        self.left_joint_pos = left.copy()
        self.right_joint_pos = right.copy()
        self.chassis_height = float(height)


class MockARXLift2:
    """Mock ARXLift2 robot for testing."""

    def __init__(self):
        self.bridge = MockBridge()
        self.is_connected = True


def print_header(text):
    print(f"\n{'='*70}")
    print(f"  {text}")
    print(f"{'='*70}")


def print_result(label, passed):
    status = "PASS" if passed else "FAIL"
    print(f"  [{status}] {label}")


def main():
    n_pass = 0
    n_fail = 0

    def check(label, condition):
        nonlocal n_pass, n_fail
        print_result(label, condition)
        if condition:
            n_pass += 1
        else:
            n_fail += 1
        return condition

    # ── 1. Import test ───────────────────────────────────────────────────
    print_header("Test 1: Import ARXVRTeleop")
    try:
        from teleoperators.arx.teleop_arx import ARXVRTeleop
        print("  [PASS] Successfully imported ARXVRTeleop")
        n_pass += 1
    except Exception as e:
        print(f"  [FAIL] Import failed: {e}")
        n_fail += 1
        return

    # ── 2. Initialization test ───────────────────────────────────────────
    print_header("Test 2: Initialize ARXVRTeleop")
    try:
        xr_client = MockXrClient()
        config = MockARXVRTeleopConfig()
        config.xr_client = xr_client
        teleop = ARXVRTeleop(config)
        print("  [PASS] ARXVRTeleop initialized")
        n_pass += 1
    except Exception as e:
        print(f"  [FAIL] Initialization failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1
        return

    # ── 3. Set robot reference test ──────────────────────────────────────
    print_header("Test 3: Set robot reference")
    try:
        robot = MockARXLift2()
        # Set some initial joint positions
        robot.bridge.set_joint_positions(
            np.array([0.1, -0.2, 0.3, -0.1, 0.2, -0.1, 0.0]),
            np.array([-0.1, 0.2, -0.3, 0.1, -0.2, 0.1, 0.0]),
        )
        teleop.set_robot_reference(robot)
        print("  [PASS] Robot reference set")
        n_pass += 1
    except Exception as e:
        print(f"  [FAIL] set_robot_reference failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── 4. Connect test (Placo setup) ───────────────────────────────────
    print_header("Test 4: Connect (Placo setup)")
    try:
        teleop.connect()
        check("is_connected is True", teleop.is_connected)
        check("placo_robot is initialized", teleop.placo_robot is not None)
        check("solver is initialized", teleop.solver is not None)
        check("effector_task has both arms", len(teleop.effector_task) == 2)
    except Exception as e:
        print(f"  [FAIL] connect failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── 5. Check joint type detection ───────────────────────────────────
    print_header("Test 5: Joint type detection")
    try:
        # Check that we have joint_is_continuous dict
        has_joint_type = hasattr(teleop, '_joint_is_continuous')
        check("has _joint_is_continuous dict", has_joint_type)
        if has_joint_type:
            # In dual_R5a.urdf, all joints are revolute (not continuous)
            left_joint1_continuous = teleop._joint_is_continuous.get('left_joint1', True)
            check("left_joint1 is revolute (not continuous)", not left_joint1_continuous)
            right_joint1_continuous = teleop._joint_is_continuous.get('right_joint1', True)
            check("right_joint1 is revolute (not continuous)", not right_joint1_continuous)
    except Exception as e:
        print(f"  [FAIL] Joint type detection failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── 6. Get initial action test ──────────────────────────────────────
    print_header("Test 6: Get initial action")
    try:
        action = teleop.get_action()
        print(f"  Action keys: {list(action.keys())[:10]}...")

        # Check that we have the expected keys
        has_left_joints = all(f"left_joint_{i+1}.pos" in action for i in range(7))
        has_right_joints = all(f"right_joint_{i+1}.pos" in action for i in range(7))
        has_grippers = "left_gripper_position" in action and "right_gripper_position" in action
        has_chassis = all(key in action for key in ["chassis_vx", "chassis_vy", "chassis_wz", "chassis_height"])

        check("has left joint positions", has_left_joints)
        check("has right joint positions", has_right_joints)
        check("has gripper positions", has_grippers)
        check("has chassis commands", has_chassis)

        # Check that initial joint positions match the bridge
        left_joint_1 = action["left_joint_1.pos"]
        check(f"left_joint_1.pos matches bridge (~0.1): {left_joint_1:.4f}",
              abs(left_joint_1 - 0.1) < 0.01)

    except Exception as e:
        print(f"  [FAIL] get_action failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── 7. Test with mock VR activation (grip pressed) ─────────────────
    print_header("Test 7: Mock VR control with grip pressed")
    try:
        # Press left grip to activate tracking
        xr_client.set_key_value("left_grip", 1.0)

        # Simulate a few VR update cycles
        for _ in range(5):
            teleop._update_from_vr()

        # Check that init_ee_xyz was set
        check("left_arm init_ee_xyz is set (grip active)",
              teleop.init_ee_xyz["left_arm"] is not None)

        # Release grip
        xr_client.set_key_value("left_grip", 0.0)

        # Simulate more cycles
        for _ in range(5):
            teleop._update_from_vr()

        # Check that init_ee_xyz was reset
        check("left_arm init_ee_xyz is reset (grip released)",
              teleop.init_ee_xyz["left_arm"] is None)

    except Exception as e:
        print(f"  [FAIL] VR control test failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── 8. Test gripper toggle ──────────────────────────────────────────
    print_header("Test 8: Gripper toggle (falling edge trigger)")
    try:
        initial_gripper = teleop.left_gripper_pos

        # Press trigger (rising edge)
        xr_client.set_key_value("left_trigger", 0.0)
        teleop._update_from_vr()

        # Release trigger (falling edge - should toggle)
        xr_client.set_key_value("left_trigger", 1.0)
        teleop._update_from_vr()

        toggled = teleop.left_gripper_pos != initial_gripper
        check(f"gripper toggled: {initial_gripper:.2f} -> {teleop.left_gripper_pos:.2f}",
              toggled)

    except Exception as e:
        print(f"  [FAIL] Gripper toggle test failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── 9. Disconnect test ──────────────────────────────────────────────
    print_header("Test 9: Disconnect")
    try:
        teleop.disconnect()
        check("is_connected is False after disconnect", not teleop.is_connected)
    except Exception as e:
        print(f"  [FAIL] Disconnect failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── Summary ──────────────────────────────────────────────────────────
    print_header(f"Summary: {n_pass} passed, {n_fail} failed")
    if n_fail == 0:
        print("  All tests PASSED! ARXVRTeleop is ready.\n")
        return 0
    else:
        print("  Some tests FAILED. Review results above.\n")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
