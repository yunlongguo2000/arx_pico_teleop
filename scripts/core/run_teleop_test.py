"""
Pure VR Teleoperation Test Script (No Cameras, No Recording).

Tests bidirectional VR teleop fluency:
  VR controller → Placo IK → joint commands → robot arms

Usage:
  # After RPC server is running (./start_rpc_server.sh)
  conda run -n ur_data python scripts/core/run_teleop_test.py

  # Or via the one-stop launcher:
  ./start_teleop_test.sh
"""

import yaml
import time
import signal
import sys
import threading
import logging
from pathlib import Path

import numpy as np
from xrobotoolkit_teleop.common.xr_client import XrClient
from robots.arx import ARXConfig, ARXLift2
from teleoperators.arx import ARXVRTeleopConfig, ARXVRTeleop

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)

# ── Smooth reset (reused from run_record_arx.py) ──

def reset_to_init_position(cfg, robot, teleop, first_time=True, duration=3.0):
    """Smoothly interpolate to initial joint positions."""
    target_left = np.array(cfg["robot"]["left_init_joints"], dtype=np.float64)
    target_right = np.array(cfg["robot"]["right_init_joints"], dtype=np.float64)

    try:
        state = robot.bridge.get_full_state()
        current_left = np.array(state["left_arm"]["joint_positions"], dtype=np.float64)
        current_right = np.array(state["right_arm"]["joint_positions"], dtype=np.float64)
    except Exception as e:
        logger.warning(f"Cannot read state, jumping directly: {e}")
        robot.bridge.set_dual_joint_positions(target_left, target_right)
        time.sleep(1.0)
        if not first_time:
            teleop.reset_tracking()
        return

    hz = 50
    steps = int(duration * hz)
    logger.info(f"Resetting to init position ({duration}s)...")
    for i in range(1, steps + 1):
        t = i / steps
        alpha = t * t * (3.0 - 2.0 * t)  # smoothstep
        interp_left = current_left + alpha * (target_left - current_left)
        interp_right = current_right + alpha * (target_right - current_right)
        robot.bridge.set_dual_joint_positions(interp_left, interp_right)
        time.sleep(1.0 / hz)

    robot.bridge.set_dual_joint_positions(target_left, target_right)
    if not first_time:
        teleop.reset_tracking()
    logger.info("Done.")


def main():
    # ── Load config ──
    cfg_path = Path(__file__).resolve().parent.parent / "config" / "cfg_arx.yaml"
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)["record"]

    fps = cfg.get("fps", 20)
    enable_lift = cfg["robot"].get("enable_lift", False)
    teleop_cfg = cfg["teleop"]
    gripper_cfg = teleop_cfg["gripper"]
    chassis_cfg = teleop_cfg["chassis"]
    placo_cfg = teleop_cfg["placo"]
    filter_cfg = teleop_cfg.get("filter", {})

    # ── Init XrClient ──
    logger.info("Connecting to VR controller...")
    xr_client = XrClient()

    # ── Init teleop (Placo IK) ──
    teleop_config = ARXVRTeleopConfig(
        xr_client=xr_client,
        fps=fps,
        scale_factor=teleop_cfg["scale_factor"],
        position_filter_alpha=filter_cfg.get("position_alpha", 1.0),
        rotation_filter_alpha=filter_cfg.get("rotation_alpha", 1.0),
        R_headset_world=teleop_cfg["R_headset_world"],
        trigger_reverse=gripper_cfg["trigger_reverse"],
        trigger_threshold=gripper_cfg["trigger_threshold"],
        close_position=gripper_cfg["close_position"],
        open_position=gripper_cfg["open_position"],
        enable_lift=enable_lift,
        chassis_vx_scale=chassis_cfg["vx_scale"],
        chassis_vy_scale=chassis_cfg["vy_scale"],
        chassis_wz_scale=chassis_cfg["wz_scale"],
        chassis_height_scale=chassis_cfg["height_scale"],
        chassis_mode=chassis_cfg.get("mode", 2),
        control_mode=teleop_cfg.get("control_mode", "vrteleop"),
        robot_urdf_path=placo_cfg["robot_urdf_path"],
        servo_time=placo_cfg["servo_time"],
        visualize_placo=placo_cfg["visualize_placo"],
    )
    teleop = ARXVRTeleop(teleop_config)

    # ── Init robot (NO cameras) ──
    robot_cfg = cfg["robot"]
    robot_gripper = robot_cfg["gripper"]
    robot_config = ARXConfig(
        left_can=robot_cfg["left_can"],
        right_can=robot_cfg["right_can"],
        arm_type=robot_cfg["arm_type"],
        enable_lift=enable_lift,
        num_joints=7,
        dt=robot_cfg.get("dt", 0.05),
        left_init_joints=robot_cfg["left_init_joints"],
        right_init_joints=robot_cfg["right_init_joints"],
        use_gripper=robot_gripper["use_gripper"],
        gripper_close=robot_gripper["close_position"],
        gripper_open=robot_gripper["open_position"],
        debug=False,
        cameras={},  # <-- NO cameras
    )
    robot = ARXLift2(robot_config)

    # ── Connect ──
    logger.info("Connecting to RPC server (localhost:4242)...")
    robot.connect()
    teleop.set_robot_reference(robot)
    reset_to_init_position(cfg, robot, teleop, first_time=True)
    teleop.connect()

    # ── VR button listener (A=exit, X=exit program) ──
    stop_event = threading.Event()

    def vr_listener():
        while not stop_event.is_set():
            try:
                if xr_client.get_button_state_by_name("X"):
                    logger.info("X button pressed — exiting")
                    stop_event.set()
            except Exception:
                pass
            time.sleep(0.1)

    threading.Thread(target=vr_listener, daemon=True).start()

    # ── Graceful shutdown ──
    def on_sigint(sig, frame):
        logger.info("\nCtrl+C — stopping...")
        stop_event.set()

    signal.signal(signal.SIGINT, on_sigint)

    # ── Main teleop loop ──
    dt = 1.0 / fps
    loop_count = 0
    t_start = time.perf_counter()

    logger.info("")
    logger.info("=" * 50)
    logger.info("  VR Teleop Test Running")
    logger.info(f"  FPS: {fps}  |  LIFT: {enable_lift}")
    logger.info("  Grip buttons = activate arm IK")
    logger.info("  Triggers = gripper open/close")
    logger.info("  X button or Ctrl+C = exit")
    logger.info("=" * 50)
    logger.info("")

    try:
        while not stop_event.is_set():
            t0 = time.perf_counter()

            # 1. Get teleop action (VR → IK → joint targets)
            action = teleop.get_action()

            # 2. Send to robot
            robot.send_action(action)

            # 3. Read observation (for state sync, no cameras)
            robot.get_observation()

            # 4. Rate limit
            elapsed = time.perf_counter() - t0
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

            loop_count += 1

    finally:
        stop_event.set()
        logger.info("Disconnecting...")
        teleop.disconnect()
        robot.disconnect()
        total = time.perf_counter() - t_start
        if total > 0 and loop_count > 0:
            logger.info(f"Average loop rate: {loop_count / total:.1f} Hz over {loop_count} frames")
        logger.info("Done.")


if __name__ == "__main__":
    main()
