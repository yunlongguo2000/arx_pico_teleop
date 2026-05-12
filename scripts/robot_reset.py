#!/usr/bin/env python
"""
robot-reset: Move ARX R5 robot to home (initial) position via RPC.

Reads left_init_joints / right_init_joints from config and sends command.

Usage:
    robot-reset
    robot-reset <robot-ip>
    ARX_RPC_HOST=<robot-ip> robot-reset
"""

import os
import sys
import yaml
import logging
from pathlib import Path

# Add package root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from robots.arx import ARXLift2Config, ARXLift2

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)

def main():
    # Load config from scripts/config/cfg_arx.yaml
    config_name = os.environ.get("REPLAY_CONFIG", "cfg_arx.yaml")
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path / "config" / config_name
    if not cfg_path.exists():
        cfg_path = parent_path / "config" / "cfg.yaml"
        if not cfg_path.exists():
            logger.error(f"Config not found: {parent_path / 'config' / config_name}")
            sys.exit(1)

    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    # Get robot config from replay or record section
    if "replay" in cfg and "robot" in cfg["replay"]:
        robot_cfg = cfg["replay"]["robot"]
    elif "record" in cfg and "robot" in cfg["record"]:
        robot_cfg = cfg["record"]["robot"]
    else:
        robot_cfg = cfg.get("robot", {})

    # Create robot config
    arx_config = ARXLift2Config(
        left_can=robot_cfg.get("left_can", "can1"),
        right_can=robot_cfg.get("right_can", "can3"),
        arm_type=robot_cfg.get("arm_type", 0),
        use_gripper=robot_cfg.get("use_gripper", True),
        debug=False
    )

    # Load initial joint positions from config
    if "record" in cfg and "robot" in cfg["record"]:
        arx_config.left_init_joints = cfg["record"]["robot"].get(
            "left_init_joints", arx_config.left_init_joints
        )
        arx_config.right_init_joints = cfg["record"]["robot"].get(
            "right_init_joints", arx_config.right_init_joints
        )

    logger.info(f"Connecting to robot at {os.environ.get('ARX_RPC_HOST', 'localhost')}...")
    robot = ARXLift2(arx_config)
    robot.connect()

    logger.info(f"Left init: {[round(x, 4) for x in arx_config.left_init_joints]}")
    logger.info(f"Right init: {[round(x, 4) for x in arx_config.right_init_joints]}")
    logger.info("Moving arms to home (initial) position...")

    robot.go_home()

    logger.info("Done. Arms are at home position. RPC connection remains open.")
    # Don't disconnect - keep connection open for next commands

if __name__ == "__main__":
    main()
