from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("arx_r5_robot")
@dataclass
class ARXR5Config(RobotConfig):
    """Configuration for ARX R5 dual-arm robot (arms only, no LIFT chassis)."""

    # CAN interface configuration
    left_can: str = "can1"
    right_can: str = "can3"

    # Arm type (0=R5 L5, 1=R5 L5 Pro)
    arm_type: int = 0

    # Number of joints per arm (R5 has 7 joints)
    num_joints: int = 7

    # Control parameters
    dt: float = 0.05  # Control period (20Hz)

    # Initial joint positions (radians) - R5 has 7 joints
    left_init_joints: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    right_init_joints: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Gripper configuration
    use_gripper: bool = True
    gripper_close: float = -0.2
    gripper_open: float = 0.2

    # Debug mode (if True, robot won't execute movements)
    debug: bool = True

    # Camera configuration
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Head camera extrinsic calibration
    enable_calibration: bool = False
    calibration_params_path: str = ""  # auto-detect if empty
