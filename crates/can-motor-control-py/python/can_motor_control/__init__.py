"""can_motor_control — Rust-backed control library for Damiao-family CAN motors."""

from can_motor_control._native import (
    CanFrame,
    MockCanBus,
    SocketCanBus,
    MotorSpec,
    Motor,
    Arm,
    Gripper,
    MotorGroup,
    Robot,
    RobotBuilder,
    DmError,
    TransportError,
    CodecError,
    ConfigError,
    LifecycleError,
)
from can_motor_control import damiao

__all__ = [
    "CanFrame",
    "MockCanBus",
    "SocketCanBus",
    "MotorSpec",
    "Motor",
    "Arm",
    "Gripper",
    "MotorGroup",
    "Robot",
    "RobotBuilder",
    "DmError",
    "TransportError",
    "CodecError",
    "ConfigError",
    "LifecycleError",
    "damiao",
]

__version__ = "0.1.0"
