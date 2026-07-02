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

try:
    from can_motor_control._native import MockFeedbackCodec
except ImportError:
    MockFeedbackCodec = None  # type: ignore[assignment]

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

if MockFeedbackCodec is not None:
    __all__.append("MockFeedbackCodec")

__version__ = "0.1.0"
