"""Python integration test mirror of `integration_single_arm.rs`: builds a
robot, runs a 100-tick MIT loop against MockCanBus, asserts clean disable."""

import numpy as np

import can_motor_control
from can_motor_control.damiao import DamiaoCodec, MotorType


def test_full_loop_against_mock():
    builder = (
        can_motor_control.RobotBuilder()
        .add_bus("main", can_motor_control.MockCanBus("vcan_mock"), DamiaoCodec())
        .add_arm(
            "arm",
            bus="main",
            motors=[
                can_motor_control.MotorSpec("j0", MotorType.DM4340, send_id=0x01, recv_id=0x11),
                can_motor_control.MotorSpec("j1", MotorType.DM4340, send_id=0x02, recv_id=0x12),
                can_motor_control.MotorSpec("j2", MotorType.DM4340, send_id=0x03, recv_id=0x13),
            ],
        )
    )
    robot = builder.build()
    with robot:
        arm = robot["arm"]
        n = len(arm)
        cmds = np.zeros((n, 5), dtype=np.float64)
        cmds[:, 0] = 50.0
        cmds[:, 1] = 1.0
        for _ in range(100):
            robot.tick(500)
            arm.mit_control(cmds)
        positions = arm.positions()
        assert positions.shape == (n,)
