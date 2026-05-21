"""End-to-end Python example: 10-second 1 kHz MIT control loop on a single
Damiao arm. Uses --mock for CI; --interface vcan0 for real hardware.

Usage:
    python examples/single_arm.py --mock
    python examples/single_arm.py --interface vcan0
"""

import argparse
import time

import numpy as np

import dm_control
from dm_control.damiao import DamiaoCodec, MotorType


def build_robot(use_mock: bool, interface: str) -> dm_control.Robot:
    transport = (
        dm_control.MockCanBus("vcan_mock")
        if use_mock
        else dm_control.SocketCanBus(interface, fd=False)
    )
    return (
        dm_control.RobotBuilder()
        .add_bus("main", transport, DamiaoCodec())
        .add_arm(
            "arm",
            bus="main",
            motors=[
                dm_control.MotorSpec("j0", MotorType.DM4340, send_id=0x01, recv_id=0x11),
                dm_control.MotorSpec("j1", MotorType.DM4340, send_id=0x02, recv_id=0x12),
                dm_control.MotorSpec("j2", MotorType.DM4340, send_id=0x03, recv_id=0x13),
                dm_control.MotorSpec("j3", MotorType.DM4340, send_id=0x04, recv_id=0x14),
            ],
        )
        .build()
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock", action="store_true", help="use MockCanBus (no hardware)")
    parser.add_argument("--interface", default="vcan0", help="SocketCAN interface")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--period", type=float, default=1e-3)
    parser.add_argument("--deadline-us", type=int, default=500)
    args = parser.parse_args()

    robot = build_robot(args.mock, args.interface)

    with robot:
        arm = robot["arm"]
        n = len(arm)
        cmds = np.zeros((n, 5), dtype=np.float64)
        cmds[:, 0] = 50.0  # kp
        cmds[:, 1] = 1.0  # kd

        deadline = time.monotonic() + args.seconds
        ticks = 0
        while time.monotonic() < deadline:
            t0 = time.monotonic()
            robot.tick(args.deadline_us)
            arm.mit_control(cmds)
            ticks += 1
            rem = args.period - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)
        print(f"completed {ticks} ticks")


if __name__ == "__main__":
    main()
