"""Developer-only API smoke test. NOT a hardware bring-up example.

Runs a 1 kHz MIT control loop against a four-motor arm over MockCanBus
(`--mock`) or SocketCAN (`--interface vcan0`). This exists to exercise the
Python API surface end-to-end; mock execution proves shape and wiring, NOT
that real hardware will respond correctly.

For real hardware bring-up, see the `00_`–`08_` examples in `examples/`.
They add risk one step at a time (interface check -> single motor -> arm ->
calibration) and are the supported path for first-time hardware testing.

Usage:
    python examples/dev_api_smoke.py --mock
    python examples/dev_api_smoke.py --interface vcan0
    python examples/dev_api_smoke.py --interface vcanfd0 --fd
"""

import argparse
import time

import numpy as np

import dm_control
from dm_control.damiao import DamiaoCodec, MotorType


def build_robot(use_mock: bool, interface: str, fd: bool = False) -> dm_control.Robot:
    if use_mock:
        transport = (
            dm_control.MockCanBus.new_fd("vcan_mock")
            if fd
            else dm_control.MockCanBus("vcan_mock")
        )
    else:
        transport = dm_control.SocketCanBus(interface, fd=fd)
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
    parser.add_argument("--fd", action="store_true", help="open the bus in CAN-FD mode")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--period", type=float, default=1e-3)
    parser.add_argument("--deadline-us", type=int, default=500)
    args = parser.parse_args()

    robot = build_robot(args.mock, args.interface, args.fd)

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
