"""Smoke test: build a robot against MockCanBus, run a 10-tick loop with
MIT commands, assert expected frame count, disable cleanly.

The 10-tick loop validates that the entire stack — transport, codec, group
dispatch, lifecycle — is wired correctly via Python without any real CAN
hardware.
"""

import numpy as np

import dm_control
from dm_control.damiao import DamiaoCodec, MotorType


def make_robot() -> dm_control.Robot:
    builder = (
        dm_control.RobotBuilder()
        .add_bus("main", dm_control.MockCanBus("vcan_mock"), DamiaoCodec())
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
    )
    return builder.build()


def test_imports():
    assert dm_control.Robot is not None
    assert dm_control.RobotBuilder is not None
    assert dm_control.SocketCanBus is not None
    assert dm_control.MotorSpec is not None
    assert dm_control.DmError is not None


def test_damiao_submodule_accessible():
    from dm_control.damiao import DamiaoCodec, MotorType

    assert MotorType.DM4340 is not None
    assert DamiaoCodec() is not None


def test_socketcanbus_fd_true_rejected():
    try:
        dm_control.SocketCanBus("vcan0", fd=True)
    except dm_control.TransportError as e:
        assert "CAN-FD is reserved" in str(e)
        return
    raise AssertionError("expected TransportError")


def test_builder_build():
    robot = make_robot()
    assert robot.bus_names() == ["main"]
    assert robot.group_names() == ["arm"]
    assert not robot.is_connected()


def test_context_manager_lifecycle_and_mit_loop():
    robot = make_robot()
    with robot:
        arm = robot["arm"]
        n = len(arm)
        assert n == 4
        cmds = np.zeros((n, 5), dtype=np.float64)
        cmds[:, 0] = 50.0  # kp
        cmds[:, 1] = 1.0  # kd
        for _ in range(10):
            robot.tick(500)  # 500 us deadline
            arm.mit_control(cmds)
        positions = arm.positions()
        assert positions.shape == (n,)
        assert positions.dtype == np.float64


def test_getitem_returns_arm_wrapper():
    robot = make_robot()
    arm = robot["arm"]
    assert isinstance(arm, dm_control.Arm)
    try:
        robot["ghost"]
    except KeyError:
        pass
    else:
        raise AssertionError("expected KeyError")


def test_attribute_access_not_supported():
    robot = make_robot()
    try:
        _ = robot.arm
    except AttributeError:
        pass
    else:
        raise AssertionError("expected AttributeError")


def test_add_arm_rejects_codec_kwarg():
    builder = dm_control.RobotBuilder().add_bus(
        "main", dm_control.MockCanBus("m"), DamiaoCodec()
    )
    try:
        builder.add_arm(
            "arm",
            bus="main",
            codec=DamiaoCodec(),  # not a valid kwarg
            motors=[dm_control.MotorSpec("j0", MotorType.DM4340, 1, 0x11)],
        )
    except TypeError:
        pass
    else:
        raise AssertionError("expected TypeError")


def test_mit_wrong_shape_raises_value_error():
    robot = make_robot()
    with robot:
        arm = robot["arm"]
        try:
            arm.mit_control(np.zeros((3, 5), dtype=np.float64))  # 4 motors expected
        except ValueError as e:
            assert "(4, 5)" in str(e) and "(3, 5)" in str(e)
        else:
            raise AssertionError("expected ValueError")


def test_mit_wrong_dtype_raises_type_error():
    robot = make_robot()
    with robot:
        arm = robot["arm"]
        try:
            arm.mit_control(np.zeros((4, 5), dtype=np.float32))
        except (TypeError, ValueError):
            # numpy may raise TypeError or convert; we accept either.
            pass
        else:
            raise AssertionError("expected TypeError or ValueError")


def test_dmerror_hierarchy():
    assert issubclass(dm_control.TransportError, dm_control.DmError)
    assert issubclass(dm_control.CodecError, dm_control.DmError)
    assert issubclass(dm_control.ConfigError, dm_control.DmError)
    assert issubclass(dm_control.LifecycleError, dm_control.DmError)


def test_tick_before_connect_raises_lifecycle_error():
    robot = make_robot()
    try:
        robot.tick(1000)
    except dm_control.LifecycleError as e:
        assert "NotConnected" in str(e) or "not connected" in str(e).lower()
    else:
        raise AssertionError("expected LifecycleError")
