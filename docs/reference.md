# Python API

The reference below is generated from the installed `dm_control` package by
[mkdocstrings](https://mkdocstrings.github.io/). The prose comes from the
docstrings authored in the Rust PyO3 source, so it matches exactly what
`help()` and Jupyter `?` show.

!!! note
    This page requires the native extension to be importable. Build it first
    with `maturin develop` (the `make docs` / `make docs-build` targets do this
    for you).

## Robot and builder

::: dm_control.Robot

::: dm_control.RobotBuilder

## Groups

::: dm_control.Arm

::: dm_control.Gripper

::: dm_control.MotorGroup

::: dm_control.Motor

## Specs and frames

::: dm_control.MotorSpec

::: dm_control.CanFrame

## Transports

::: dm_control.MockCanBus

::: dm_control.SocketCanBus

## Damiao codec

::: dm_control.damiao.DamiaoCodec

::: dm_control.damiao.MotorType

## Errors

::: dm_control.DmError

::: dm_control.TransportError

::: dm_control.CodecError

::: dm_control.ConfigError

::: dm_control.LifecycleError
