# SocketCAN setup for `dm_control` hardware examples

`dm_control` v1 talks to Damiao motors over **classical CAN only** on
Linux SocketCAN. CAN-FD is intentionally not supported in v1; the bus
rejects `fd=true` (see [`docs/can-fd.md`](./can-fd.md)).

This document is the minimum operator checklist for the
`examples/00_*` -> `examples/08_*` ladder.

## Real CAN interface (e.g. USB or PCIe CAN adapter)

```bash
# Adapter dependent; consult the vendor docs. For a SocketCAN-native
# device the kernel exposes it as e.g. can0:
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Verify:
ip link show can0
candump can0   # leave running in another terminal during bring-up
```

A common Damiao default is **1 Mbit/s** classical CAN; double-check
the bitrate against your motor datasheet before powering motors.

## Virtual CAN (`vcan0`) for development without hardware

`vcan` is a kernel module that exposes a software-only CAN interface.
It is sufficient to syntax-check the bring-up examples end-to-end
without any USB CAN adapter and without any motor. It is **not** a
substitute for real hardware testing -- a `vcan0` run cannot tell you
whether the motor sends back the expected state replies.

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up

# Verify:
ip link show vcan0
candump vcan0
```

## What the examples assume

Before running anything in `examples/01_*` or beyond:

1. `00_can_interface_check.py` exits cleanly on your chosen interface.
2. `ip link show <iface>` reports `UP`.
3. `candump <iface>` is running in a second terminal and quiet.
4. The motor (or arm) is powered, mounted, and clear of obstacles.
5. The CAN IDs (`--send-id` / `--recv-id`) match the motor flash configuration.

## CI vs. bring-up

* **CI runs** the Rust unit / integration tests and `tests/python/`,
  all of which use `MockCanBus`. CI does **not** depend on `vcan0` or
  any host CAN interface, and CI does **not** validate hardware.
* **`vcan0`** is suitable for additional local smoke-testing of the
  bring-up examples without motors. It exercises the SocketCAN
  syscalls and the codec, but no motor replies arrive.
* **Real `can0`** is the only configuration that actually validates
  hardware. Treat the bring-up ladder (`00` -> `08`) as the supported
  path; mock examples do not substitute for it.

## Skipping host-CAN tests

The Rust workspace ignores SocketCAN-host-dependent tests by default
(they require a live interface and `CAP_NET_ADMIN`). If you want to
opt in locally, run them with the `--ignored` flag:

```bash
cargo test --workspace --all-targets -- --ignored
```

Python tests should never require a host CAN interface; if they do,
that is a bug -- file an issue.
