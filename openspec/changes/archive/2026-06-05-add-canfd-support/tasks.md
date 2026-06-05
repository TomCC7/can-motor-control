## 1. Remove the v1 rejection seams

- [x] 1.1 Delete `FdNotImplementedInV1` from `crates/can-motor-control/src/transport/mod.rs` (`TransportError`) and from `crates/can-motor-control/src/error.rs` (`Error`), plus their `#[error("...")]` strings.
- [x] 1.2 Remove the `if fd_enabled { return Err(FdNotImplementedInV1) }` guard in `SocketCanBus::open` (`transport/socketcan.rs`) and the send-path "defense in depth" `if frame.is_fd() { return Err(FdNotImplementedInV1) }`.
- [x] 1.3 Remove the config-parse rejection loop (`if bus.fd { return Err(FdNotImplementedInV1 ...) }`) in `crates/can-motor-control/src/config.rs`.
- [x] 1.4 Confirm `validate_send`'s `FdFrameOnNonFdBus` guard is retained (FD frame on a `supports_fd == false` bus still errors).

## 2. SocketCAN FD transport

- [x] 2.1 In `SocketCanBus::open`, when FD is enabled, set the `CAN_RAW_FD_FRAMES` socket option (setsockopt on `SOL_CAN_RAW`) before/after bind per kernel requirements.
- [x] 2.2 Set `caps` to `BusCapabilities::fd()` when opened FD, `BusCapabilities::classical()` otherwise (replace the hardcoded `classical()`).
- [x] 2.3 In `send`, branch on `frame.is_fd()`: serialize FD frames as `KernelCanFdFrame` (carry `len`, `BRS`/`ESI` flags, payload) and classical frames as `KernelCanFrame` (unchanged).
- [x] 2.4 Verify the receive path needs no change beyond FD being enabled (it already allocates `KERNEL_FD_LEN` and matches `decode_kernel_fd`); add a `size_of::<KernelCanFdFrame>() == 72` assertion test so layout drift fails loudly.

## 3. Codec capability gating

- [x] 3.1 In `DamiaoCodec::bind_to_bus`, keep storing `caps`; document that classical binding must reproduce v1 bytes exactly.
- [x] 3.2 In the decode path (`damiao-codec/src/codec.rs`), stop discarding FD frames unconditionally: accept FD-format state frames when bound to an FD bus, keep the `len != 8`/`is_fd` discard for classical binding so classical behavior is unchanged.
- [x] 3.3 Confirm the encode methods still emit classical 8-byte frames (conservative FD: valid on an FD bus); no payload-layout change in this task.

## 4. Mock transport for hardware-free tests

- [x] 4.1 Add an FD-capable constructor to `MockCanBus` (e.g. `new_fd`) advertising `BusCapabilities::fd()`; default `new` stays classical.

## 5. Config, Python surface, examples

- [x] 5.1 `config.rs`: `fd = true` now opens an FD bus; pass the flag through to `SocketCanBus::open`. `fd = false`/omitted stays classical.
- [x] 5.2 `crates/can-motor-control-py/src/transport.rs`: update `SocketCanBus(..., fd=True)` docstring (no longer raises); `crates/can-motor-control-py/src/lib.rs`: update `ConfigError` doc wording that says "unsupported option such as CAN-FD in v1".
- [x] 5.3 Add an FD example config under `configs/` (distinct interface name, `fd = true`) and confirm existing `fd = false` configs are untouched.

## 6. Tests

- [x] 6.1 Flip `crates/can-motor-control/tests/config_loader.rs::fd_true_rejected_at_parse_time` to assert the robot builds and the bus is FD-capable (rename the test accordingly).
- [x] 6.2 Add an FD round-trip test over `MockCanBus`: FD-bound codec encode → send → recv → decode preserves payload and FD flag.
- [x] 6.3 Add a regression test: classical-bound codec output is byte-for-byte identical to v1 for a representative command.
- [x] 6.4 Add a test that sending an FD frame on a classical bus returns `FdFrameOnNonFdBus`.
- [x] 6.5 Confirm the `group.rs` source-invariant grep test still passes (upper layers never reference `is_fd`/`FD_FORMAT`).

## 7. Docs

- [x] 7.1 Rewrite `docs/can-fd.md` from "not implemented / reserved" into a usage guide (how to enable FD in TOML and Python, capability handshake, the conservative-Damiao note).
- [x] 7.2 Update README status/platform notes so they no longer state CAN-FD is reserved/unsupported.

## 8. Verify

- [x] 8.1 `cargo build --workspace` and `cargo test --workspace` pass.
- [x] 8.2 `cargo clippy --workspace` clean; `make docs-build` still renders (docstring/wording changes intact).

## 9. Examples runnable in CAN-FD mode

- [x] 9.1 Expose `MockCanBus.new_fd(name)` in PyO3 (`crates/can-motor-control-py/src/transport.rs`) and the `.pyi` stub; Rust `MockCanBus::new_fd` already exists.
- [x] 9.2 Fix the FD-rejection regression in `tests/python/test_smoke.py`: replace `test_socketcanbus_fd_true_rejected` with `test_canframe_fd_constructor` + `test_fd_mock_bus_runs_full_stack` (robot on an FD mock bus runs a MIT loop).
- [x] 9.3 `examples/_bringup_common.py`: replace the static "no CAN-FD" banner with an FD-aware `wire_format_lines(fd)` (False/True/None) and add `add_fd_arg`.
- [x] 9.4 Add `--fd` to the direct-interface Python examples (00–04, 07, dev_api_smoke) plumbed to `SocketCanBus(iface, fd=args.fd)`; dev_api_smoke uses `MockCanBus.new_fd` under `--mock --fd`.
- [x] 9.5 Mixed examples 05/08: `--fd` on the direct single-motor path; config path is FD-governed by the TOML (`fd=None` banner). 06 is config-only; help points at `configs/openarm_canfd.toml`.
- [x] 9.6 Add `--fd` to all 6 Rust examples plumbed to `SocketCanBus::open(iface, args.fd)`, with an FD-aware `print_wire_format` banner and help text.
- [x] 9.7 Verify: `cargo build -p can-motor-control --examples` + clippy clean; `pytest tests/python` green (14 passed); example `--help`/`--fd` and `dev_api_smoke --mock --fd` smoke-checked.
