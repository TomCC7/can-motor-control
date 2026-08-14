## 1. Platform Foundation and Dependency Validation

- [x] 1.1 Pin a compatible `nusb` 0.2 release as a macOS-target dependency and verify its endpoint submission, completion, buffer sizing, cancellation, and interface-release APIs with a minimal compile test
- [x] 1.2 Gate the SocketCAN module, exports, configuration path, tests, and examples to Linux and remove unsupported `SocketCanBus` references from macOS targets
- [x] 1.3 Add the macOS gs_usb module/export path and make target-native transport availability automatic without a user-facing hardware feature
- [x] 1.4 Add platform-neutral `TransportError::InvalidConfiguration(String)` and verify existing Rust and Python error mapping remains exhaustive
- [x] 1.5 Correct stale `raw_fd() == None` polling documentation and the connect-time warning so they describe the robot's existing per-tick memory drain

## 2. Protocol and Timing Test Fixtures

- [x] 2.1 Add failing unit tests for gs_usb host-format, device-info, capability, mode, nominal-bit-timing, and 20-byte classical host-frame byte layouts
- [x] 2.2 Add failing standard and extended classical frame round-trip tests covering identifier flags, DLC, payload lengths, reserved fields, and channel 0
- [x] 2.3 Add failing malformed-frame tests for short buffers, invalid DLC, CAN-FD flags, unsupported channel values, and incompatible CAN flags
- [x] 2.4 Add capability-driven timing tests for common clocks and the observed 170 MHz clock, including exact 1 Mbit/s solutions and impossible bitrate/constraint combinations
- [x] 2.5 Implement the narrow local classical gs_usb protocol encoder/decoder and capability structures until the protocol tests pass
- [x] 2.6 Implement deterministic nominal bit-timing search and validation from reported clock/constraints without clock-specific preset tables

## 3. Device Selection and Initialization

- [x] 3.1 Define the Rust `GsUsbBus` configuration/open surface for VID, PID, mutually exclusive optional serial/index, bitrate, and default five-second initialization deadline
- [x] 3.2 Add fake-enumerator tests for serial selection, index selection, default index zero, ambiguous serials, out-of-range indexes, and contradictory selectors
- [x] 3.3 Implement descriptor-driven interface/alternate-setting and bulk-IN/bulk-OUT discovery with tests for varying addresses, missing endpoints, non-bulk endpoints, and ambiguous layouts
- [x] 3.4 Add initialization-state tests that assert host-format negotiation, capability query, channel-0 validation, reset-before-timing, timing programming, start, clean-session flush, and RX priming order
- [x] 3.5 Implement worker-owned discovery, open/configuration, interface claim, protocol initialization, and ready-or-error constructor handoff
- [x] 3.6 Add retry-classification tests proving complete-transaction retry for transient re-enumeration failures and immediate failure for permission, selection, descriptor, protocol, capability, and timing errors
- [x] 3.7 Implement shared-deadline initialization retry that drops all partial USB ownership before restarting from enumeration
- [x] 3.8 Add and implement bounded stale-input discard that requires a quiet session boundary before reporting ready

## 4. Worker, TX, and RX Data Paths

- [x] 4.1 Add a testable worker harness with fake IN/OUT endpoints, bounded TX commands, bounded 8,192-frame RX FIFO, terminal error state, statistics, and shutdown signaling
- [x] 4.2 Add failing tests for `CanBus` capabilities, `raw_fd() == None`, standard/extended send validation, CAN-FD rejection, and immediate empty/non-empty inbound drains
- [x] 4.3 Implement `GsUsbBus::send()` as validation plus bounded nonblocking acceptance, returning `SendBufferFull` without evicting outbound commands
- [x] 4.4 Implement worker-owned bulk-OUT submission, OUT completion collection, finite echo-credit allocation, and frame encoding
- [x] 4.5 Add tests for credit exhaustion, OUT transfer failure, normal echo release, unknown/duplicate/out-of-range echoes, and proof that echoes never enter the CAN RX queue
- [x] 4.6 Implement continuously queued bulk-IN transfers using endpoint-compatible request sizes and completed-buffer reuse
- [x] 4.7 Implement RX parsing, classical `CanFrame` conversion, FIFO arrival ordering, and memory-only `drain_inbound_nonblocking()`
- [x] 4.8 Add and pass overflow tests proving the oldest RX frame is evicted, the newest is retained, and one drop is counted per eviction
- [x] 4.9 Add sustained fake-traffic tests proving IN transfers remain primed while TX commands, OUT completions, and TX echoes are processed concurrently

## 5. Failure and Shutdown Lifecycle

- [x] 5.1 Add tests distinguishing expected shutdown cancellation, one successful clear-halt recovery, failed halt recovery, disconnect, malformed transfer, and terminal protocol failure
- [x] 5.2 Implement fail-closed worker state so a terminal failure closes TX acceptance and is returned by the next send or inbound drain without automatic reconnection
- [x] 5.3 Add deterministic shutdown tests for quiet RX pending, TX commands pending, OUT transfers pending, already-faulted workers, and partially initialized workers
- [x] 5.4 Implement ownership-driven cleanup that closes TX, requests stop, cancels and collects all endpoint completions, resets channel 0, releases endpoints/interface, and joins the worker
- [x] 5.5 Verify `Drop` reuses the tested shutdown path best-effort and cannot leave a detached worker or retained interface clone
- [x] 5.6 Add a fake-device immediate-reopen regression test proving one dropped bus cannot retain a claim or leak stale session frames into its successor

## 6. Rust Platform Integration

- [x] 6.1 Update crate-root and transport exports, descriptions, docs, config handling, examples, and tests so Linux builds use only `SocketCanBus` and macOS builds use only `GsUsbBus`
- [x] 6.2 Add Rust API documentation for device selectors, send acceptance, initialization deadline, platform availability, failure behavior, statistics, and ownership-driven cleanup
- [x] 6.3 Run Linux workspace tests and Clippy across all targets to prove SocketCAN and MockCanBus behavior remains unchanged
- [x] 6.4 Run macOS workspace build, tests, and Clippy across all compatible targets to prove no Linux CAN constants or exports leak into macOS

## 7. Python API and Ownership

- [x] 7.1 Add failing PyO3 tests for the keyword-only `GsUsbBus` constructor, mutually exclusive selectors, transport errors, and GIL release during readiness wait
- [x] 7.2 Implement the macOS `PyGsUsbBus` wrapper and native module export while conditionally compiling `PySocketCanBus` only on Linux
- [x] 7.3 Extend `RobotBuilder.add_bus()` to consume `GsUsbBus` exactly once and update platform-specific accepted-type errors and consumed-once tests
- [x] 7.4 Implement shared read-only RX-received, RX-dropped, TX-accepted, and TX-completed counters that remain accessible through the Python wrapper after transport consumption
- [x] 7.5 Update pure-Python exports, `__all__`, and `.pyi` declarations so each platform exposes only its native hardware transport without compatibility fallbacks
- [x] 7.6 Add wheel-level Python smoke tests for conditional transport presence, constructor validation without hardware, builder consumption, statistics access, and import behavior

## 8. macOS CI, Packaging, and Documentation

- [x] 8.1 Add a pinned GitHub-hosted macOS arm64 CI job that asserts `uname -m == arm64` and runs Rust build, tests, and Clippy for all macOS-compatible targets
- [x] 8.2 Build a macOS arm64 wheel in CI, assert its platform tag, install it in a clean environment, import it, and verify `GsUsbBus` is present while `SocketCanBus` is absent
- [x] 8.3 Extend Linux clean-install tests to verify `SocketCanBus` is present while `GsUsbBus` is absent
- [x] 8.4 Extend release-candidate workflow artifacts and clean-install validation to include independent Linux and macOS arm64 wheels plus the sdist without publishing
- [x] 8.5 Extend tag-driven release artifact assembly so the protected PyPI job receives and publishes the validated macOS arm64 wheel with existing distributions
- [x] 8.6 Update README/package descriptions and platform installation, API, selector, diagnostics, and troubleshooting documentation to describe target-native transports and remove Linux-only claims
- [x] 8.7 Document the downstream DimOS handoff: remove its Linux dependency marker, select native transport explicitly by platform, replace only OpenYAM's Python USB routing, and leave Galaxea unchanged

## 9. Hardware Acceptance and Final Verification

- [ ] 9.1 On macOS arm64, verify descriptor-driven endpoint discovery, 170 MHz capability-derived 1 Mbit/s timing, reset-before-configure, and fresh feedback after a prior unclean session
- [ ] 9.2 Run 7,500 OpenYAM cycles at 250 Hz across all seven motors and record average/p95/p99 tick time, CPU utilization, feedback completeness, TX statistics, and RX drops
- [ ] 9.3 Verify the sustained run has fresh feedback from every motor, zero RX drops, and no cycle beyond the existing 12.5 ms deadline
- [ ] 9.4 Repeatedly disable, destroy, reconstruct, and reconnect the same adapter within the five-second initialization deadline without manual unplugging or stale feedback
- [ ] 9.5 Record target-adapter TX echo timing and error observations and confirm the implemented echo-credit assumptions match hardware
- [ ] 9.6 Run the full Linux and macOS automated verification matrix, validate OpenSpec artifacts against the implementation, and update public release notes before declaring the change complete
