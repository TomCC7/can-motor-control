## Context

The core crate has an object-safe synchronous `CanBus` seam. `SocketCanBus::send()` accepts a frame into the kernel or reports bounded backpressure, `drain_inbound_nonblocking()` returns all currently available frames in arrival order, and `Robot::tick()` drains buses whose `raw_fd()` is `None` on every tick. The PyO3 layer wraps concrete transports in a consumed-once handle used by `RobotBuilder.add_bus()`.

macOS has no SocketCAN. The target candleLight/gs_usb adapters can instead be driven through IOKit. Hardware testing in DimOS established several device behaviors: endpoint addresses vary, TX echoes share the IN endpoint, reset-before-configuration prevents wedged RX, stop/reset may cause temporary re-enumeration, some devices report a 170 MHz CAN clock, stale frames survive session boundaries, and synchronous reads inside the control loop cannot drain high-rate feedback reliably.

The implementation crosses the core Rust transport, PyO3 bindings, target-specific exports, package stubs, CI, release workflows, and downstream installation. It must make macOS behavior as close as possible to existing SocketCAN behavior without retaining obsolete cross-platform placeholders or introducing a Python transport bridge.

## Goals / Non-Goals

**Goals:**

- Provide a first-class macOS `GsUsbBus` for one-channel classical CAN with standard and extended identifiers.
- Preserve the observable `CanBus` contract: ready-or-error construction, nonblocking send acceptance, memory-only inbound drain, fail-closed errors, and ownership-driven cleanup.
- Keep USB waits and completion processing outside the robot control thread.
- Support explicit VID/PID selection by either serial number or enumeration index and configurable nominal bitrate, including 1 Mbit/s on reported 170 MHz clocks.
- Build and publish native macOS arm64 Python wheels while retaining Linux SocketCAN wheels.
- Keep protocol parsing, timing selection, queue behavior, and lifecycle logic testable without USB hardware.

**Non-Goals:**

- CAN-FD, hardware timestamps, listen-only mode, multi-channel adapters, SLCAN, or automatic runtime reconnection.
- Userspace gs_usb on Linux, where SocketCAN is the native and preferred path.
- A `rusb`, PyUSB, callback, or system-libUSB fallback.
- A public async API or an application Tokio runtime requirement.
- Changes to motor codecs, robot topology, OpenYAM control behavior, or the separate Galaxea transport integration.
- Resolving or otherwise modifying the downstream DimOS merge as part of this upstream change.

## Decisions

### Target-native transports are mandatory and mutually exclusive

Linux builds compile and export `SocketCanBus`; macOS builds compile and export `GsUsbBus`; both retain `MockCanBus`. Unsupported transports are absent from Rust exports, the native Python module, the pure-Python package, and type stubs. `nusb` is a macOS-target-specific dependency rather than a user-facing feature, so a normal Rust or Python macOS installation cannot silently omit its hardware transport.

This removes the alternatives of failing placeholder constructors and an opt-in macOS feature. Both would create platform APIs that install successfully but cannot perform their advertised job.

### Use nusb and keep the narrow classical gs_usb protocol local

`nusb` supplies enumeration, descriptor access, IOKit integration, control transfers, exclusive endpoints, queued bulk transfers, cancellation, and buffer reuse. Local code supplies only the required gs_usb control layouts, capability decoding, nominal timing calculation, 20-byte classical host-frame encoding/decoding, and echo bookkeeping.

The published Rust `gs_usb` crate was evaluated and rejected because it uses `rusb`/libusb, hardcodes endpoint addresses, performs blocking reads and one-second writes, hides queued-transfer ownership, and supports hardcoded 40/80 MHz timing rather than the observed 170 MHz device. Pulling only its frame type would still introduce the unwanted backend.

### One worker owns the complete USB lifecycle

`GsUsbBus::open` spawns one worker and waits for a one-shot ready-or-error result. The worker owns discovery, device and interface handles, both bulk endpoints, initialization, IN and OUT completion queues, echo credits, runtime failure, cancellation, reset, and release. Once ready, the public bus retains bounded command/receive channels, shared statistics and terminal error state, and a join handle.

```text
control thread                 USB worker                      adapter
--------------                 ----------                      -------
send(frame)
  -> validate
  -> bounded try_send -------> assign echo credit
                                submit bulk-OUT -------------->

drain_inbound_nonblocking()    queued bulk-IN <--------------- RX / echo
  <- bounded RX queue <------- parse RX; consume echo

Drop ------------------------> reject TX; cancel/collect
                               reset channel; release; exit
  <--------------------------- join
```

The caller-facing API remains synchronous and no Tokio runtime is required. Blocking setup operations run in the dedicated worker. PyO3 releases the GIL while construction waits for readiness.

### Send acceptance mirrors SocketCAN

`send()` validates the frame and uses a bounded nonblocking command enqueue. `Ok(())` means the transport accepted responsibility for attempting the frame; it does not mean that USB completed, the CAN controller transmitted it, or another node acknowledged it. A full command path returns `SendBufferFull`; outbound frames are never discarded for freshness.

The worker uses a finite echo-credit table. It releases a credit when the corresponding non-RX echo arrives on bulk-IN. USB OUT completions and device echoes remain internal and never reach motor codecs. Unknown, duplicate, or out-of-range echo IDs are transport faults or recorded protocol errors, not CAN input.

### RX is continuously queued and bounded

The worker keeps multiple bulk-IN transfers submitted, sizes requests according to the endpoint's maximum-packet requirements, reuses completed buffers, and parses classical frames as they complete. It feeds a fixed 8,192-frame FIFO, matching the hardware-tested reference behavior. If full, it evicts the oldest frame, inserts the newest, and increments the drop counter. `drain_inbound_nonblocking()` only drains memory and returns immediately.

Hardware and unit tests, rather than the public API, determine the initial number of in-flight transfers. The value can be tuned without changing transport behavior.

### Construction establishes a configured clean session

The selector is VID/PID plus exactly one optional identity: serial number or enumeration index. Both is invalid; neither means index zero. Serial selection must resolve unambiguously. Index selection is explicitly enumeration-order-dependent.

The worker retries the entire initialization transaction only for errors classified as transient re-enumeration failures, under one default five-second deadline:

1. List and select the device.
2. Open it, set a configuration on macOS if required, and claim interface/alternate setting.
3. Discover one bulk-IN and one bulk-OUT endpoint from descriptors.
4. Negotiate host format and query device/channel information and nominal timing constraints.
5. Validate channel 0 and the requested classical-CAN mode.
6. Reset channel 0 before configuration.
7. Calculate and program nominal timing from the reported clock and constraints.
8. Start channel 0 in normal classical mode.
9. Drain stale input until a bounded quiet interval establishes a session boundary.
10. Prime queued IN transfers and report ready.

Permission failures, ambiguous selection, invalid descriptors, unsupported protocol/capabilities, impossible timing, and other deterministic configuration errors fail immediately. If the stale-frame flush cannot reach a quiet boundary before its bound, construction fails instead of exposing possibly stale feedback.

### Runtime failures fail closed

Cancellation caused by shutdown exits normally. An endpoint halt receives one narrowly scoped clear-halt/resubmit attempt. A failed recovery, disconnect, permission loss, malformed protocol state, or other terminal USB error stores a terminal transport error, closes the TX path, and stops the worker. The next `send()` or `drain_inbound_nonblocking()` reports the error. Recovery requires destroying and reconstructing the bus; there is no hidden runtime reconnection state machine.

`TransportError` reuses existing variants for not-found, permission, backpressure, frame/capability, and I/O failures and adds platform-neutral `InvalidConfiguration(String)` for contradictory selectors, ambiguous matches, unsupported capabilities, and impossible nominal timing.

### Cleanup follows ownership and statistics survive Python consumption

There is no new public `close()` operation. Dropping the owning Rust bus or Robot closes the TX input, requests shutdown, cancels and collects pending transfers, resets channel 0, drops endpoint/interface references, and joins the worker. `Drop` performs best-effort cleanup without leaving a detached worker.

The concrete bus exposes read-only counters such as RX received, RX dropped, TX accepted, and TX completed. The Python wrapper retains a clone of this read-only statistics handle after its consumed-once transport handle is taken by `RobotBuilder`; this does not restore send capability or allow a second add.

## Risks / Trade-offs

- **`nusb` 0.2 cancellation and multiplexing details may differ from older examples** → Pin one compatible 0.2 release, create an early macOS lifecycle test for simultaneous command/completion/shutdown handling, and collect all cancelled completions before interface release.
- **Firmware echo behavior may vary between adapters** → Unit-test echo-credit state transitions and hardware-test the OpenYAM/CANable device; keep echoes internal and fail visibly on impossible bookkeeping.
- **An 8,192-frame queue can retain significant history after a stalled consumer** → Preserve the tested reference initially, evict oldest on overflow, expose drops, require zero drops and no stale feedback in nominal acceptance, and tune internally only with measurement.
- **Drop cannot return shutdown errors** → Exercise the fallible internal shutdown routine directly in tests and record/log best-effort Drop failures.
- **macOS CI has no USB hardware** → Separate protocol/lifecycle tests behind fake endpoints from a documented hardware acceptance run, while CI still proves compilation, wheel tagging, clean installation, imports, and conditional exports.
- **Removing unsupported transport exports is a source-level breaking change** → Document the target-native API and update every internal example/config/import so supported platforms never reference an absent type.

## Migration Plan

1. Land the upstream transport, target-specific API, tests, docs, and macOS wheels without modifying DimOS.
2. Publish a release containing both Linux and macOS platform wheels.
3. In a separate downstream change, remove the Linux dependency marker, select `SocketCanBus` on Linux and `GsUsbBus` on macOS, and delete the obsolete OpenYAM Python routing while retaining the Galaxea Python implementation.
4. Roll back by pinning DimOS to the previous upstream release; no persistent data or device migration is required.

## Open Questions

No public design decisions remain open. Implementation must verify two facts before the worker is considered complete:

- The exact pinned `nusb` endpoint/event-loop pattern that promptly cancels and joins on macOS without an external async runtime.
- The target adapters' TX echo timing and error behavior under sustained OpenYAM traffic.

