## ADDED Requirements

### Requirement: Builds expose only their native hardware transport

The Rust crate and Python package SHALL expose `SocketCanBus` on Linux and `GsUsbBus` on macOS. Both platforms SHALL expose `MockCanBus`. A supported platform build MUST include its native hardware transport without a user-facing feature switch, and MUST NOT expose an unsupported transport placeholder.

#### Scenario: Linux exposes SocketCAN
- **WHEN** the Rust crate and Python extension are built for Linux
- **THEN** `SocketCanBus` and `MockCanBus` are available and `GsUsbBus` is absent

#### Scenario: macOS exposes gs_usb
- **WHEN** the Rust crate and Python extension are built for macOS
- **THEN** `GsUsbBus` and `MockCanBus` are available and `SocketCanBus` is absent

#### Scenario: macOS dependency is self-contained
- **WHEN** `can-motor-control` is built or installed on macOS
- **THEN** native gs_usb support uses `nusb`/IOKit and requires neither system `libusb` nor PyUSB

### Requirement: GsUsbBus selects exactly one adapter

`GsUsbBus` construction SHALL require a vendor ID and product ID and SHALL accept either a USB serial number or an enumeration index as the device selector. Serial number and index MUST be mutually exclusive. If neither is supplied, construction SHALL use index zero. Enumeration index SHALL be documented as unstable deployment identity.

#### Scenario: Serial selects one matching adapter
- **WHEN** a serial number identifies exactly one adapter among the matching VID/PID devices
- **THEN** construction opens that adapter regardless of enumeration order

#### Scenario: Index selects among VID/PID matches
- **WHEN** index `N` is supplied and at least `N + 1` matching VID/PID adapters exist
- **THEN** construction opens match `N` in the enumerated match set

#### Scenario: Omitted selector chooses first match
- **WHEN** neither serial number nor index is supplied
- **THEN** construction behaves as if index zero were supplied

#### Scenario: Contradictory selectors are rejected
- **WHEN** both serial number and index are supplied
- **THEN** construction returns `TransportError::InvalidConfiguration`

#### Scenario: Serial selection is ambiguous
- **WHEN** more than one matching adapter reports the requested serial number
- **THEN** construction returns `TransportError::InvalidConfiguration` without opening an arbitrary match

### Requirement: Construction produces a ready clean classical-CAN channel

Construction SHALL complete device selection, open/configuration, interface claim, descriptor-driven endpoint discovery, gs_usb host-format negotiation, device and channel validation, reset, nominal bit timing, channel start, stale-input discard, and queued-IN priming before returning success. The first version SHALL use gs_usb channel 0 in normal classical-CAN mode and SHALL NOT expose channel, listen-only, hardware timestamp, or CAN-FD configuration.

#### Scenario: Endpoints come from descriptors
- **WHEN** a compatible adapter exposes bulk-IN and bulk-OUT addresses that differ from common defaults
- **THEN** construction uses the descriptor addresses and does not assume OUT endpoint `0x02`

#### Scenario: Host format is negotiated
- **WHEN** construction initializes a selected adapter
- **THEN** it sends the gs_usb host-format request before decoding device control structures or data frames

#### Scenario: Previous session left the channel started
- **WHEN** construction opens an adapter after a crashed or killed session
- **THEN** it resets channel 0 before programming timing and starting the channel

#### Scenario: Clean session boundary is established
- **WHEN** stale frames remain in the adapter after channel start
- **THEN** construction discards them until it observes the configured quiet interval before reporting ready

#### Scenario: Input does not become quiet
- **WHEN** the bounded stale-input discard cannot observe a quiet interval
- **THEN** construction fails instead of exposing stale input to the robot

#### Scenario: Unsupported channel layout is rejected
- **WHEN** device information does not expose usable channel 0 or descriptors do not expose exactly one usable bulk-IN and bulk-OUT path
- **THEN** construction returns `TransportError::InvalidConfiguration`

### Requirement: Initialization retries only transient re-enumeration failures

Construction SHALL retry the complete initialization transaction after transient device disappearance or not-ready errors under one configurable deadline whose default is five seconds. It MUST fail deterministic selection, permission, descriptor, protocol, capability, and timing errors immediately.

#### Scenario: Device is visible before control transfers are ready
- **WHEN** enumeration finds the selected adapter but open, claim, or control setup reports a transient re-enumeration error
- **THEN** construction drops the partial handles and retries from device enumeration

#### Scenario: Transient failures exceed deadline
- **WHEN** the adapter does not complete initialization before the shared deadline
- **THEN** construction returns a transport error that identifies the selector and elapsed deadline

#### Scenario: Permission is denied
- **WHEN** macOS denies access to the selected adapter or interface
- **THEN** construction returns `TransportError::PermissionDenied` without waiting through the retry deadline

### Requirement: Nominal bit timing comes from device capabilities

`GsUsbBus` SHALL calculate nominal classical-CAN timing from the adapter-reported CAN clock and timing constraints for the requested bitrate. It MUST validate every timing field against those constraints and MUST NOT depend on hardcoded clock-specific timing tables.

#### Scenario: One megabit timing on a 170 MHz adapter
- **WHEN** a compatible adapter reports a 170 MHz CAN clock and constraints that can produce 1 Mbit/s
- **THEN** construction calculates valid timing, programs it, and starts the channel

#### Scenario: Requested bitrate is impossible
- **WHEN** no timing tuple within the reported constraints produces the requested bitrate
- **THEN** construction returns `TransportError::InvalidConfiguration` with the requested bitrate and reported constraints

### Requirement: GsUsbBus preserves the synchronous CanBus contract

`GsUsbBus` SHALL implement `CanBus` with classical-CAN capabilities, standard and extended identifiers, nonblocking send acceptance, immediate memory-only inbound drain, and `raw_fd() == None`. `send()` success SHALL mean the transport accepted responsibility for attempting the frame, not that the frame reached the wire or another node acknowledged it.

#### Scenario: Standard frame is accepted
- **WHEN** `send()` receives a valid standard classical frame and TX capacity is available
- **THEN** it returns `Ok(())` after a bounded in-memory enqueue without waiting for USB

#### Scenario: Extended frame round-trips
- **WHEN** an extended classical frame is sent or received
- **THEN** its 29-bit identifier and extended-ID flag are preserved by gs_usb encoding and decoding

#### Scenario: CAN-FD is rejected
- **WHEN** `send()` receives a CAN-FD frame
- **THEN** it returns `TransportError::FdFrameOnNonFdBus` without USB I/O

#### Scenario: TX path is full
- **WHEN** the bounded TX path cannot accept another frame
- **THEN** `send()` returns `TransportError::SendBufferFull` and does not discard an older outbound frame

#### Scenario: Inbound queue is empty
- **WHEN** no decoded CAN frames are currently queued
- **THEN** `drain_inbound_nonblocking()` immediately returns an empty vector

#### Scenario: Inbound queue contains frames
- **WHEN** decoded CAN frames are queued
- **THEN** `drain_inbound_nonblocking()` immediately returns all currently queued frames in arrival order without waiting for USB

### Requirement: One worker continuously services USB traffic

One background worker SHALL exclusively own the selected device, interface, bulk endpoints, queued transfers, echo credits, and terminal worker state. It SHALL keep multiple bulk-IN transfers pending and reuse completed buffers so the robot control thread never waits for USB traffic.

#### Scenario: RX remains continuously submitted
- **WHEN** the bus is running under sustained feedback traffic
- **THEN** each successful bulk-IN completion is parsed and its buffer is resubmitted while the worker remains healthy

#### Scenario: Received CAN frame enters the RX queue
- **WHEN** a valid gs_usb host frame has echo ID `0xFFFFFFFF`
- **THEN** the worker converts it to one classical `CanFrame` and enqueues it for inbound drain

#### Scenario: TX echo remains internal
- **WHEN** a gs_usb host frame has a valid allocated echo ID other than `0xFFFFFFFF`
- **THEN** the worker releases the matching TX credit and does not expose the frame to the motor codec

#### Scenario: RX queue overflows
- **WHEN** a decoded frame arrives while the fixed 8,192-frame RX queue is full
- **THEN** the worker removes the oldest frame, retains the new frame, and increments the RX-drop counter

#### Scenario: Echo bookkeeping is impossible
- **WHEN** the worker receives an out-of-range, duplicate, or unallocated TX echo ID
- **THEN** it records a protocol failure and does not expose that frame as CAN input

### Requirement: Runtime USB failures fail closed

`GsUsbBus` SHALL distinguish expected shutdown cancellation from runtime USB failure. It MAY attempt one clear-halt recovery for an endpoint stall; any failed recovery, disconnect, malformed transfer, or other terminal USB error MUST fault the worker, reject new sends, and become observable through a subsequent transport operation. It MUST NOT reconnect automatically.

#### Scenario: Endpoint halt recovers once
- **WHEN** an endpoint stalls and one clear-halt operation plus resubmission succeeds
- **THEN** the worker resumes normal transfer processing without reconstructing the bus

#### Scenario: Adapter disconnects while running
- **WHEN** the selected adapter disconnects after construction
- **THEN** the worker enters a terminal fault and the next `send()` or `drain_inbound_nonblocking()` returns `TransportError::Io`

#### Scenario: Faulted bus rejects sends
- **WHEN** the worker has entered a terminal fault
- **THEN** `send()` rejects the frame and does not enqueue it for later reconnection

### Requirement: Ownership-driven shutdown is deterministic

Dropping `GsUsbBus` SHALL close the TX input, reject further work, request worker shutdown, cancel and collect pending IN and OUT transfers, reset channel 0, release all endpoint and interface ownership, and join the worker. The transport SHALL NOT expose a separate public close operation or leave a detached worker.

#### Scenario: Quiet bus is dropped
- **WHEN** the owning Robot or standalone `GsUsbBus` is dropped while IN transfers are pending
- **THEN** pending transfers are cancelled and the worker terminates before Drop completes

#### Scenario: Bus is dropped with TX pending
- **WHEN** the bus is dropped with queued or submitted outbound frames
- **THEN** no new frames are accepted, pending USB transfers are cancelled or collected, and the interface is released

#### Scenario: Adapter is reopened immediately
- **WHEN** one `GsUsbBus` is dropped and a new instance selects the same attached adapter
- **THEN** the new construction can reset, configure, receive, and transmit without a stale interface claim or wedged RX state

### Requirement: Python preserves consumed-once ownership and read-only statistics

The macOS Python module SHALL expose `GsUsbBus` with keyword-only VID, PID, optional mutually exclusive serial/index selectors, bitrate, and initialization timeout. `RobotBuilder.add_bus()` SHALL consume it exactly once like existing transports. Its wrapper SHALL retain read-only shared runtime counters after consumption without retaining transport operations or permitting reuse.

#### Scenario: Python bus is consumed once
- **WHEN** one Python `GsUsbBus` is added to a builder and then added again
- **THEN** the second add raises the same consumed-transport error used by existing bus wrappers

#### Scenario: GIL is released during construction
- **WHEN** Python constructs `GsUsbBus` while initialization waits or retries
- **THEN** the extension does not hold the Python GIL for the blocking readiness wait

#### Scenario: Statistics remain readable after consumption
- **WHEN** the wrapper has been consumed by `RobotBuilder.add_bus()`
- **THEN** read-only RX-received, RX-dropped, TX-accepted, and TX-completed counters remain accessible

#### Scenario: Consumed wrapper cannot regain transport ownership
- **WHEN** code reads statistics from a consumed wrapper
- **THEN** it cannot send frames, close the owned transport separately, or add the bus to another builder

### Requirement: Hardware acceptance proves OpenYAM operation

The native transport SHALL pass a documented hardware acceptance run on the target macOS arm64 OpenYAM/gs_usb setup before release. The run SHALL exercise connect, feedback, sustained command/refresh traffic, disable, cleanup, and immediate reconstruction while collecting average, p95, and p99 tick time, CPU utilization, RX drops, and feedback completeness.

#### Scenario: Sustained OpenYAM workload succeeds
- **WHEN** the system runs 7,500 control cycles at 250 Hz with all seven motors attached
- **THEN** every motor continues producing fresh feedback, RX drops remain zero, and no cycle exceeds the existing 12.5 ms control deadline

#### Scenario: Repeated reconstruction succeeds
- **WHEN** the acceptance run repeatedly destroys and reconstructs the bus for the same attached adapter
- **THEN** each construction completes within its five-second initialization deadline and receives fresh feedback without manual unplugging

