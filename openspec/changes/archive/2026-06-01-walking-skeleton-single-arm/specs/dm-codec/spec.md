## ADDED Requirements

### Requirement: DamiaoCodec implements MotorCodec

The `dm-codec` crate SHALL define a `DamiaoCodec` struct that implements `motor_codec::MotorCodec`. The struct MUST be constructible via `DamiaoCodec::new()` with no required arguments and MUST be `Send + Sync`. It MUST be `Default` so it can be constructed implicitly where ergonomic.

#### Scenario: Constructible and trait-object-compatible
- **WHEN** a developer writes `let codec: Box<dyn MotorCodec> = Box::new(DamiaoCodec::new());`
- **THEN** the code compiles and `codec.vendor_name() == "damiao"`

### Requirement: Supports the Damiao motor SKUs targeted by v1

`DamiaoCodec::supports` SHALL return `true` for these motor type discriminants: DM3507, DM4310, DM4310_48V, DM4340, DM4340_48V, DM6006, DM8006, DM8009, DM10010L, DM10010, DMH3510, DMH6215, DMG6220. Each MUST have an entry in the internal limit-param table with the `(p_max, v_max, t_max)` values published by Damiao. Discriminants outside this set MUST return `false`.

#### Scenario: Every listed SKU has limits
- **WHEN** `DamiaoCodec::limits(MotorTypeId::Damiao(disc))` is called for each listed discriminant
- **THEN** every call returns `Ok(Limits { .. })` with positive p_max, v_max, and t_max

#### Scenario: Unknown discriminant rejected
- **WHEN** `DamiaoCodec::supports(MotorTypeId::Damiao(0xFFFF))` is called
- **THEN** it returns `false`
- **AND** `DamiaoCodec::limits(MotorTypeId::Damiao(0xFFFF))` returns `Err(CodecError::UnknownMotorType { vendor: "damiao", type_id: 0xFFFF })`

### Requirement: MIT command encoding matches Damiao bit layout

`DamiaoCodec::encode_command` for `Command::Mit` SHALL produce an 8-byte payload with this bit packing (matching both reference implementations):
- bits 0-15:  `q_uint`   (16 bits) — `q` scaled to `[-p_max, +p_max]`
- bits 16-27: `dq_uint`  (12 bits) — `dq` scaled to `[-v_max, +v_max]`
- bits 28-39: `kp_uint`  (12 bits) — `kp` scaled to `[0, 500]`
- bits 40-51: `kd_uint`  (12 bits) — `kd` scaled to `[0, 5]`
- bits 52-63: `tau_uint` (12 bits) — `tau` scaled to `[-t_max, +t_max]`

The frame ID MUST equal `motor.send_id` (no offset is added for MIT mode). The frame MUST have `len == 8`, `flags.contains(EXTENDED_ID) == false`, and `flags.contains(FD_FORMAT) == false` (v1 always emits classical regardless of bound bus capabilities).

#### Scenario: Zero command on DM4340 packs to known bytes
- **WHEN** `encode_command(motor_ref_dm4340, &Command::Mit { kp: 0.0, kd: 0.0, q: 0.0, dq: 0.0, tau: 0.0 })` is called
- **THEN** the returned frame payload equals the bit-packed encoding of `(p_max midpoint, v_max midpoint, 0, 0, t_max midpoint)` consistent with the reference Python implementation

#### Scenario: Out-of-range torque
- **WHEN** `encode_command` is called with `tau` greater than `t_max` for the motor
- **THEN** it returns `Err(CodecError::OutOfRange { field: "tau" })`

### Requirement: PosVel uses CAN ID offset 0x100

`DamiaoCodec::encode_command` for `Command::PosVel` SHALL produce a frame with `id == 0x100 + motor.send_id`, payload bytes 0-3 = `f32_le_bytes(q)`, bytes 4-7 = `f32_le_bytes(dq)`. Frame length MUST equal 8.

#### Scenario: PosVel frame ID and layout
- **WHEN** `encode_command(motor_ref{send_id=0x01}, &Command::PosVel { q: 1.0, dq: 2.0 })` is called on a Damiao motor
- **THEN** the returned frame has `id == 0x101`, `len == 8`, `&payload[0..4] == 1.0_f32.to_le_bytes()`, `&payload[4..8] == 2.0_f32.to_le_bytes()`

### Requirement: Vel uses CAN ID offset 0x200

`DamiaoCodec::encode_command` for `Command::Vel` SHALL produce a frame with `id == 0x200 + motor.send_id`, payload bytes 0-3 = `f32_le_bytes(dq)`, bytes 4-7 = zeros. Frame length MUST equal 8.

#### Scenario: Vel frame ID
- **WHEN** `encode_command(motor_ref{send_id=0x01}, &Command::Vel { dq: 1.5 })` is called
- **THEN** the returned frame has `id == 0x201` and `&payload[0..4] == 1.5_f32.to_le_bytes()`

### Requirement: PosForce uses CAN ID offset 0x300

`DamiaoCodec::encode_command` for `Command::PosForce` SHALL produce a frame with `id == 0x300 + motor.send_id`, payload bytes 0-3 = `f32_le_bytes(q)`, bytes 4-5 = `(dq * 100.0) as u16` little-endian, bytes 6-7 = `(i_pu * 10000.0) as u16` little-endian. Frame length MUST equal 8.

#### Scenario: PosForce frame ID and integer scaling
- **WHEN** `encode_command(motor_ref{send_id=0x01}, &Command::PosForce { q: 1.0, dq: 2.0, i_pu: 0.5 })` is called
- **THEN** the returned frame has `id == 0x301`, `&payload[4..6] == 200u16.to_le_bytes()`, `&payload[6..8] == 5000u16.to_le_bytes()`

### Requirement: encode_enable / encode_disable / encode_set_zero use the special command pattern

`encode_enable` SHALL produce a frame with `id == motor.send_id`, payload `[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]`. `encode_disable` MUST be identical except trailing byte `0xFD`. `encode_set_zero` MUST be identical except trailing byte `0xFE`. Frame length MUST equal 8.

#### Scenario: Enable frame bytes
- **WHEN** `encode_enable(motor_ref{send_id=0x05})` is called
- **THEN** the returned frame has `id == 0x05`, `len == 8`, `payload == [0xFF; 7] ++ [0xFC]`

### Requirement: decode parses Damiao state response frames

`DamiaoCodec::decode` SHALL recognize state response frames with command byte `0x11` and produce `Event::State`. The motor's recv ID is the source of the `motor_id` field. q, dq, tau MUST be unpacked from the bit-packed payload and scaled back into f64 using the motor-type's limits. `t_mos` and `t_rotor` MUST be decoded from the dedicated temperature bytes per Damiao's protocol.

#### Scenario: Round-trip MIT state
- **WHEN** a frame encoded by Damiao firmware in response to a MIT command is passed to `decode`
- **THEN** the returned event is `Ok(Some(Event::State { q, dq, tau, t_mos, t_rotor, motor_id }))` where each field equals the firmware-reported value within ±1 LSB of the encoded precision

#### Scenario: Non-Damiao frame returns Ok(None)
- **WHEN** a frame with ID outside any Damiao-known range is passed to `decode`
- **THEN** the returned result is `Ok(None)`

### Requirement: DamiaoCodecExt exposes the 0x7FF parameter sub-protocol

The `dm-codec` crate SHALL define a `DamiaoCodecExt` trait with at minimum these methods, all returning `CanFrame`:
- `encode_read_param(motor: MotorRef<'_>, rid: DamiaoRid) -> CanFrame` — packs a frame with ID `0x7FF`, command byte `0x33`, and the target motor's CAN ID + RID
- `encode_write_param<T: Into<ParamValue>>(motor: MotorRef<'_>, rid: DamiaoRid, val: T) -> CanFrame` — packs with command byte `0x55`
- `encode_save_to_flash(motor: MotorRef<'_>) -> CanFrame` — packs with command byte `0xAA`
- `encode_refresh(motor: MotorRef<'_>) -> CanFrame` — packs with command byte `0xCC`

`DamiaoCodec` MUST implement `DamiaoCodecExt`. The trait MUST NOT depend on `motor-codec`'s `MotorCodec` (it stands alone for downcast use).

#### Scenario: write_param frame layout
- **WHEN** `DamiaoCodec::encode_write_param(motor_ref{send_id=0x01}, DamiaoRid::MstId, 0x11_u32)` is called
- **THEN** the returned frame has `id == 0x7FF`, `len == 8`, `payload[0..2] == 0x01u16.to_le_bytes()`, `payload[2] == 0x55`, `payload[3] == DamiaoRid::MstId as u8`, `payload[4..8] == 0x11u32.to_le_bytes()`

### Requirement: DamiaoRid enumerates the parameter register IDs

The `dm-codec` crate SHALL define a `DamiaoRid` enum covering at minimum the register IDs documented in Damiao's protocol manual: `UV_Value`, `KT_Value`, `OT_Value`, `OC_Value`, `ACC`, `DEC`, `MAX_SPD`, `MST_ID`, `ESC_ID`, `TIMEOUT`, `CTRL_MODE`, `Damp`, `Inertia`, `hw_ver`, `sw_ver`, `SN`, `NPP`, `Rs`, `LS`, `Flux`, `Gr`, `PMAX`, `VMAX`, `TMAX`, `I_BW`, `KP_ASR`, `KI_ASR`, `KP_APR`, `KI_APR`, `OV_Value`, `GREF`, `Deta`, `V_BW`, `IQ_c1`, `VL_c1`, `can_br`, `sub_ver`. The enum MUST be `#[non_exhaustive]` and convertible to `u8` via `From`.

#### Scenario: RID to u8 conversion
- **WHEN** `u8::from(DamiaoRid::CTRL_MODE)` is called
- **THEN** the returned value equals the Damiao-documented byte value for `CTRL_MODE`

### Requirement: Damiao motor type strings parse to MotorTypeId

A `DamiaoCodec::parse_motor_type(s: &str) -> Option<MotorTypeId>` SHALL accept the documented Damiao SKU strings (`"DM4340"`, `"DM4310_48V"`, etc.) and return the corresponding `MotorTypeId::Damiao(disc)`. Unknown strings MUST return `None`. This is the function the TOML loader uses to resolve `type = "DM4340"` config fields.

#### Scenario: Known SKU parses
- **WHEN** `DamiaoCodec::parse_motor_type("DM4340")` is called
- **THEN** it returns `Some(MotorTypeId::Damiao(<DM4340 discriminant>))`

#### Scenario: Unknown SKU returns None
- **WHEN** `DamiaoCodec::parse_motor_type("DM_DOES_NOT_EXIST")` is called
- **THEN** it returns `None`

### Requirement: dm-codec crate is no_std

The `dm-codec` crate SHALL declare `#![no_std]` and depend only on `motor-codec` and `core`/`alloc`. It MUST NOT depend on `std`, `socketcan`, `serde`, or any IO crate.

#### Scenario: Compiles on bare-metal target
- **WHEN** `cargo build -p dm-codec --no-default-features --target thumbv7em-none-eabihf` is run
- **THEN** the build succeeds

### Requirement: DamiaoCodec emits classical CAN frames in v1 regardless of bus capability

`DamiaoCodec::bind_to_bus(caps)` SHALL store the supplied `BusCapabilities` but in v1 MUST NOT use it to switch to CAN-FD emission. Every encode method (`encode_enable`, `encode_disable`, `encode_set_zero`, `encode_command`) MUST return a `CanFrame` with `flags.contains(FD_FORMAT) == false` and `len == 8` regardless of whether the bound bus advertises `supports_fd == true`. CAN-FD frame emission by the Damiao codec is reserved for a future change and MUST NOT be added as a silent runtime behavior in v1.

#### Scenario: Damiao codec on classical bus emits classical
- **WHEN** `DamiaoCodec::new()` is bound to a bus with `supports_fd: false` and `encode_command` is called
- **THEN** the returned frame has `flags.contains(FD_FORMAT) == false`

#### Scenario: Damiao codec on FD-capable bus still emits classical in v1
- **WHEN** `DamiaoCodec::new()` is bound to a bus with `supports_fd: true` (constructed via `MockCanBus::with_capabilities`) and `encode_command` is called
- **THEN** the returned frame still has `flags.contains(FD_FORMAT) == false` and `len == 8`
