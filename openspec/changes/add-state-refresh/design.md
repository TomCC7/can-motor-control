## Context

`examples/read_single_motor.py` enables a motor then loops `robot.tick()` only.
On hardware, state freezes after the first reading because:

- `tick()` is receive-only (`crates/dm-control/src/robot.rs:155`; the
  `robot-composition` spec "Sends are never coupled to inbound reads").
- A Damiao motor only replies with state **in response to a frame we send**
  (`dm-codec` decode). With nothing sent after enable, no further replies arrive.

openarm reads state via a **refresh query** (`examples/demo.cpp`:
`refresh_all(); recv_all(timeout)`). `create_refresh_command()` sends `0x7FF` +
`[id_lo, id_hi, 0xCC, 0,0,0,0,0]` — `refresh_motor_status`, which requests
feedback without commanding motion. This repo already has the identical encoder
(`crates/dm-codec/src/ext.rs:88` `encode_refresh`) but it is mislabeled "refresh
from flash", unreachable from a connected robot, and unexposed to Python.

## Goals / Non-Goals

**Goals:**
- Expose a `refresh` state-query primitive from the codec through `Group`/`Robot`
  and the Python API, vendor-agnostically.
- Make `read_single_motor.py` show live state via the openarm `refresh → tick`
  loop, with no motion commanded.
- Keep it a primitive: one refresh frame per motor per call.

**Non-Goals:**
- The `step()`/staged-command control loop (one frame/motor/cycle with
  auto-refresh-when-idle) — separate `add-control-loop-step` change.
- Changing a motor's control mode, or addressing enable-time stiffness (firmware/
  mode behavior; refresh commands no torque and does not change it).
- Vendor protocols other than Damiao (the trait default makes them no-ops).

## Decisions

### 1. Generic trait method over a Damiao downcast
Add `encode_refresh` to `MotorCodec` with a default `Ok(None)`, rather than
having `dm-control` downcast `Box<dyn MotorCodec>` to `DamiaoCodecExt`. *Why:*
keeps `Group`/`Robot` vendor-agnostic (matches `encode_enable`/`encode_disable`
shape) and avoids leaking Damiao specifics into the control layer. The existing
`DamiaoCodecExt::encode_refresh` bytes are reused by the trait impl. *Alternative:*
the `Arm::codec_ext::<DamiaoCodec>()` downcast hinted in `ext.rs:5` — rejected as
a vendor leak and more surface than needed.

### 2. `Option<CanFrame>` return, skip on `None`
`encode_refresh -> Result<Option<CanFrame>, CodecError>`; codecs without a refresh
return `Ok(None)` and `refresh_all` skips them. *Why:* refresh is genuinely
vendor-optional; a hard error or a panic would be wrong for a generic robot with
mixed/none-supporting codecs.

### 3. Refresh is send-only; `tick` still owns reads
`refresh_all` only encodes + `bus.transport.send` per motor — it never drains.
*Why:* preserves the architecture's single-inbound-owner invariant; the read loop
is explicitly `refresh()` then `tick()`. This mirrors openarm's `refresh_all()`
then `recv_all()`.

### 4. Reader loop = `refresh → tick → render`
`read_single_motor.py` sends a refresh (no motion) then ticks to receive, each
cycle. One frame per motor per cycle, no redundancy, openarm-aligned. Remove the
debug print; the Python `Motor` getters are a live view so no re-fetch is needed.

### 5. Fix the mislabel
`ext.rs` doc "refresh from flash" → "request a state-feedback frame (Damiao
`refresh_motor_status`, 0xCC on 0x7FF) — commands no motion", with a unit test
pinning the layout to openarm's `create_refresh_command`.

## Risks / Trade-offs

- **0xCC semantics were only doc-asserted here** → mitigated: the byte layout is
  byte-for-byte openarm's `create_refresh_command` (confirmed against the
  reference), and the decode path already handles the feedback frame. Hardware
  verification is the user's `--interface can0` run.
- **Two ways to elicit state (control command vs refresh)** → for a *reader* only
  refresh is sent, so no redundancy. Mixing a control command and a refresh in the
  same cycle would double-reply; that coordination is the job of the future
  `step()` change, explicitly out of scope here.
- **Stiffness on enable persists** → refresh commands no torque; it cannot relax a
  motor that firmware holds in a non-MIT mode. Documented as out of scope.

## Migration Plan

Additive — no breaking changes. The trait method default keeps every existing
codec compiling. Existing examples that send control commands are unaffected;
only the read-only examples adopt `refresh`.

## Open Questions

- Whether to also convert `02_single_motor_read_state.py` /
  `05_single_arm_state_monitor.py` in this change or defer — included as tasks but
  low-risk (same motion-free pattern).
