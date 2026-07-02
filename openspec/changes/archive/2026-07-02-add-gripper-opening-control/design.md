## Context

The current gripper support is a one-motor specialization over `MotorGroup`. Rust and Python expose raw motor-oriented controls (`mit_control`, `pos_vel_control`, `pos_force_control`, `set_mode`, `refresh`, `enable`, `disable`) and `RobotBuilder.add_gripper` enforces the one-motor shape. That is useful for hardware bring-up, but it does not express the end-user concept of commanding the gripper opening.

The project language now distinguishes normalized **opening** (`0.0` closed, `1.0` open), per-session **opening calibration**, and static **opening direction**. The formal gripper API should make those concepts explicit while preserving the raw controls for advanced use.

## Goals / Non-Goals

**Goals:**
- Provide a single normalized gripper command API: `set_opening(opening, current=...)`.
- Provide `open()` and `close()` as convenience methods for `set_opening(1.0)` and `set_opening(0.0)`.
- Provide a configured default opening current with per-call current overrides.
- Calibrate gripper opening endpoints automatically during the robot enable lifecycle, before normal opening commands are accepted.
- Require explicit gripper opening direction configuration so calibration knows which way opens the mechanism.
- Fail the robot enable lifecycle if opening calibration fails.
- Demonstrate the normalized API in both Python and Rust examples.

**Non-Goals:**
- Physical force calibration in Newtons or other jaw-force units.
- Fully automatic discovery of opening direction.
- Blocking “move and wait until reached” semantics for `set_opening`.
- Removing or deprecating raw gripper controls.
- Automatic calibration hidden inside the first `set_opening` call.

## Decisions

### Normalized API names opening, not position

Use `set_opening(opening, ...)` because raw motor position already means radians/encoder position elsewhere in the project. `opening` is the user-facing aperture command: `0.0` closed and `1.0` open.

Alternatives considered:
- `set_position`: rejected because it conflicts with raw motor position.
- `move_to`: rejected because it hides the normalized domain and can imply blocking movement.
- `control`: rejected as too vague for the primary gripper operation.

### Opening commands are non-blocking commands

`set_opening`, `open`, and `close` queue or send commands in the same style as existing gripper and arm methods. They do not wait until the measured motor state reaches the target.

Alternatives considered:
- Blocking until target reached: rejected because it would be inconsistent with the current `Robot.tick`-driven API and would need tolerance/timeout semantics not yet designed.
- Lazy calibration on first command: rejected because the first ordinary command would unexpectedly move to both stops.

### Out-of-range openings are rejected

Opening values outside `[0.0, 1.0]` return an error / raise `ValueError`. The API does not silently clamp.

Alternatives considered:
- Clamp: rejected because it hides configuration and caller bugs around hardware movement.

### Optional regulation is current, not force

The first formal API exposes optional per-unit motor current, because the current low-level command path is `pos_force_control(q, dq, i_pu)`. The API should not call this `force` unless the library can map it to a calibrated physical jaw force.

Opening commands resolve current by precedence: per-call current first, configured default opening current second, and a conservative library default only if no configured default is present. `open()` and `close()` accept the same optional current override as `set_opening()`.

Alternatives considered:
- `force`: rejected for now because it would imply a physical unit or calibrated jaw force that the system does not provide.
- Mutable runtime `set_default_current`: rejected for the first version because hidden mutable defaults are harder to audit during hardware work than configuration plus explicit per-call override.

### Opening endpoints are session-calibrated during enable

Each gripper discovers its open and closed motor positions during `Robot.enable()` / Python context-manager entry. The enable lifecycle enters an opening calibration stage after connecting and before the robot becomes ready for normalized opening control.

The calibration sequence uses the configured opening direction to touch the open stop with small current, records the open endpoint, then touches the closed stop in the opposite direction and records the closed endpoint. The exact stop-detection strategy should be conservative and bounded by timeout, minimum movement/span checks, and current/velocity limits.

Alternatives considered:
- Static `open_position` / `closed_position` config: rejected because the desired first version should discover session endpoints on startup.
- Completely automatic direction discovery: rejected because guessing the direction is unsafe.
- Manual calibration call: rejected because the chosen lifecycle model should include calibration as a normal stage before ready.

### Opening direction is configuration, not a runtime command

Config uses explicit values such as `"increasing_position"` and `"decreasing_position"`. Rust can store this as a typed enum internally. End users do not change opening direction during normal control.

Alternatives considered:
- Numeric signs (`1` / `-1`): rejected because they are less self-documenting and easier to confuse.

### Calibration failure fails the lifecycle

If opening calibration times out, detects too little motion/span, or otherwise cannot establish endpoints, `Robot.enable()` fails and the program should fail unless the caller explicitly catches the error. Normalized opening commands are unavailable without successful calibration.

Alternatives considered:
- Leave the robot partially enabled but mark the gripper failed: rejected because most failures imply connection, configuration, or hardware setup problems and should stop the normal program path.

## Risks / Trade-offs

- Calibration motion can contact mechanical stops → use small current, bounded velocity, explicit safety messaging in examples, and fail on timeout.
- Automatic calibration during enable may surprise users who previously expected enable to only enable motors → document the lifecycle and require `opening_direction` only for grippers that opt into normalized opening support.
- Per-unit current is less friendly than physical force → use honest naming now and leave physical force calibration for a later design.
- Existing raw gripper examples may compete with the new semantic example → make normalized opening the primary example and keep raw controls as advanced/reference examples.
- Calibration state adds lifecycle complexity → keep the state model small: uncalibrated, calibrating, ready, failed.

## Migration Plan

- Add normalized opening support as additive API; existing raw gripper commands continue to work.
- Existing gripper configs without `opening_direction` can continue to use raw gripper controls, but normalized opening commands require opening calibration and therefore opening direction. Configs may provide a default opening current so `set_opening`, `open`, and `close` do not need per-call current arguments.
- Update docs, stubs, examples, and changelog limitations so users see normalized opening as the preferred gripper path.

## Open Questions

- Exact stop-detection heuristic and default calibration current/velocity/timeout values.
- Whether normalized opening support should be opt-in per gripper via `opening_direction`, or whether every `kind = "gripper"` must configure it once this feature lands.
- Whether Rust should represent optional current as `Option<f64>` or provide overload-like convenience methods such as `set_opening(opening)` and `set_opening_with_current(opening, current)`.
