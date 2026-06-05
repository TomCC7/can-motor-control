## Context

The repository is pre-0.1.0 and not yet published, so this is the last low-cost
point to fix public identity. Today the repo and release-facing surfaces are
split across:

- repo/product: `can-motor-control`
- Rust runtime crate: `can-motor-control`, imported as `can_motor_control`
- Python distribution/import: `can_motor_control`
- Python native module: `can_motor_control._native`
- vendor crate: `damiao-codec`, imported as `damiao_codec`
- shared trait crate: `motor-codec`, imported as `motor_codec`

That identity no longer matches the architecture. The runtime crate is already
vendor-agnostic above the codec seam: it owns CAN transport, bus routing, robot
composition, arms, grippers, generic groups, and Python bindings. Damiao support
is one vendor codec; future vendors such as Robostride are explicitly planned.

Name availability checks already performed during exploration found 404/not
taken responses for `can-motor-control` and normalized variants on crates.io,
PyPI, and docs.rs. Public GitHub repositories with the same/similar name exist
outside the target namespace, so the repo name is not globally unique, but
`github.com/cc/can-motor-control` returned 404 and is likely available under the
current owner.

## Goals / Non-Goals

**Goals:**

- Make `can-motor-control` the canonical release-facing project name.
- Make Rust users depend on `can-motor-control` and import `can_motor_control`.
- Make Python users install `can-motor-control` and import `can_motor_control`.
- Keep Damiao as a vendor namespace (`damiao`) rather than the project identity.
- Update all active docs, examples, tests, CI, PyO3 metadata, stubs, package
  metadata, and publish instructions in one coherent refactor.
- Preserve the existing runtime API shape: `Robot`, `RobotBuilder`, `Arm`,
  `Gripper`, `Motor`, `MotorGroup`, `CanBus`, `SocketCanBus`, `MotorCodec`, and
  the `damiao` vendor module remain concepts with the same semantics.

**Non-Goals:**

- Add a new vendor implementation such as Robostride.
- Rename domain types from `Motor*` to `Actuator*`.
- Add compatibility shims for the old `can_motor_control` Python import or old
  `can-motor-control` Rust package name. The project is unpublished, so old names are
  drafts, not external contracts.
- Change control-loop behavior, CAN-FD behavior, config schema semantics, or
  hardware bring-up safety behavior beyond naming text.
- Publish packages or create/rename the GitHub repository during implementation.

## Decisions

### Decision 1: Use `can-motor-control` as package/distribution and `can_motor_control` as import/module

Rust packages and Python distributions can use hyphenated names, while Rust and
Python imports must use identifier-compatible underscores. The implementation
will therefore use:

```text
Project / repo / docs title: can-motor-control
Rust package:               can-motor-control
Rust crate import:          can_motor_control
Python distribution:        can-motor-control
Python import package:      can_motor_control
PyO3 native module:         can_motor_control._native
```

Alternatives considered:

- `can-control`: shorter, but too close to generic CAN tooling.
- `can-motor-rs`: available on registries, but `-rs` is discouraged in Rust crate
  naming and weakens the Python story.
- `canmotion`: distinctive, but reads like trajectory/motion planning rather
  than hardware motor control.

### Decision 2: Rename the primary runtime crate and Python binding crate together

The Rust runtime crate and the Python distribution expose the same conceptual
surface. Renaming only one would create confusing docs and examples. The Rust
crate directory can either be renamed to `crates/can-motor-control` or kept with
an internal path, but the published package name, library crate name, examples,
and imports must move together. Prefer renaming directories so path names match
published package names and reduce future grep noise.

Implementation target:

```text
crates/can-motor-control       -> crates/can-motor-control
crates/can-motor-control-py    -> crates/can-motor-control-py
python/can_motor_control       -> python/can_motor_control
```

### Decision 3: Rename `damiao-codec` to `damiao-codec`, keep `motor-codec` unless implementation decides otherwise

The abbreviation `dm` is ambiguous and contributed to the top-level naming
problem. `damiao-codec` is clearer for publishing and remains vendor-specific.
The shared trait crate `motor-codec` is already vendor-neutral, concise, and
accurate; renaming it would increase churn without much user-facing benefit.

Default target:

```text
damiao-codec       -> damiao-codec
damiao_codec       -> damiao_codec
motor-codec    -> motor-codec
motor_codec    -> motor_codec
```

If implementation discovers that `damiao-codec` is unavailable or undesirable,
it may keep `damiao-codec`, but must record that decision in the final notes and
release docs.

### Decision 4: Do not provide old-name compatibility aliases

The old names have not shipped. Adding `can_motor_control` Python shims or Rust alias
packages would create two public identities before the first release and make
docs/tests more complex. The implementation should do a clean rename and update
all references.

Historical OpenSpec archives may retain old names where they describe past
planning context, but any active/current requirement, docs, example, or release
instruction must use the new identity.

### Decision 5: Treat PyO3 module metadata as a first-class rename surface

The docs site imports the native module through griffe/mkdocstrings. A simple
file-path/package rename is not enough: `#[pyclass(module = "...")]`,
`create_exception!(...)`, pure-Python re-exports, preload modules, stubs, and
logging namespaces must agree on `can_motor_control`. Otherwise docs can build
with stale aliases or render classes under the wrong module.

### Decision 6: Verify through the user surfaces, not just compile checks

The rename is only correct if users can install/import/depend on the new names.
Verification must include:

- `cargo metadata`, `cargo build --workspace`, `cargo test --workspace`, and
  `cargo clippy --workspace --all-targets -- -D warnings`
- no-std builds for `motor-codec` and the selected Damiao codec crate
- `maturin build --release --strip` for the renamed Python package
- clean-venv wheel install followed by `import can_motor_control`
- `pytest tests/python -v`
- `make docs-build` with mkdocstrings importing `can_motor_control._native`
- `cargo publish --dry-run` in dependency order
- repository grep for stale old public names outside allowed historical context

## Risks / Trade-offs

- **[Risk] Rename touches many files and can leave mixed old/new imports.** →
  Mitigation: use broad repository searches, AST-aware import checks where
  possible, and explicit stale-name grep tasks before verification completes.
- **[Risk] PyO3 class metadata or exceptions still report `can_motor_control`.** →
  Mitigation: include a Python inspection smoke test for public classes,
  exceptions, `__all__`, and `can_motor_control.damiao`.
- **[Risk] docs build fails because griffe preloads the old module.** →
  Mitigation: update `mkdocs.yml` preload modules and docs paths together with
  the Python package directory; run `make docs-build`.
- **[Risk] Registry availability changes before publication.** → Mitigation:
  re-run crates.io/PyPI/docs.rs/GitHub checks during implementation and again
  immediately before publishing.
- **[Risk] Historical OpenSpec files create noisy stale-name grep results.** →
  Mitigation: distinguish active release-facing surfaces from archived history;
  update archived references only when they describe current behavior or would
  confuse future implementation.
- **[Risk] Renaming `damiao-codec` to `damiao-codec` expands scope beyond the project
  name.** → Mitigation: keep it as a deliberate decision because it removes the
  same abbreviation problem at the vendor boundary; allow retaining `damiao-codec`
  if availability or churn argues against the rename.

## Migration Plan

1. Re-run name availability checks for `can-motor-control`, `can_motor_control`,
   `can.motor.control`, `damiao-codec`, and selected companion names.
2. Rename package metadata and directories with git-aware moves where possible.
3. Update Rust imports, dependency aliases, package names, examples, tests, and
   CI commands.
4. Update PyO3 module names, Python package directory, stubs, re-export modules,
   exceptions, logging, docs preload modules, and wheel smoke checks.
5. Update README, docs, CHANGELOG, configs comments, OpenSpec current specs, and
   release instructions.
6. Run verification, fix stale references, and repeat until the user-facing
   surfaces work end-to-end.

Rollback before publication is a normal git revert of the rename change. After
publication, rollback would require publishing replacement versions and is out
of scope for this pre-release rename.

## Open Questions

- Should `motor-codec` remain as-is, or should it become `can-motor-codec` for a
  stronger crate family? Current recommendation: keep `motor-codec`.
- Should archived OpenSpec artifacts be bulk-updated for search cleanliness, or
  left as historical records except where they describe current release behavior?
  Current recommendation: update active/current behavior, leave clearly archived
  historical reasoning alone.
