## 1. Docs toolchain dependencies

- [x] 1.1 Add a `docs` optional-dependency group (`mkdocs-material`, `mkdocstrings[python]`) in `crates/dm-control-py/pyproject.toml` (or a `requirements-docs.txt`), kept out of runtime deps
- [x] 1.2 Verify a clean `pip install '.[docs]'` (or equivalent) pulls the toolchain and that a plain install does not

## 2. PyO3 docstrings (prose source of truth)

- [x] 2.1 Add user-facing `///` docstrings to `#[pyclass]` items in `crates/dm-control-py/src/`: `Robot`, `RobotBuilder`, `Arm`, `Gripper`, `Motor`, `MotorGroup`, `MotorSpec`, `CanFrame`, `MockCanBus`, `SocketCanBus`
- [x] 2.2 Add `///` docstrings to the `#[pymethods]` of `Robot` (`from_config`, `builder`, `connect`, `enable`, `disable`, `tick`, context-manager, group/bus accessors)
- [x] 2.3 Add `///` docstrings to `Arm`/`Gripper` control methods (`mit_control`, `pos_vel_control`, `vel_control`, `pos_force_control`, `enable_all`, `disable_all`, `set_zero_all`, etc.), documenting the numpy command-array shapes
- [x] 2.4 Document the `damiao` module surface (`DamiaoCodec`, `MotorType`) and the error types (`DmError`, `TransportError`, `CodecError`, `ConfigError`, `LifecycleError`)
- [x] 2.5 `maturin develop` and confirm `help(dm_control.Robot.tick)` (and a Jupyter `?` spot-check) shows the authored prose

## 3. MkDocs site scaffold

- [x] 3.1 Add `mkdocs.yml` (mkdocs-material theme + `mkdocstrings` plugin with the python/griffe handler) at the chosen site root
- [x] 3.2 Configure the mkdocstrings python handler in import mode targeting the `dm_control` package, merging `.pyi` stubs for signatures
- [x] 3.3 Create the site pages: home/overview (from README), a Python API reference page using `::: dm_control`, and a rustdoc bridge page
- [x] 3.4 Wire the existing `docs/can-fd.md`, `docs/socketcan-setup.md`, `docs/multi-vendor.md` into the nav by reference (no content duplication)

## 4. Single local command

- [x] 4.1 Add a `make docs` target (serve) that runs `maturin develop` then `mkdocs serve`
- [x] 4.2 Add a `make docs-build` target (static) that runs `maturin develop` then `mkdocs build` into a local output dir
- [x] 4.3 Ensure the command exits non-zero with a clear message when the docs dependency group is not installed

## 5. Validation

- [x] 5.1 Run `make docs-build` on a clean checkout and confirm a browsable site is produced
- [x] 5.2 Verify every class in `dm_control.__all__` appears in the rendered API reference with rendered signatures
- [x] 5.3 Confirm a rendered method's description matches its Rust `///` source and the `help()` output
- [x] 5.4 Confirm the three guides and the rustdoc bridge are reachable from the nav
- [x] 5.5 Document the local docs command in the README (build prerequisites + the one command)
