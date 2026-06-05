## 1. Registry and naming decisions

- [x] 1.1 Re-run availability checks for `can-motor-control`, `can_motor_control`, `can.motor.control`, `damiao-codec`, `dm-codec`, and any selected companion crate names on crates.io, PyPI, docs.rs, and the target GitHub namespace.
- [x] 1.2 Record the final crate-family decision: keep `motor-codec`; rename `dm-codec` to `damiao-codec`.
- [x] 1.3 Update release notes/changelog publish order to use the selected crate names and the `can-motor-control` Python distribution.

## 2. Rust workspace and crate rename

- [x] 2.1 Rename workspace members and directories: `crates/dm-control` to `crates/can-motor-control`, `crates/dm-control-py` to `crates/can-motor-control-py`, and `crates/dm-codec` to `crates/damiao-codec`.
- [x] 2.2 Update root `Cargo.toml` members, workspace dependencies, package names, repository URL, descriptions, and path dependencies for the new crate names.
- [x] 2.3 Update package manifests under renamed crates, including `[package].name`, `[lib].name` where needed, descriptions, dev-dependencies, and feature dependency references.
- [x] 2.4 Replace Rust imports and paths from `dm_control::...` to `can_motor_control::...` and from `dm_codec::...` to `damiao_codec::...`.
- [x] 2.5 Update Rust examples, tests, rustdoc comments, config comments, and error/doc strings that instruct users to run `cargo run -p dm-control` or depend on `dm-control`.
- [x] 2.6 Update vendor-isolation checks to inspect `cargo tree -p can-motor-control --edges normal` and reject selected vendor codec package names.

## 3. Python package and PyO3 rename

- [x] 3.1 Update `crates/can-motor-control-py/pyproject.toml`: distribution name `can-motor-control`, `module-name = "can_motor_control._native"`, docs extra wording, and maturin settings.
- [x] 3.2 Rename the pure Python package directory from `python/dm_control` to `python/can_motor_control`, including `__init__.py`, `__init__.pyi`, `damiao.py`, and `damiao.pyi` imports/docstrings.
- [x] 3.3 Update PyO3 `#[pyclass(module = ...)]` metadata from `dm_control` / `dm_control.damiao` to `can_motor_control` / `can_motor_control.damiao`.
- [x] 3.4 Update `create_exception!` module arguments, Python exception docstrings, and any `__module__`-sensitive code so exported exceptions report `can_motor_control`.
- [x] 3.5 Update PyO3 crate references from `dm_control::Error` / `dm_control::TransportError` to `can_motor_control::...` after the Rust crate rename.
- [x] 3.6 Update logger namespace from `dm_control` to `can_motor_control`.
- [x] 3.7 Remove or regenerate stale built native extension artifacts such as `python/dm_control/_native.abi3.so` so imports cannot pass against old build outputs.

## 4. Python examples, tests, and uv environment

- [x] 4.1 Replace `import dm_control` and `from dm_control.damiao ...` in all Python examples and tests with `can_motor_control` imports.
- [x] 4.2 Update example prose, usage text, safety banners, and developer snippets that mention `dm_control`.
- [x] 4.3 Update root `pyproject.toml` dev/docs environment: package name, description, dependency `can-motor-control[docs]`, and `[tool.uv.sources]` key/path.
- [x] 4.4 Regenerate `uv.lock` after the Python distribution rename and verify it contains the normalized `can-motor-control` package identity.

## 5. Documentation and release text

- [x] 5.1 Update README title, description, install snippets, build-from-source paths, layout table, docs commands, rustdoc commands, repository URL, and publish-facing package names.
- [x] 5.2 Update `mkdocs.yml`: `site_name`, `site_description`, `repo_url`, mkdocstrings `paths`, preload module `can_motor_control._native`, and comments.
- [x] 5.3 Update docs pages (`docs/index.md`, `docs/reference.md`, `docs/rust.md`, `docs/can-fd.md`, `docs/socketcan-setup.md`, `docs/multi-vendor.md`) for the new public identity and import snippets.
- [x] 5.4 Update CHANGELOG crate names, Python package names, example paths, and publish order.
- [x] 5.5 Update config comments in `configs/*.toml` so copy-paste Rust/Python snippets use the renamed crates and package.

## 6. CI and build tooling

- [x] 6.1 Update `.github/workflows/ci.yml` job names, package selectors, no_std build package names, vendor-isolation grep, Python wheel working directory, artifact name, and import smoke to `can_motor_control`.
- [x] 6.2 Update `Makefile` docs-sync command to rebuild `can-motor-control` / `can_motor_control` instead of `dm_control`.
- [x] 6.3 Update any command snippets or scripts that reference `crates/dm-control-py`, `dm-control`, `dm-codec`, or `dm_control`.

## 7. OpenSpec and stale-reference cleanup

- [x] 7.1 Update live spec `openspec/specs/python-docs-site/spec.md` to match the renamed docs package behavior from this change.
- [x] 7.2 Update active changes (`add-canfd-support`, `add-state-refresh`) where they describe current release-facing package names, paths, docs commands, or examples.
- [x] 7.3 Decide archived OpenSpec treatment: archived snippets were updated for search cleanliness where they referenced current release instructions.
- [x] 7.4 Run targeted stale-name searches for `dm_control_rs`, `dm-control`, `dm_control`, `dm-control-py`, `dm-codec`, and `dm_codec`; classify remaining matches as fixed, historical, or intentionally retained.

## 8. Verification

- [x] 8.1 Run `cargo metadata --format-version 1` and confirm package graph uses the renamed package identities.
- [x] 8.2 Run `cargo fmt --all --check`.
- [x] 8.3 Run `cargo clippy --workspace --all-targets -- -D warnings`.
- [x] 8.4 Run `cargo test --workspace`.
- [x] 8.5 Run no_std builds for `motor-codec` and the selected Damiao codec crate on `thumbv7em-none-eabihf`.
- [x] 8.6 Run `maturin build --release --strip` from the renamed Python binding crate and confirm the wheel filename uses `can_motor_control-...whl`.
- [x] 8.7 Install the wheel into a clean venv with only numpy and run `python -c "import can_motor_control; print(can_motor_control.__name__)"`.
- [x] 8.8 Run a Python inspection smoke that imports `can_motor_control.damiao`, checks `__all__`, and confirms public classes/exceptions report `__module__` under `can_motor_control`.
- [x] 8.9 Run `pytest tests/python -v`.
- [x] 8.10 Run `uv sync --reinstall-package can-motor-control` (or the normalized package key required by uv) and `make docs-build`.
- [ ] 8.11 Run `cargo publish --dry-run` for publishable crates in dependency order using the selected package names. Blocked for downstream crates until `motor-codec` exists on crates.io; `motor-codec` dry-run passes.
- [x] 8.12 Repeat registry/name checks immediately before publishing and include the results in the implementation summary.
