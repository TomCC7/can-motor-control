## Why

The `dm_control` Python package exposes a full control API (`Robot`, `Arm`, `Gripper`, `RobotBuilder`, codecs, errors) but ships **zero user-facing docstrings** — the `#[pymethods]` carry only internal implementation notes and the `.pyi` stubs carry signatures with no prose. Python users at a REPL/Jupyter get nothing from `help()`, and there is no rendered reference site. The repo also has standalone prose guides under `docs/` (SocketCAN setup, CAN-FD, multi-vendor) that live disconnected from any navigable site. We want one locally-buildable documentation site that unifies the Python API reference with these guides.

## What Changes

- Author user-facing docstrings as Rust `///` comments on the PyO3 `#[pyclass]` / `#[pymethods]` items so they bake into the compiled module's `__doc__` — making `help()` and Jupyter `?` work as a side effect.
- Add a MkDocs site (`mkdocs-material` + `mkdocstrings[python]` / griffe) that renders the Python API reference from the built extension, merging `.pyi` signatures with the runtime docstrings.
- Wire the existing `docs/*.md` guides into the site navigation, add an overview/home page, and add a bridge page pointing to the Rust rustdoc.
- Provide a single local command (e.g. a `make`/shell target) that builds the extension with `maturin develop` and serves/builds the site — **no CI or publishing in scope**.
- Add the docs toolchain as a dev/optional dependency group (not a runtime dependency of the wheel).

## Capabilities

### New Capabilities
- `python-docs-site`: A locally-buildable documentation site for the `dm_control` Python package — API reference auto-generated from PyO3 docstrings plus the existing prose guides, driven by one command.

### Modified Capabilities
<!-- None — no existing spec-level behavior changes. -->

## Impact

- **Source**: `crates/dm-control-py/src/*.rs` — add `///` docstrings (and `#[pyo3(text_signature)]` where griffe needs signatures not covered by `.pyi`).
- **New files**: `mkdocs.yml`, a `docs/` site structure (or `crates/dm-control-py/docs/`), API reference stub pages, a local build command/target.
- **Dependencies**: dev-only `mkdocs-material`, `mkdocstrings[python]` (griffe); requires `maturin` (already a build dep) for the doc build step.
- **Existing content**: `docs/can-fd.md`, `docs/socketcan-setup.md`, `docs/multi-vendor.md` are absorbed into the site nav (moved or referenced, not rewritten).
- **No runtime impact**: docs deps are excluded from the published wheel; no API behavior changes.
