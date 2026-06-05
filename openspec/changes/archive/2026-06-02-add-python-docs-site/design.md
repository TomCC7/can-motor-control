## Context

`can_motor_control` is a PyO3 extension: the user-visible Python classes (`Robot`, `Arm`, `Gripper`, …) are implemented in Rust under `crates/can-motor-control-py/src/` and re-exported by a thin pure-Python package at `crates/can-motor-control-py/python/can_motor_control/`. The typed surface lives in `.pyi` stubs (`__init__.pyi`, `damiao.pyi`); they carry signatures but no prose. The Rust `#[pymethods]` carry only internal notes (e.g. GIL rationale), so there are no user-facing docstrings today and `help()` returns nothing useful.

The repo also has three standalone prose guides under `docs/` and a Rust core documented with rustdoc. The goal of this change is one locally-buildable site that unifies the Python API reference with those guides. No CI, no hosting, no crates.io/PyPI publishing is in scope (the repo is private for now).

Two decisions were settled during exploration:
1. Docstring prose lives in **Rust `///`** (so `help()`/Jupyter `?` work), not in `.pyi`.
2. The site is a **unified project site** (Python API + guides + rustdoc bridge), not API-only.

## Goals / Non-Goals

**Goals:**
- One command to build the extension and bring up the docs site locally (serve + static build).
- Python API reference auto-generated from the package; prose sourced from Rust docstrings.
- `help()` / Jupyter `?` show the same prose as the rendered site.
- Existing `docs/*.md` guides absorbed into site navigation; a bridge to rustdoc.
- Docs toolchain is dev-only; the runtime wheel is unaffected.

**Non-Goals:**
- CI jobs, GitHub Pages, docs.rs, or any publishing.
- Versioned/multi-version docs.
- Rewriting the prose guides or documenting the Rust crates beyond a link to rustdoc.
- Enforcing docstring coverage with linting/gates (can come later).

## Decisions

### Decision: MkDocs Material + mkdocstrings[python] (griffe)

Use `mkdocs-material` as the site generator and `mkdocstrings[python]` (griffe backend) for the API reference.

- **Why over Sphinx**: markdown-native, so the existing `docs/*.md` guides drop in unchanged; far lower setup friction; good defaults. Sphinx would force RST/MyST and heavier config for little gain at this scale.
- **Why over pdoc**: pdoc is reference-only and can't host the guides in one navigable site; we explicitly want a unified site.

### Decision: griffe runs in import mode against the built extension

Because the classes are a compiled extension, griffe must **import** `can_motor_control` (after `maturin develop`) to read the runtime `__doc__` that PyO3 generates from the Rust `///` comments. This guarantees the rendered prose is exactly what `help()` shows — one source of truth, no drift.

- **Alternative considered**: griffe static/`.pyi`-only mode (no build). Rejected as the *prose* source because `.pyi` docstrings never reach `help()` — but `.pyi` is still used for signatures (below).

### Decision: signatures via inspection; griffe `force_inspection: true`

**As built:** the mkdocstrings python handler runs with `force_inspection: true`, so griffe imports the built extension and reads the runtime `__doc__` (the Rust `///` prose) and the signatures PyO3 auto-generates as `__text_signature__`. In practice PyO3 0.22 emits usable text signatures for the constructors and methods, so the rendered reference shows real parameter lists without scattering `#[pyo3(text_signature)]`. The `.pyi` stubs are **not** merged in by griffe; they remain the typed contract for IDEs/type-checkers, but the docs site's signatures come from inspection.

- **Fallback**: if a specific method renders a bare signature, add a targeted `#[pyo3(text_signature)]` to that method.

### Decision: consistent `__module__ = "can_motor_control"` across all exported types

griffe resolves the package's re-exports (`from can_motor_control._native import …`) by following each object's `__module__`. The `#[pyclass]` types already set `module = "can_motor_control"`, but the exceptions were created with `create_exception!(_native, …)`, stamping `__module__ = "_native"` — a bare name griffe cannot locate, which broke alias resolution for every error type. The fix is to create them with `create_exception!(can_motor_control, …)` so all exported types agree on `can_motor_control`. This is also more correct for Python: `repr(DmError)` now reads `<class 'can_motor_control.DmError'>`, matching where users import it.

### Decision: one local command, uv-managed reproducible environment

`make docs` / `make docs-build` are the single entry points. They run `uv sync --reinstall-package can_motor_control` (build the extension editable via maturin + install the locked toolchain) then `uv run mkdocs serve` / `mkdocs build`. The environment is a **uv project** rooted at the repo root: a non-package `pyproject.toml` (`[tool.uv] package = false`) that depends on `can_motor_control[docs]` via an editable path source, pinned by a committed `uv.lock`. This makes local and (future) CI builds byte-for-byte reproducible — CI is `uv sync --frozen` + `uv run mkdocs build`, no extra glue.

- **Alternatives considered**: (a) uv pinning only the Python tools while `maturin develop` stays a separate builder — cleaner Rust/Python separation but two build commands and a tool list duplicated outside `pyproject.toml`; (b) `uv pip compile` to a `requirements-docs.txt` (pip-tools style) — lightest, but no single-source-of-truth manifest. Rejected in favor of the idiomatic uv-project + lockfile.

### Decision: docs deps in the `[docs]` optional-dependency group

`mkdocs-material` and `mkdocstrings[python]` live in `[project.optional-dependencies].docs` in the crate's `pyproject.toml` — the single source of truth the root uv project pulls via `can_motor_control[docs]`. They are gated behind `Provides-Extra: docs` and never become runtime deps of the wheel (verified: runtime `Requires-Dist` is `numpy` only).

### Decision: site lives next to the package, guides referenced from `docs/`

Put `mkdocs.yml` so its `docs_dir` can include the repo's existing `docs/*.md` (via nav entries or by pointing at the repo `docs/` directory). Avoid duplicating the guide content — reference the existing files so there is a single copy.

## Risks / Trade-offs

- **Doc build requires a successful extension build** → `uv sync` sequences it; if the extension fails to build, docs fail loudly (acceptable — a broken extension has no API to document).
- **griffe alias resolution across the static `__init__.py` / compiled `_native` boundary** (the sharp edge anticipated here) → resolved by `force_inspection: true` plus consistent `__module__ = "can_motor_control"` on every exported type. Verified end-to-end: all 17 exported classes resolve with docstrings and render with signatures.
- **`uv run maturin` is unavailable** (maturin runs only inside uv's isolated build env, not as a project dep) → fine, because the Makefile never calls maturin directly; it drives the build through `uv sync`.
- **Rust-docstring edits need an explicit rebuild** (uv won't auto-detect Rust source changes for an editable install) → `make docs*` uses `uv sync --reinstall-package can_motor_control`, forcing the rebuild every run (a cargo no-op when nothing changed).
- **No coverage enforcement** → undocumented methods render empty; acceptable for now, a future change can add a `missing_docs`-style gate.
- **MkDocs 2.0 / mkdocs-material future-version notice** prints on every build → cosmetic only; not an error.
