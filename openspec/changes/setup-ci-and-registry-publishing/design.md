## Context

The workspace publishes three Rust library crates (`motor-codec`, `damiao-codec`, `can-motor-control`) and one Python package (`can-motor-control`) built from the `can-motor-control-py` PyO3/maturin crate. The current CI already runs Rust formatting, Clippy, workspace tests, no_std builds, vendor-isolation checks, a Linux maturin wheel build, and Python smoke tests. The README and changelog describe future installation commands and the crates.io dependency order, but there is no release workflow or maintainer guide for actually publishing v0.1.0.

Key constraints:
- The Rust workspace uses a shared `workspace.package.version = "0.1.0"` and Rust MSRV 1.85.
- The Python package metadata lives in `crates/can-motor-control-py/pyproject.toml`, uses maturin, and declares `requires-python = ">=3.10"` with PyO3 `abi3-py310`.
- `can-motor-control-py` is a binding crate for wheels and must not be published to crates.io.
- Linux SocketCAN is the v1 runtime target; cross-platform CI should distinguish codec/library checks from hardware behavior.
- crates.io package-name availability must be checked directly instead of inferred from the GitHub repository name.

## Goals / Non-Goals

**Goals:**
- Keep fast PR CI for code quality and installability.
- Add release-candidate validation that performs dry-run packaging for every registry artifact.
- Add protected tag/manual release workflows for PyPI and crates.io publishing.
- Prefer short-lived OIDC/Trusted Publishing credentials where the registries support them.
- Document the exact maintainer sequence for version checks, registry setup, dry-runs, publishing, and post-publish smoke tests.

**Non-Goals:**
- Do not change public Rust or Python APIs.
- Do not add hardware-in-the-loop release gates; mock and packaging tests remain the automated boundary.
- Do not publish `can-motor-control-py` to crates.io.
- Do not solve multi-platform Python wheel expansion beyond the first Linux-focused release unless a later change chooses that scope.

## Decisions

### Decision 1 — Split CI, release-candidate validation, and publish workflows

Keep `.github/workflows/ci.yml` focused on pull requests and `main`, then add separate release-oriented workflows:
- a manually runnable release-candidate workflow that runs all packaging checks with no uploads;
- a protected release workflow for tags such as `v0.1.0` or manual dispatch after maintainer review.

This avoids granting registry publish permissions to routine CI. It also lets maintainers exercise packaging dry-runs before creating or pushing a final release tag.

Alternative considered: add publish steps to the existing CI workflow. Rejected because registry credentials and OIDC permissions should be scoped to a small number of release jobs.

### Decision 2 — Extend PR CI with docs and lock-aware packaging checks

The existing CI should remain the base quality gate, but it should add docs rendering with the existing `uv sync --frozen` + `uv run mkdocs build` path described in the README. Wheel CI should use locked/reproducible commands where possible and install the built artifact into a clean environment before import and pytest checks.

Alternative considered: move all wheel validation into release-only workflows. Rejected because a broken Python binding should block pull requests before release time.

### Decision 3 — Use maturin for building and PyPI Trusted Publishing for upload

Build Python distributions from `crates/can-motor-control-py` with maturin, including a wheel and sdist. Use PyPI-compatible checks (`--compatibility pypi` where supported by the chosen maturin invocation) and upload artifacts only from a dedicated publish job using `pypa/gh-action-pypi-publish` with `id-token: write` and a protected GitHub environment such as `pypi` or `release`.

This follows PyPI's Trusted Publishing guidance: the publish job retrieves already-built distributions and uploads without a long-lived PyPI token. The PyPI project must be configured with the repository owner, repository name, workflow filename, and optional environment before the job can publish.

Alternative considered: `maturin upload` or `uv publish` with a stored PyPI API token. Rejected for the default path because Trusted Publishing removes long-lived token management and produces registry-backed attestations where supported.

### Decision 4 — Publish Rust crates in explicit dependency order

The Rust release flow should run `cargo publish --dry-run -p motor-codec`, then `damiao-codec`, then `can-motor-control`, and publish in the same order once upstream workspace crates already exist on crates.io. For the first crates.io release, downstream dry-runs cannot fully resolve until upstream crates are public, so release-candidate validation should dry-run the first crate and explicitly skip downstream registry checks until maintainers rerun dry-runs after each upstream crate is published. The binding crate `can-motor-control-py` remains excluded from crates.io publication.

This matches the current changelog and the workspace dependency graph. Dry-runs catch packaging metadata errors before upload; the actual upload order ensures downstream crates can resolve already-published dependency versions.

Alternative considered: `cargo publish --workspace`. Rejected because the workspace includes a Python binding crate that is not intended for crates.io, and explicit order is clearer for first releases.

### Decision 5 — Use crates.io Trusted Publishing after first publication, with a documented first-release fallback

crates.io Trusted Publishing should be the preferred CI path for crates that already exist on crates.io, using a protected release environment, `id-token: write`, and the official crates.io OIDC authentication action when configured. However, crates.io requires a crate to already be published before Trusted Publishing can be configured, so the v0.1.0 initial publication may need a maintainer-held API token or manual local `cargo publish` after dry-runs.

The guide must make this distinction explicit for each crate. After the first successful publication of `motor-codec`, `damiao-codec`, and `can-motor-control`, maintainers should configure crates.io Trusted Publishing for the release workflow and remove any long-lived CI token path.

Alternative considered: require long-lived `CARGO_REGISTRY_TOKEN` in GitHub secrets indefinitely. Rejected because Trusted Publishing is now available on crates.io and reduces token exposure for subsequent releases.

### Decision 6 — Add a dedicated release guide

Create a maintainer release guide, likely `docs/release.md`, and link it from the README. The guide should cover:
- package-name checks on PyPI and crates.io;
- registry account/project setup;
- GitHub environment setup for protected release jobs;
- version synchronization across workspace metadata and Python package `__version__`;
- dry-run commands;
- crates.io publish order;
- PyPI upload flow;
- post-publish smoke tests for `pip install can-motor-control` and Cargo dependencies.

Alternative considered: keep release commands only in `CHANGELOG.md`. Rejected because the changelog is release history, not an operational runbook.

## Risks / Trade-offs

- [Risk] Initial crates.io publication cannot use Trusted Publishing until each crate exists on crates.io → Mitigation: document the one-time manual/token-based publish and immediately configure Trusted Publishing afterward.
- [Risk] PyPI Trusted Publishing fails if the PyPI project is configured with the wrong workflow filename or environment → Mitigation: document the exact GitHub owner/repo/workflow/environment values and use a protected environment consistently.
- [Risk] The first release may accidentally include large or unintended files in `.crate` or wheel artifacts → Mitigation: require `cargo package`/`cargo publish --dry-run`, inspect `target/package`, and install built wheels in clean environments.
- [Risk] Multi-platform wheel expectations may exceed the Linux-only v1 runtime guarantee → Mitigation: document Linux as the automated release target and leave broader wheel matrices for a later change.
- [Risk] Registry name collisions can block the planned package names → Mitigation: check PyPI and crates.io directly during release preparation and resolve naming before implementation or publication.

## Migration Plan

1. Add or update CI jobs without registry permissions.
2. Add release-candidate workflow and confirm it only uploads workflow artifacts.
3. Add release workflow with protected jobs and registry publishing disabled until maintainers configure environments/registries.
4. Add the release guide and link it from the README.
5. For v0.1.0, perform dry-runs, publish crates in dependency order, publish the Python package, then verify public installs.
6. After initial crates.io publication, configure crates.io Trusted Publishing for future releases.

Rollback is operational: disable or delete the release workflow, remove the registry trusted-publisher entries, and continue using PR CI while fixing the release plan. Already-published registry versions cannot be overwritten, so failed releases require a new patch version.

## Open Questions

- Which GitHub environment name should maintainers standardize on for release publishing: `release`, `pypi`, or separate `pypi`/`crates-io` environments?
- Should v0.1.0 ship only Linux x86_64 wheels, or should the release workflow immediately include broader manylinux architectures despite the Linux SocketCAN runtime constraint?
- Are the planned package names already owned/available on PyPI and crates.io, and who will own them?
