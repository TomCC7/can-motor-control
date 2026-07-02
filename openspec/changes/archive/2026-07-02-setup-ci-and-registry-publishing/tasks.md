## 1. CI Quality Gates

- [x] 1.1 Update `.github/workflows/ci.yml` to add a docs build job using `uv sync --frozen` and `uv run mkdocs build`
- [x] 1.2 Update `.github/workflows/ci.yml` wheel steps to use release-like maturin arguments, including locked dependencies and PyPI-compatible packaging checks where supported
- [x] 1.3 Update `.github/workflows/ci.yml` Python validation so built wheels are installed in a clean environment before import checks and pytest execution
- [x] 1.4 Run the CI commands locally where possible: `cargo fmt --all --check`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo test --workspace`, no_std codec builds, docs build, wheel build, and pytest

## 2. Release Candidate Workflow

- [x] 2.1 Add a manual `.github/workflows/release-candidate.yml` workflow that runs all CI quality gates without registry credentials
- [x] 2.2 Add Rust packaging dry-runs/staged first-release checks to the release-candidate workflow for `can-motor-codec`, `can-motor-damiao-codec`, and `can-motor-control` in dependency order
- [x] 2.3 Add Python wheel and sdist builds to the release-candidate workflow from `crates/can-motor-control-py`
- [x] 2.4 Upload release-candidate `.crate`, wheel, and sdist outputs only as GitHub workflow artifacts

## 3. Registry Release Workflow

- [x] 3.1 Add a protected release workflow triggered by version tags and manual dispatch
- [x] 3.2 Add a PyPI publish job that downloads prebuilt Python distributions and publishes with `pypa/gh-action-pypi-publish` using Trusted Publishing/OIDC
- [x] 3.3 Add a crates.io publish job that publishes `can-motor-codec`, `can-motor-damiao-codec`, and `can-motor-control` in dependency order and excludes `can-motor-control-py`
- [x] 3.4 Configure the release workflow with minimal job permissions and a GitHub environment for maintainer approval
- [x] 3.5 Document the first-release crates.io fallback for crates that cannot use Trusted Publishing until after initial publication

## 4. Package Metadata and Release Guide

- [x] 4.1 Audit Rust crate metadata for crates.io readiness, including descriptions, license, repository, README/include behavior, and package name availability
- [x] 4.2 Audit Python package metadata for PyPI readiness, including project URLs, classifiers, README metadata, wheel contents, and package name availability
- [x] 4.3 Add a maintainer release guide covering registry setup, GitHub environments, version synchronization, dry-runs, publish order, and post-publish checks
- [x] 4.4 Link the release guide from `README.md` and keep the changelog publish-order note consistent with the guide

## 5. Verification

- [x] 5.1 Run OpenSpec validation/status for this change and confirm all release-automation requirements are represented
- [ ] 5.2 Run the release-candidate workflow through `workflow_dispatch` before enabling any publish path
- [ ] 5.3 Verify post-publish installation commands in clean environments after the first release: `pip install can-motor-control` and Cargo dependencies for the published crate versions
