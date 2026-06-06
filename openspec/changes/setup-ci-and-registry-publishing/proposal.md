## Why

The project already has a mixed Rust/Python release surface, but registry publication is still documented as future work and the current CI only validates builds and smoke tests. Before v0.1.0 is released, contributors need a repeatable CI and release plan that proves the workspace is publishable, preserves the crate dependency order, and gives maintainers clear PyPI and crates.io publication guidance.

## What Changes

- Add release automation requirements for pull-request CI, release-candidate validation, and tag-driven publishing.
- Extend the existing GitHub Actions approach from build-only CI into registry-ready workflows for Rust crates and Python wheels.
- Document maintainer guidance for publishing `can-motor-codec`, `can-motor-damiao-codec`, and `can-motor-control` to crates.io in dependency order.
- Document maintainer guidance for publishing the `can-motor-control` Python package built from `crates/can-motor-control-py` to PyPI.
- Add dry-run and safety gates so release artifacts are validated before any registry upload.

## Capabilities

### New Capabilities
- `release-automation`: CI and registry-publishing behavior for Rust crates, Python wheels, and maintainer-facing release guidance.

### Modified Capabilities

None.

## Impact

- `.github/workflows/ci.yml` and new release workflow files under `.github/workflows/`.
- Rust workspace metadata in `Cargo.toml` and package metadata in each crate `Cargo.toml` if required for registry publication.
- Python package metadata in `crates/can-motor-control-py/pyproject.toml`.
- Maintainer documentation in `README.md`, `CHANGELOG.md`, or a dedicated release guide.
- GitHub repository settings for PyPI Trusted Publishing/OIDC and crates.io credential handling.
