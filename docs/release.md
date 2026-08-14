# Release guide

This guide is the maintainer runbook for publishing `can-motor-control` to
PyPI and the Rust crates to crates.io. It assumes the release is cut from
`main` after CI is green.

## Release surfaces

| Registry | Package | Source | Notes |
| --- | --- | --- | --- |
| PyPI | `can-motor-control` | `crates/can-motor-control-py` | Built by maturin as the import package `can_motor_control` |
| crates.io | `can-motor-codec` | `crates/motor-codec` | Publish first; no_std shared trait crate, imported as `motor_codec` |
| crates.io | `can-motor-damiao-codec` | `crates/damiao-codec` | Publish after `can-motor-codec`, imported as `damiao_codec` |
| crates.io | `can-motor-control` | `crates/can-motor-control` | Publish after `can-motor-damiao-codec` |

Do not publish `can-motor-control-py` to crates.io. It exists to build the
Python wheel.

## Maintainer setup

Before the first release:

1. Verify package-name ownership/availability directly on PyPI and crates.io:
   - `https://pypi.org/project/can-motor-control/`
   - `https://crates.io/crates/can-motor-codec`
   - `https://crates.io/crates/can-motor-damiao-codec`
   - `https://crates.io/crates/can-motor-control`
2. Configure a protected GitHub environment named `pypi` for the PyPI publish
   job.
3. Configure a protected GitHub environment named `crates-io` for the Rust
   publish job.
4. Configure GitHub Pages with source set to GitHub Actions. Documentation is
   deployed by `.github/workflows/pages.yml` to the `github-pages` environment
   after pushes to `main` and manual dispatches.
5. On PyPI, configure Trusted Publishing for the repository, workflow file
   `.github/workflows/release.yml`, and environment `pypi`.
6. On crates.io, configure Trusted Publishing for each crate after that crate
   has been published once. crates.io requires the crate to exist before a
   trusted publisher can be added.

For the first crates.io publication of a new crate, use a maintainer-owned
crates.io API token or a local `cargo publish` after the release-candidate
workflow has passed. Once the first version exists, configure crates.io Trusted
Publishing and use the release workflow for subsequent versions.

## Version and metadata audit

Before every release, confirm these values all describe the same version:

- `Cargo.toml` `workspace.package.version`
- workspace dependency versions for `can-motor-codec`, `can-motor-damiao-codec`, and
  `can-motor-control`
- `crates/can-motor-control-py/python/can_motor_control/__init__.py`
  `__version__`
- `CHANGELOG.md` release heading

Rust crate metadata readiness:

- Each published crate has `name`, `description`, inherited `license`, inherited
  `repository`, and inherited `rust-version` metadata.
- The top-level README is the canonical user-facing README for release notes and
  install commands.
- The release-candidate workflow runs `cargo publish --dry-run` for the crates
  in dependency order and uploads the generated `.crate` files for inspection.

Python package metadata readiness:

- The publishable Python metadata lives in
  `crates/can-motor-control-py/pyproject.toml`, not the root dev-only
  `pyproject.toml`.
- The project name is `can-motor-control`; the import package is
  `can_motor_control`; the native extension is `can_motor_control._native`.
- The package declares Python `>=3.10`, Linux and macOS classifiers, dual-license
  classifiers, and runtime dependency `numpy>=1.24`.
- The release-candidate workflow builds Linux and macOS arm64 wheels plus one
  sdist, then installs each wheel into a clean environment before running tests.

## Release-candidate validation

Run the manual release-candidate workflow before publishing:

```bash
gh workflow run release-candidate.yml --ref main
```

For the first crates.io publication, leave `initial_crates_io_release` enabled.
That mode dry-runs `can-motor-codec` and skips downstream crates, because
`can-motor-damiao-codec` cannot complete a crates.io dry-run until `can-motor-codec` exists
on crates.io, and `can-motor-control` cannot complete one until its upstream
crates exist. After publishing `can-motor-codec`, rerun the workflow with
`initial_crates_io_release=false` before publishing `can-motor-damiao-codec`; repeat the
staged check before publishing `can-motor-control`.

The workflow does not receive registry credentials. It runs:

- Rust formatting, Clippy, workspace tests, no_std codec builds, and vendor
  isolation checks.
- `cargo publish --dry-run -p can-motor-codec`
- `cargo publish --dry-run -p can-motor-damiao-codec` when upstream crates are already on crates.io
- `cargo publish --dry-run -p can-motor-control` when upstream crates are already on crates.io
- staged downstream validation during the initial release, after upstream crates are public
- Linux and macOS arm64 `maturin` wheel builds, plus one Linux-built sdist
- clean wheel installation, platform-export checks, and `pytest tests/python -v`
- docs build through `scripts/build-docs.sh --frozen`, including MkDocs and
  hosted rustdoc under `site/rustdoc/`

Inspect the uploaded `crate-packages`, `python-distributions-linux`, and
`python-distributions-macos-arm64` artifacts
before publishing. During the very first crates.io run, `crate-packages` only
contains crates whose dry-runs can resolve against the current crates.io index.

The workflows remove `target/wheels` before each maturin build and upload only
`can_motor_control-*` files. Keep that cleanup in place so stale local wheels or
old package names cannot be installed or published by a broad wildcard.
They use current artifact actions (`actions/upload-artifact@v7` and
`actions/download-artifact@v5`); keep matrix artifact names unique if wheel
builds expand beyond the current Linux target.

## Publishing

Create a version tag after the release-candidate workflow is green:

```bash
git tag v0.1.0
git push origin v0.1.0
```

The release workflow publishes from protected jobs:

- PyPI uses Trusted Publishing/OIDC through `pypa/gh-action-pypi-publish`; no
  long-lived PyPI token should be stored in repository secrets.
- crates.io uses Trusted Publishing/OIDC through
  `rust-lang/crates-io-auth-action` after each crate has a trusted publisher
  configured. Run that job by manually dispatching `release.yml` with
  `publish_crates=true`; it is intentionally not automatic on the first tag
  push because new crates cannot use Trusted Publishing until after their first
  publication.

For the initial crates.io publication, publish in this order if Trusted
Publishing cannot be configured yet:

```bash
cargo publish -p can-motor-codec
cargo publish --dry-run -p can-motor-damiao-codec
cargo publish -p can-motor-damiao-codec
cargo publish --dry-run -p can-motor-control
cargo publish -p can-motor-control
```

## Post-publish checks

After registry publication, verify clean installs.

Python:

```bash
python -m venv /tmp/can-motor-control-pypi
/tmp/can-motor-control-pypi/bin/python -m pip install --upgrade pip
/tmp/can-motor-control-pypi/bin/pip install can-motor-control==0.1.0
/tmp/can-motor-control-pypi/bin/python -c "import can_motor_control; print(can_motor_control.__version__)"
```

Rust:

```bash
tmpdir=$(mktemp -d)
cd "$tmpdir"
cargo init --lib --name can_motor_control_release_check
cargo add can-motor-control@0.1
cargo add can-motor-damiao-codec@0.1 --rename damiao-codec
cargo check
```

If a registry publish fails after one artifact is already public, do not reuse
the same version. Fix the issue, bump to the next patch version, and repeat the
release-candidate flow.
