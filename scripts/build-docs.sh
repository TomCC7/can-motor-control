#!/usr/bin/env bash
set -euo pipefail

sync_args=()
if [[ "${1:-}" == "--frozen" ]]; then
  sync_args+=(--frozen)
  shift
fi

if [[ $# -ne 0 ]]; then
  echo "usage: $0 [--frozen]" >&2
  exit 64
fi

site_dir="${SITE_DIR:-site}"

uv sync "${sync_args[@]}" --reinstall-package can-motor-control
uv run --with mkdocs --with mkdocs-material --with 'mkdocstrings[python]' -- python -m mkdocs build --site-dir "${site_dir}"

cargo doc --no-deps --workspace --locked
rm -rf "${site_dir}/rustdoc"
mkdir -p "${site_dir}/rustdoc"
cp -R target/doc/. "${site_dir}/rustdoc/"
