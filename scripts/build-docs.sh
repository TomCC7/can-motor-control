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
cat > "${site_dir}/rustdoc/index.html" <<'HTML'
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Rust API - can-motor-control</title>
    <style>
      body { font-family: system-ui, sans-serif; line-height: 1.5; margin: 2rem; }
      code { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
    </style>
  </head>
  <body>
    <h1>Rust API</h1>
    <p>Rustdoc for the crates in the <code>can-motor-control</code> workspace.</p>
    <ul>
      <li><a href="can_motor_control/"><code>can_motor_control</code></a></li>
      <li><a href="damiao_codec/"><code>damiao_codec</code></a></li>
      <li><a href="motor_codec/"><code>motor_codec</code></a></li>
    </ul>
    <p><a href="../rust/">Back to the Rust API guide</a></p>
  </body>
</html>
HTML
