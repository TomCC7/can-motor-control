.PHONY: docs docs-build docs-serve docs-sync

# Host/port for the docs servers. Override on the command line, e.g.
#   make docs ADDR=0.0.0.0:8000     # reachable from other devices on the LAN
ADDR ?= 127.0.0.1:8000

# Provision (or refresh) the pinned docs environment from uv.lock. This builds
# the `can_motor_control` extension editable via maturin and installs the locked docs
# toolchain. `--reinstall-package` forces a rebuild so edits to the Rust
# docstrings are always reflected in the rendered site.
docs-sync:
	@command -v uv >/dev/null 2>&1 || { \
	  echo "error: 'uv' not found."; \
	  echo "  install it:  https://docs.astral.sh/uv/getting-started/installation/"; \
	  exit 1; }
	uv sync --reinstall-package can-motor-control

# Serve the docs site with live reload (rebuilds on save). Best for editing.
docs: docs-sync
	uv run --with mkdocs --with mkdocs-material --with 'mkdocstrings[python]' -- python -m mkdocs serve --dev-addr $(ADDR)

# Render a static site into ./site, including hosted rustdoc under ./site/rustdoc
docs-build:
	bash scripts/build-docs.sh

# Host the already-built ./site locally with a plain static server. Builds
# first if needed. Nothing is pushed anywhere.
docs-serve: docs-build
	@echo "Serving ./site at http://$(ADDR)  (Ctrl-C to stop)"
	uv run python -m http.server --directory site $(word 2,$(subst :, ,$(ADDR))) --bind $(word 1,$(subst :, ,$(ADDR)))
