"""Shared pytest fixtures."""

import pytest

# Allow tests to skip cleanly when the native extension isn't built yet
# (during `cargo test` from a fresh checkout).
try:
    import can_motor_control  # noqa: F401
except ImportError:
    pytest.skip(
        "can_motor_control native module not built; run `maturin develop` first",
        allow_module_level=True,
    )
