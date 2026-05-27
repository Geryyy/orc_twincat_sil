"""H-5: ensure LinearAxis/Iiwa/Kinova constructors share the server_port kwarg.

These tests are source-level (no import of the built extension required) plus
a runtime parity test gated by importorskip.
"""

from __future__ import annotations

import inspect
import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
ROBOTS_DIR = REPO / "python" / "robots"


def _read(name: str) -> str:
    return (ROBOTS_DIR / name).read_text(encoding="utf-8")


@pytest.mark.parametrize("fname", ["Iiwa.py", "Kinova.py", "LinearAxis.py"])
def test_ctor_uses_server_port_kwarg(fname: str) -> None:
    src = _read(fname)
    # All three robots must expose a server_port parameter.
    assert re.search(r"server_port\s*[:=]", src), f"{fname}: missing server_port kwarg"


def test_linear_axis_deprecated_robot_port_alias() -> None:
    src = _read("LinearAxis.py")
    assert "robot_port" in src and "DeprecationWarning" in src, (
        "LinearAxis should still accept robot_port with DeprecationWarning"
    )


def test_runtime_ctor_parity() -> None:
    """If the extension is built, constructing all three robots should use
    the same keyword-argument surface. We don't open sockets here; we just
    inspect the signatures."""
    orcpy = pytest.importorskip("orcpy")
    from orcpy.robots import Iiwa, Kinova, LinearAxis

    for cls in (Iiwa, Kinova, LinearAxis):
        sig = inspect.signature(cls.__init__)
        assert "server_port" in sig.parameters, f"{cls.__name__}.__init__ missing server_port"
        assert "client_port" in sig.parameters, f"{cls.__name__}.__init__ missing client_port"
