"""H-6: Python-facing attributes should not carry trailing underscores.

Source-level checks run everywhere; runtime checks are gated by the built
extension via importorskip.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
BIND_DIR = REPO / "python" / "core" / "src"


def _read(p: Path) -> str:
    return p.read_text(encoding="utf-8")


def test_robotstate_time_bound_without_underscore() -> None:
    src = _read(BIND_DIR / "RobotState_bindings.h")
    assert '.def_rw("time_"' not in src
    assert re.search(r'\.def_rw\(\s*"time"\s*,\s*&RobotState::time_', src)


def test_robot_serializer_bound_without_underscore() -> None:
    src = _read(BIND_DIR / "Robot_bindings.h")
    assert '.def_rw("serializer_"' not in src
    assert re.search(r'\.def_rw\(\s*"serializer"\s*,\s*&RobotX::serializer_', src)


def test_no_trailing_underscore_python_names_in_bindings() -> None:
    """Sweep: no def_*("name_", ...) patterns in any binding header/cpp."""
    pattern = re.compile(r'\.def[_a-z]*\(\s*"([A-Za-z][A-Za-z0-9]*_)"')
    offenders: list[str] = []
    for path in BIND_DIR.rglob("*"):
        if path.suffix not in {".h", ".cpp"}:
            continue
        for m in pattern.finditer(_read(path)):
            offenders.append(f"{path.name}: {m.group(1)}")
    assert not offenders, f"Trailing-underscore Python names: {offenders}"


def test_runtime_robotstate_time_roundtrip() -> None:
    orcpy = pytest.importorskip("orcpy")
    # Try any robot module that exposes RobotState.
    robots = orcpy.core.robots
    for name in ("robot7", "robot2"):
        mod = getattr(robots, name, None)
        if mod is None:
            continue
        RobotState = getattr(mod, "RobotState", None)
        if RobotState is None:
            continue
        state = RobotState()
        # time accessor must exist and round-trip.
        t0 = orcpy.core.Time(0.125)
        state.time = t0
        assert state.time.to_sec() == pytest.approx(0.125)
        return
    pytest.skip("No RobotState submodule found at runtime.")
