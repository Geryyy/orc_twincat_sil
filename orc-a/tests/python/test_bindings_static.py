"""Static verification tests for python bindings fixes.

These tests do not require building the extension module; they inspect
source files to verify that bindings match expected patterns. Dynamic
import-based tests follow after, guarded by importorskip.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
BIND_DIR = REPO / "python" / "core" / "src"


def _read(p: Path) -> str:
    return p.read_text(encoding="utf-8")


# ---- H-12 -------------------------------------------------------------
def test_singular_perturbation_B_binding_name_matches_member():
    src = _read(BIND_DIR / "ControllerParameter_bindings.h")
    # The stray .def_rw("D", ...::B, ...) must be gone.
    assert '.def_rw("D", &SingularPerturbationParameter::B' not in src
    # And a proper "B" -> B binding should exist.
    assert re.search(r'\.def_rw\(\s*"B"\s*,\s*&SingularPerturbationParameter::B', src)


# ---- H-13 -------------------------------------------------------------
@pytest.mark.parametrize(
    "fname",
    ["Iiwa_bindings.cpp", "Kinova_bindings.cpp", "LinearAxis_bindings.cpp"],
)
def test_trajectory_server_methods_release_gil(fname: str):
    src = _read(BIND_DIR / fname)
    # Every run/poll/send_robot_data binding must be followed (within a
    # small window) by a gil_scoped_release call_guard.
    for method in ("run", "poll", "send_robot_data"):
        # Find each occurrence of def("method",
        for m in re.finditer(rf'\.def\(\s*"{method}"', src):
            window = src[m.start() : m.start() + 400]
            assert "gil_scoped_release" in window, (
                f"{fname}: binding for {method} missing call_guard"
            )


# ---- M-16 -------------------------------------------------------------
def test_robot_py_has_no_duplicate_method_defs():
    src = (REPO / "python" / "robots" / "Robot.py").read_text()
    for name in (
        "send_cartesian_velocity_trajectory",
        "send_jointspace_velocity_trajectory",
        "send_joint_ctr_parameter_trajectory",
    ):
        count = len(re.findall(rf"^\s*def\s+{name}\s*\(", src, re.MULTILINE))
        assert count == 1, f"{name} defined {count} times"


# ---- M-17 -------------------------------------------------------------
def test_kinova_bindings_include_eigen_dense():
    src = _read(BIND_DIR / "Kinova_bindings.cpp")
    assert "<nanobind/eigen/dense.h>" in src


# ---- M-18 -------------------------------------------------------------
def test_iiwa_bindings_no_dead_commented_update_override():
    src = _read(BIND_DIR / "Iiwa_bindings.cpp")
    assert "nb::overload_cast<Time, bool>(&Iiwa::update)" not in src


# ---- M-19 -------------------------------------------------------------
def test_windows_mujoco_path_not_typo():
    src = (REPO / "python" / ".cibuildwheels.toml").read_text()
    assert "C:/Program/mujoco/bin" not in src
    assert "mujoco/bin" in src


# ---- M-20 -------------------------------------------------------------
def test_root_cmake_sets_pic_on_orc_target():
    src = (REPO / "CMakeLists.txt").read_text()
    assert "POSITION_INDEPENDENT_CODE ON" in src


# ---- M-21 -------------------------------------------------------------
def test_buildwheels_yaml_has_smoke_test_step():
    src = (REPO / ".github" / "workflows" / "release.yml").read_text()
    assert "import orcpy" in src


# ---- M-22 -------------------------------------------------------------
@pytest.mark.parametrize(
    "fname",
    ["Kinova_bindings.cpp", "LinearAxis_bindings.cpp"],
)
def test_copy_binding_present(fname: str):
    src = _read(BIND_DIR / fname)
    assert re.search(r'\.def\(\s*"copy"', src), f"{fname} missing copy() binding"


# ---- Verification (documentation) tests for false positives -----------
def test_H11_subclasses_set_model_before_super_init():
    # Iiwa, Kinova, LinearAxis assign self.model before calling super().__init__().
    for fname in ("Iiwa.py", "Kinova.py", "LinearAxis.py"):
        src = (REPO / "python" / "robots" / fname).read_text()
        m_model = re.search(r"self\.model\s*=", src)
        m_super = re.search(r"super\(\)\.__init__", src)
        assert m_model and m_super
        assert m_model.start() < m_super.start(), (
            f"{fname}: self.model assigned after super().__init__()"
        )
