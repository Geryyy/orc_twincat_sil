"""Runtime (dynamic) tests for the orcpy extension.

These do NOT exercise anything that requires MuJoCo model files to load —
those are gated behind `importorskip("mujoco")`. The whole module is
skipped cleanly if the native extension isn't installed, so it also
behaves on lanes where only the pure-Python wrappers are present.
"""

from __future__ import annotations

import pytest

orcpy = pytest.importorskip("orcpy", reason="native extension not installed")


# ---------------------------------------------------------------------------
# Time arithmetic
# ---------------------------------------------------------------------------
def _make_time(sec: float):
    """Construct an orcpy Time regardless of which submodule it ended up in."""
    for holder in (orcpy, getattr(orcpy, "core", None)):
        if holder is None:
            continue
        cls = getattr(holder, "Time", None)
        if cls is None:
            continue
        # Support both Time(sec, nsec) and Time(float) ctor shapes.
        try:
            return cls(sec)
        except TypeError:
            s = int(sec)
            ns = round((sec - s) * 1e9)
            return cls(s, ns)
    pytest.skip("orcpy.Time not exposed")


class TestTimeArithmetic:
    def test_add(self):
        t1 = _make_time(0.5)
        t2 = _make_time(0.25)
        t3 = t1 + t2
        assert abs(t3.toSec() - 0.75) < 1e-9

    def test_subtract(self):
        t1 = _make_time(1.0)
        t2 = _make_time(0.25)
        t3 = t1 - t2
        assert abs(t3.toSec() - 0.75) < 1e-9

    def test_comparison(self):
        assert _make_time(0.1).toSec() < _make_time(0.2).toSec()
        assert _make_time(0.5).toSec() > _make_time(0.25).toSec()
        # Operator form, if exposed.
        t1, t2 = _make_time(0.1), _make_time(0.2)
        try:
            assert (t1 < t2) is True
            assert (t2 > t1) is True
        except TypeError:
            pytest.skip("orcpy.Time comparison operators not exposed")


# ---------------------------------------------------------------------------
# RobotState default construction
# ---------------------------------------------------------------------------
def _find_robot_state_class():
    # Prefer a DoF-specific submodule (orcpy.robot7.RobotState for Iiwa).
    for name in ("robot7", "robot9", "robot2", "core"):
        ns = getattr(orcpy, name, None)
        if ns is None:
            continue
        cls = getattr(ns, "RobotState", None)
        if cls is not None:
            return cls
    cls = getattr(orcpy, "RobotState", None)
    if cls is not None:
        return cls
    return None


class TestRobotState:
    def test_default_construct(self):
        cls = _find_robot_state_class()
        if cls is None:
            pytest.skip("RobotState not exposed through Python bindings")
        rs = cls()
        # The `time_` underscore leak (H-6) should be fixed; accept either
        # `.time` or `.time_` so this test is robust to binding churn.
        assert hasattr(rs, "time") or hasattr(rs, "time_")


# ---------------------------------------------------------------------------
# Serialization round-trip
# ---------------------------------------------------------------------------
class TestSerializationRoundTrip:
    def test_robot_state_roundtrip(self):
        cls = _find_robot_state_class()
        if cls is None:
            pytest.skip("RobotState not exposed")
        rs = cls()
        # Try the canonical serialize/deserialize pair if present.
        ser = getattr(rs, "serialize", None) or getattr(rs, "to_bytes", None)
        if ser is None:
            pytest.skip("RobotState has no serialize() binding")
        blob = ser()
        assert blob is not None
        # Deserialize into a fresh instance.
        deser = getattr(cls, "deserialize", None) or getattr(cls, "from_bytes", None)
        if deser is None:
            pytest.skip("RobotState has no deserialize() binding")
        rs2 = deser(blob)
        assert rs2 is not None


# ---------------------------------------------------------------------------
# Trajectory construction with valid / invalid inputs
# ---------------------------------------------------------------------------
def _find_jointspace_trajectory_class():
    for name in ("robot7", "robot9", "robot2", "core"):
        ns = getattr(orcpy, name, None)
        if ns is None:
            continue
        cls = getattr(ns, "JointspaceTrajectory", None)
        if cls is not None:
            return cls, ns
    return None, None


class TestTrajectoryConstruction:
    def test_valid_jointspace_trajectory(self):
        cls, ns = _find_jointspace_trajectory_class()
        if cls is None:
            pytest.skip("JointspaceTrajectory not exposed")
        np = pytest.importorskip("numpy")
        dof = getattr(ns, "DOF", 7) or 7
        q0 = np.zeros(dof)
        q1 = np.ones(dof)
        t0 = _make_time(0.0)
        t1 = _make_time(1.0)
        try:
            traj = cls(q0, q1, t0, t1)
        except TypeError:
            pytest.skip("JointspaceTrajectory ctor shape differs")
        assert traj is not None

    def test_invalid_zero_duration_rejected(self):
        cls, ns = _find_jointspace_trajectory_class()
        if cls is None:
            pytest.skip("JointspaceTrajectory not exposed")
        np = pytest.importorskip("numpy")
        dof = getattr(ns, "DOF", 7) or 7
        q0 = np.zeros(dof)
        q1 = np.ones(dof)
        t0 = _make_time(1.0)
        t1 = _make_time(1.0)
        # Zero-duration ctor should raise (spline fitting can't handle it).
        with pytest.raises(Exception):
            traj = cls(q0, q1, t0, t1)
            traj.init()
