"""Runtime smoke tests: actually execute every example under examples/python.

Each example is spawned as a subprocess with a strict timeout. The test
classifies every outcome and fails loudly on unexpected behaviour so
bugs surface on the next pytest run instead of waiting for someone to
manually re-run an example.

Outcomes
--------
* ``exit 0 within timeout``         → PASS.
* ``timed out``                     → PASS for long-running examples
                                      (servers, viewers, sim loops),
                                      FAIL otherwise.
* ``non-zero exit before timeout``  → FAIL — test displays the tail of
                                      stderr so the error is obvious.

Known broken / environment-dependent examples are marked ``xfail`` with
a short reason. When the underlying bug is fixed, the xfail flips to an
unexpected pass and the test starts failing, forcing us to delete the
entry.
"""

from __future__ import annotations

import contextlib
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
EXAMPLES = REPO / "examples" / "python"
HEADLESS_SIM = EXAMPLES / "orc_paper_experiments" / "_headless_sim.py"
MODELS = REPO / "models"


def _preset_missing(name: str) -> bool:
    """Preset .mjb absent from the repo (needs generation via MujocoMeshManager)."""
    return not (MODELS / "presets" / name).exists()


# Longer than the worst-case setup in any runnable example, shorter than
# typical CI per-test caps. Raise if flaky on slower runners.
TIMEOUT = 15


# ---------------------------------------------------------------------------
# Per-example configuration.
#
# Keys are paths relative to examples/python/. Any example not listed is
# treated as a plain "run once, expect clean exit within TIMEOUT".
#
#   args:         extra argv
#   env:          extra environment vars
#   timeout:      override TIMEOUT for this example
#   long_running: timing out is acceptable (server, viewer, etc.)
#   needs_sim:    "iiwa" or "kinova" — spawn _headless_sim.py first
#   xfail:        reason string — test is skipped-as-xfail
#   skip:         reason string — test is outright skipped (e.g. missing
#                 system dep that's orthogonal to ORC)
# ---------------------------------------------------------------------------

EXAMPLE_CONFIG: dict[str, dict] = {
    # --- orc_paper_experiments ------------------------------------------
    "orc_paper_experiments/_headless_sim.py": {
        "args": ["--robot", "iiwa", "--duration", "2"],
    },
    "orc_paper_experiments/sending_leminscate.py": {
        # Sender needs a running _headless_sim on the UDP endpoint;
        # without one it sits waiting for the first RobotState packet.
        "long_running": True,
    },
    "orc_paper_experiments/sending_simple.py": {
        "xfail": "send_hybrid_force_motion_trajectory → missing FlatBufferSerializer7."
        "serialize_hybrid_force_motion_trajectory binding",
    },
    "orc_paper_experiments/sending_square_octogon.py": {
        "xfail": "same missing serializer binding as sending_simple.py",
    },
    # --- hybrid_force_motion (preset .mjb generated via MujocoMeshManager) -
    # The simulate_*.py variants self-generate the preset on first run
    # if MujocoMeshManager is installed; the sender scripts only load it.
    # Test xfails iff the .mjb file is absent.
    "hybrid_force_motion/experiment_whiteboard/sending_simple.py": {
        "requires_preset": "iiwa_hanging_adapter_pen.mjb",
    },
    "hybrid_force_motion/experiment_whiteboard/sending_circle_square.py": {
        "requires_preset": "iiwa_hanging_adapter_pen.mjb",
    },
    "hybrid_force_motion/experiment_whiteboard/simulate.py": {
        "requires_preset": "iiwa_hanging_adapter_pen.mjb",
        "long_running": True,
    },
    "hybrid_force_motion/sponge/sending_ex.py": {
        "requires_preset": "iiwa_hanging_with_sponge.mjb",
    },
    "hybrid_force_motion/sponge/simulate_iiwa.py": {
        "requires_preset": "iiwa_hanging_with_sponge.mjb",
        "long_running": True,
    },
    # --- top-level examples ---------------------------------------------
    "interpolator_example.py": {},  # self-contained, exits quickly under Agg
    "sending_iiwa_ex.py": {
        # Whole-sequence move_jointspace → taskspace → back to zero needs
        # a running iiwa sim to read EE pose from.
        "needs_sim": "iiwa",
        "timeout": 30,
    },
    "sending_kinova_ex.py": {},  # plain trajectory push; exits cleanly
    "sending_linear_axis_ex.py": {},  # simulation=True → 127.0.0.1 send
    "sending_robot9dof_ex.py": {
        "long_running": True,  # long blocking move_jointspace calls
    },
    "sending_param_ex.py": {},  # prints usage and exits 0
    "simulate_iiwa.py": {"long_running": True},
    "simulate_kinova.py": {"long_running": True},
    "simulate_linear_axis.py": {"long_running": True},
    "simulate_robot9dof.py": {
        "requires_preset": "pascal_bernoulli.mjb",
        "long_running": True,
    },
    "kinova_simulation.py": {"long_running": True},
    "dense_trajectory_streaming_new.py": {
        # Override the hardcoded 10-minute duration via env var and run
        # against _headless_sim for a ~6-second end-to-end check.
        "needs_sim": "iiwa",
        "env": {"ORC_DENSE_TRAJ_DURATION_MIN": "0.1"},
        "timeout": 30,
    },
}


def _discover_examples() -> list[Path]:
    return sorted(p for p in EXAMPLES.rglob("*.py") if "__pycache__" not in p.parts)


def _params() -> list:
    out = []
    for path in _discover_examples():
        rel = str(path.relative_to(EXAMPLES))
        cfg = EXAMPLE_CONFIG.get(rel, {})
        marks = []
        if reason := cfg.get("skip"):
            marks.append(pytest.mark.skip(reason=reason))
        if reason := cfg.get("xfail"):
            # strict=True flips unexpected passes back to failures so that
            # fixing an example removes its entry here automatically.
            marks.append(pytest.mark.xfail(reason=reason, strict=True))
        # ``requires_preset`` is conditional: xfail only when the .mjb is
        # absent. With the preset locally generated (via the matching
        # script in models/generation_scripts/) the test runs normally.
        if (preset := cfg.get("requires_preset")) and _preset_missing(preset):
            marks.append(
                pytest.mark.xfail(
                    reason=f"preset {preset} not present — run "
                    f"models/generation_scripts/generate_{Path(preset).stem}.py",
                    strict=True,
                )
            )
        out.append(pytest.param(path, id=rel, marks=marks))
    return out


def _tail(text: str, n: int = 40) -> str:
    lines = text.splitlines()
    return "\n".join(lines[-n:])


@contextlib.contextmanager
def _running_headless_sim(robot: str, duration_s: int = 60):
    """Yield a background _headless_sim.py process; tear it down on exit."""
    proc = subprocess.Popen(
        [sys.executable, str(HEADLESS_SIM), "--robot", robot, "--duration", str(duration_s)],
        cwd=REPO,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    # Give the C++ TrajectoryServer a moment to bind its UDP port before
    # the sender tries to push a trajectory onto it.
    time.sleep(1.0)
    try:
        yield proc
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()


def test_examples_discovered() -> None:
    # Guard against silent empty-glob regressions if the directory moves.
    assert _discover_examples(), "no example scripts discovered under examples/python"


@pytest.mark.parametrize("path", _params())
def test_example_runs(path: Path) -> None:
    rel = str(path.relative_to(EXAMPLES))
    cfg = EXAMPLE_CONFIG.get(rel, {})

    long_running = bool(cfg.get("long_running"))
    args = cfg.get("args", [])
    timeout = int(cfg.get("timeout", TIMEOUT))

    env = {
        **os.environ,
        # Prevent matplotlib plt.show() from blocking on a GUI backend.
        "MPLBACKEND": "Agg",
        # Ensure bundled orcpy models resolve regardless of CWD.
        "ORCPY_MODELS_DIR": str(REPO / "models"),
        **cfg.get("env", {}),
    }

    sim_ctx = (
        _running_headless_sim(cfg["needs_sim"], duration_s=timeout + 10)
        if cfg.get("needs_sim")
        else contextlib.nullcontext()
    )

    cmd = [sys.executable, str(path), *args]
    with sim_ctx:
        # start_new_session so we can kill the whole process group on timeout —
        # otherwise UDP receive threads can outlive the parent.
        proc = subprocess.Popen(
            cmd,
            cwd=REPO,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        timed_out = False
        try:
            out, _ = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                out, _ = proc.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                out, _ = proc.communicate()

    text = out.decode(errors="replace")

    if timed_out:
        if not long_running:
            pytest.fail(
                f"{rel} timed out after {timeout}s (not marked long_running). "
                f"Tail of output:\n{_tail(text)}"
            )
        return  # long-running: timeout without a crash is a pass

    if proc.returncode != 0:
        pytest.fail(f"{rel} exited with code {proc.returncode}. Tail of output:\n{_tail(text)}")
