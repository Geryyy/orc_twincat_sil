"""Smoke tests for every example under ``examples/python``.

These are source-level checks — they do NOT execute the examples (which
need MuJoCo plus a running simulator / real robot). What they catch:

1. ``test_example_compiles`` — every example parses as valid Python.
2. ``test_example_referenced_models_exist`` — every ``.mjb`` string
   literal in an example resolves to an existing file on disk.
3. ``test_example_uses_default_model_path_helper`` — examples that live
   directly under ``examples/python/`` route through the bundled
   ``default_model_path()`` helper instead of hard-coding ``models/…``.
4. ``test_default_model_path_helper_resolves_bundled_model`` — the
   helper itself resolves a bundled model to an absolute path.

Examples under ``examples/python/hybrid_force_motion/`` reference
generated presets that are not committed to the repo; those files are
xfailed with a clear reason so that dropping in the preset makes the
test pass without any further edits.
"""

from __future__ import annotations

import os
import py_compile
import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
EXAMPLES = REPO / "examples" / "python"
MODELS = REPO / "models"

# Presets referenced by some examples that the repo doesn't ship as
# binaries (~38–113 MB each). Each has a generation script under
# ``models/generation_scripts/`` that builds it from MujocoMeshManager
# components. If the .mjb is present (locally generated), the
# referenced-model test for the example passes; if not, it xfails with a
# pointer to the generation script instead of a mysterious FileNotFound.
_PRESET_GENERATORS = {
    "presets/iiwa_hanging_adapter_pen.mjb": "models/generation_scripts/generate_iiwa_hanging_adapter_pen.py",
    "presets/iiwa_hanging_with_sponge.mjb": "models/generation_scripts/generate_iiwa_hanging_with_sponge.py",
    "presets/pascal_bernoulli.mjb": "models/generation_scripts/generate_pascal_bernoulli.py",
}
MISSING_PRESETS = {ref for ref in _PRESET_GENERATORS if not (MODELS / ref).exists()}


def _discover_examples() -> list[Path]:
    return sorted(p for p in EXAMPLES.rglob("*.py") if "__pycache__" not in p.parts)


EXAMPLE_FILES = _discover_examples()
EXAMPLE_IDS = [str(p.relative_to(EXAMPLES)) for p in EXAMPLE_FILES]


# ---------------------------------------------------------------------------
# helper shared with the examples


def _resolve_helper():
    """Import ``default_model_path`` without pulling in orcpy.core.

    The helper is a stdlib-only function living in
    ``python/robots/util_functions.py``; we isolate it so the test can
    run without a built extension.
    """
    src = (REPO / "python" / "robots" / "util_functions.py").read_text()
    m = re.search(r"def default_model_path\(.*?\n(?=def |\Z)", src, re.DOTALL)
    assert m, "default_model_path() not found in util_functions.py"
    prelude = "import os\nfrom pathlib import Path\n"
    ns: dict = {"__file__": str(REPO / "python" / "robots" / "util_functions.py")}
    exec(compile(prelude + m.group(0), "util_functions.py", "exec"), ns)
    return ns["default_model_path"]


# ---------------------------------------------------------------------------
# model-reference extraction

_MJB_LITERAL = re.compile(r"""["']([^"']*\.mjb)["']""")


def _extract_mjb_refs(src: str) -> list[str]:
    """Return every string literal in *src* that ends with .mjb."""
    return _MJB_LITERAL.findall(src)


def _normalise_model_ref(ref: str) -> str:
    """Map a reference as it appears in source to a repo-relative name.

    Examples:
        "iiwa_hanging.mjb"                 -> "iiwa_hanging.mjb"
        "models/iiwa_hanging.mjb"          -> "iiwa_hanging.mjb"
        "models/presets/foo.mjb"           -> "presets/foo.mjb"
        "presets/pascal_bernoulli.mjb"     -> "presets/pascal_bernoulli.mjb"
    """
    ref = ref.replace("\\", "/")
    if ref.startswith("models/"):
        ref = ref[len("models/") :]
    return ref


# ---------------------------------------------------------------------------
# tests


def test_default_model_path_helper_resolves_bundled_model() -> None:
    resolve = _resolve_helper()
    p = resolve("iiwa_hanging.mjb")
    assert os.path.isabs(p), f"expected absolute path, got {p!r}"
    assert Path(p).exists(), f"helper did not find bundled model: {p}"


def test_examples_discovered() -> None:
    # Guard against the glob silently returning nothing (e.g. if the
    # directory is renamed without updating this file).
    assert EXAMPLE_FILES, "no example scripts discovered under examples/python"


@pytest.mark.parametrize("path", EXAMPLE_FILES, ids=EXAMPLE_IDS)
def test_example_compiles(path: Path, tmp_path: Path) -> None:
    try:
        py_compile.compile(str(path), cfile=str(tmp_path / "out.pyc"), doraise=True)
    except py_compile.PyCompileError as exc:  # pragma: no cover - diagnostic
        pytest.fail(f"{path.relative_to(REPO)} failed to compile: {exc.msg}")


@pytest.mark.parametrize("path", EXAMPLE_FILES, ids=EXAMPLE_IDS)
def test_example_referenced_models_exist(path: Path) -> None:
    src = path.read_text()
    refs = _extract_mjb_refs(src)
    if not refs:
        pytest.skip("no .mjb references in this example")

    missing: list[str] = []
    xfail_refs: list[str] = []
    for ref in refs:
        rel = _normalise_model_ref(ref)
        if rel in MISSING_PRESETS:
            xfail_refs.append(rel)
            continue
        if not (MODELS / rel).exists():
            missing.append(ref)

    if xfail_refs and not missing:
        pytest.xfail(
            f"preset not committed to repo (generate via models/generation_scripts/): {xfail_refs}"
        )
    assert not missing, f"{path.relative_to(REPO)} references models that do not exist: {missing}"


# Only the top-level examples/python/*.py scripts are constrained to use
# the helper. Nested examples (orc_paper_experiments, hybrid_force_motion)
# have their own path strategies (REPO_ROOT-relative, preset-specific).
TOP_LEVEL_EXAMPLES = [p for p in EXAMPLE_FILES if p.parent == EXAMPLES]
TOP_LEVEL_IDS = [p.name for p in TOP_LEVEL_EXAMPLES]

# Examples that intentionally compute their own path (not via the helper)
# because they need the raw REPO_ROOT for sibling data files too.
_TOP_LEVEL_HELPER_EXEMPT = {"kinova_simulation.py", "dense_trajectory_streaming_new.py"}


@pytest.mark.parametrize("path", TOP_LEVEL_EXAMPLES, ids=TOP_LEVEL_IDS)
def test_example_uses_default_model_path_helper(path: Path) -> None:
    if path.name in _TOP_LEVEL_HELPER_EXEMPT:
        pytest.skip(f"{path.name} uses a bespoke path strategy")
    src = path.read_text()
    if not _extract_mjb_refs(src):
        pytest.skip("example does not load a .mjb")
    # No raw "models/foo.mjb" literal passed to a robot constructor.
    raw = re.findall(r'["\']models/[^"\']+\.mjb["\']', src)
    assert not raw, f"{path.name} still hardcodes paths: {raw}"
    assert "default_model_path" in src, (
        f"{path.name} should route .mjb lookups through default_model_path()"
    )
