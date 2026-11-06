"""sx-embodiments depends only on the lower sx-contracts vocabulary package.

Two optional extras widen it for one tree each: ``capture`` lets ``encoder_serial.py`` read a
serial port, ``calibrate`` lets the solvers under ``calibrate/`` use NumPy, SciPy and OpenCV,
and ``render`` lets ``render_pack.py`` use NumPy and the frames kernel ``sx_geometry`` (its
root rotation computed from axes, #246), the ``collision_proxies.py`` it fits
use NumPy and SciPy (its convex pieces), as ``tool_frames.py`` reading the jaws from that
geometry uses NumPy and ``frames.py`` (each frame's axes and roles) the frames kernel,
``sx_geometry``. Nothing else in the package may import them,
so the registry stays importable bare.
"""

import ast
import subprocess
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "sx_embodiments"
_ALLOWED = frozenset(sys.stdlib_module_names) | {"sx_contracts", "sx_embodiments"}

_EXTRAS = {
    "encoder_serial.py": frozenset({"serial"}),
    "calibrate": frozenset({"numpy", "scipy", "cv2"}),
    "render_pack.py": frozenset({"numpy", "sx_geometry"}),
    "collision_proxies.py": frozenset({"numpy", "scipy"}),
    "tool_frames.py": frozenset({"numpy"}),
    "frames.py": frozenset({"numpy", "sx_geometry"}),
}

# The sanctioned environment reads live in assets.py alone: the local-tree override,
# the asset mirror, and its cache root. No other module reads the environment.
_ENV_ALLOWED = {"assets.py"}


def _modules() -> list[Path]:
    paths = sorted(SRC.rglob("*.py"))
    assert paths, f"no modules found under {SRC}"
    return paths


def test_imports_stay_within_the_budget() -> None:
    violations: list[str] = []
    for path in _modules():
        for node in ast.walk(ast.parse(path.read_text())):
            names: set[str] = set()
            if isinstance(node, ast.Import):
                names = {alias.name.split(".")[0] for alias in node.names}
            elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
                names = {node.module.split(".")[0]}
            allowed = _ALLOWED | _EXTRAS.get(path.relative_to(SRC).parts[0], frozenset())
            for name in names - allowed:
                violations.append(f"{path.relative_to(SRC)}: imports {name}")
    assert not violations, "imports beyond the sx-embodiments budget:\n" + "\n".join(violations)


def test_no_environment_reads_outside_assets() -> None:
    for path in _modules():
        if path.name in _ENV_ALLOWED and path.parent == SRC:
            continue
        tree = ast.parse(path.read_text())
        offenders = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Attribute) and node.attr in {"environ", "getenv"}
        ]
        assert not offenders, f"{path.relative_to(SRC)} reads the environment"


def test_the_bare_registry_imports_no_extra() -> None:
    """``import sx_embodiments`` needs no extra: the render, calibrate and capture trees load
    only when imported, so the bare registry runs without NumPy or the frames kernel."""
    probe = (
        "import sys, sx_embodiments; "
        "loaded = [m for m in ('numpy', 'scipy', 'cv2', 'sx_geometry', 'serial') "
        "if m in sys.modules]; "
        "assert not loaded, loaded"
    )
    result = subprocess.run(
        [sys.executable, "-c", probe],
        cwd=SRC.parent,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
