"""sx-embodiments depends only on the lower sx-contracts vocabulary package.

Two optional extras widen it for one tree each: ``capture`` lets ``encoder_serial.py`` read a
serial port, and ``calibrate`` lets the solvers under ``calibrate/`` use NumPy, SciPy and
OpenCV. Nothing else in the package may import them, so the registry stays importable bare.
"""

import ast
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "sx_embodiments"
_ALLOWED = frozenset(sys.stdlib_module_names) | {"sx_contracts", "sx_embodiments"}

_EXTRAS = {
    "encoder_serial.py": frozenset({"serial"}),
    "calibrate": frozenset({"numpy", "scipy", "cv2"}),
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
