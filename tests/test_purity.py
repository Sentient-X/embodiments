"""sx-embodiments depends only on the lower sx-capabilities contract kernel."""

import ast
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src" / "sx_embodiments"
_ALLOWED = frozenset(sys.stdlib_module_names) | {"sx_capabilities", "sx_embodiments"}

# The one sanctioned environment read: assets.py resolves SX_EMBODIMENTS_ASSETS because
# Deployed workers may override the packaged description tree. Nothing else reads the environment.
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
            for name in names - _ALLOWED:
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
