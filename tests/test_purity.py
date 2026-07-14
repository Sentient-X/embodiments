"""sx-embodiments is a dependency-free contract package."""

import ast
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src" / "sx_embodiments"
_ALLOWED = frozenset(sys.stdlib_module_names) | {"sx_embodiments"}


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


def test_no_environment_reads() -> None:
    for path in _modules():
        tree = ast.parse(path.read_text())
        assert not [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Attribute) and node.attr in {"environ", "getenv"}
        ]
