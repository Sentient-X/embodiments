import errno
import os
from pathlib import Path

import pytest

from sx_embodiments.ego_wuji import _publish


@pytest.mark.skipif(os.name == "nt", reason="Windows rename already refuses replacement")
def test_portable_mesh_on_volume_without_hardlinks(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    def unsupported(*_args: object) -> None:
        raise OSError(errno.EOPNOTSUPP, "volume does not implement hard links")

    monkeypatch.setattr(os, "link", unsupported)
    mesh = tmp_path / "finger.stl"
    _publish(mesh, b"mesh")
    _publish(mesh, b"mesh")
    with pytest.raises(ValueError, match="existing file differs"):
        _publish(mesh, b"different mesh")
    assert mesh.read_bytes() == b"mesh"
    assert list(tmp_path.iterdir()) == [mesh]
