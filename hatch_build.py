"""Package the asset tree into standard wheels only.

An editable install imports ``sx_embodiments`` from this checkout, where ``asset_root()`` already
resolves ``assets/`` beside the package, so copying the 730 MiB tree into every editable wheel
buys nothing and fills each workspace's build cache. A standard wheel carries it as
``sx_embodiments/_assets``.
"""

from typing import Any

from hatchling.builders.hooks.plugin.interface import BuildHookInterface


class AssetTreeHook(BuildHookInterface):  # type: ignore[type-arg]
    def initialize(self, version: str, build_data: dict[str, Any]) -> None:
        if version == "standard":
            build_data["force_include"]["assets"] = "sx_embodiments/_assets"
