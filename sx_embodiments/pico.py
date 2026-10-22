"""Build identities for the camera-free PICO classic-YUBI observation profile.

Registration identifies a producer contract; Factory acceptance is separate. This
registry is deliberately empty until an exact APK and its source evidence are
registered. A PICO build never inherits a Quest coordinate convention.
"""

from collections.abc import Mapping
from types import MappingProxyType
from typing import Final

HEADSET_PICO_APK_BUILDS: Final[Mapping[str, str]] = MappingProxyType({})
"""App version to canonical ``sha256:<lowercase digest>`` for PICO observation v1."""
