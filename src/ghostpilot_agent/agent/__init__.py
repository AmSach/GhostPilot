"""GhostPilot Agent compatibility layer.

The canonical public API lives in the sibling modules in this directory. This
package keeps legacy imports working without forcing ROS2 to import at package
load time.
"""

from __future__ import annotations

from typing import Any

from .mission_parser import HAS_ROS2, MissionParser

Goal = dict[str, Any]
MissionResult = dict[str, Any]

try:
    from .executor import MissionExecutor
except Exception:  # noqa: BLE001
    MissionExecutor = None  # type: ignore[assignment]

__all__ = ['HAS_ROS2', 'Goal', 'MissionParser', 'MissionExecutor', 'MissionResult']
