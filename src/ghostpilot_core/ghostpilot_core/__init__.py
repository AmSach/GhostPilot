"""GhostPilot Core - GPS-denied navigation stack.

The package exposes ROS2 nodes lazily so non-ROS tooling can import this
package metadata without crashing on missing ROS2 dependencies.
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

__version__ = '0.1.0'

__all__ = ['SLAMNode', 'PoseBridge', 'HAS_ROS2']


def __getattr__(name: str) -> Any:
    """Lazily resolve core nodes when accessed."""
    if name in {'SLAMNode', 'HAS_ROS2'}:
        module = import_module('ghostpilot_core.slam_node')
        return getattr(module, name)
    if name == 'PoseBridge':
        module = import_module('ghostpilot_core.pose_bridge')
        return getattr(module, name)
    raise AttributeError(name)
