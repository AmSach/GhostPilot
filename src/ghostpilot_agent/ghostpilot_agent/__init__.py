"""GhostPilot Agent - Agentic AI mission planning.

This package exposes the canonical agent API lazily so importing the package
itself does not require ROS2 unless the actual nodes are accessed.
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

__version__ = '0.1.0'

__all__ = ['HAS_ROS2', 'MissionParser', 'MissionExecutor', 'SYSTEM_PROMPT', 'MISSION_EXAMPLES', 'get_mission_prompt']


def __getattr__(name: str) -> Any:
    """Lazily resolve agent symbols."""
    if name in {'MissionParser', 'HAS_ROS2'}:
        module = import_module('ghostpilot_agent.mission_parser')
        return getattr(module, name)
    if name == 'MissionExecutor':
        module = import_module('ghostpilot_agent.executor')
        return getattr(module, name)
    if name in {'SYSTEM_PROMPT', 'MISSION_EXAMPLES', 'get_mission_prompt'}:
        module = import_module('ghostpilot_agent.mission_prompts')
        return getattr(module, name)
    raise AttributeError(name)
