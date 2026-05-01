"""
GhostPilot Agent - Agentic AI layer for drone mission planning.

Public API for the agent module.
"""

from .mission_parser import MissionParser, Goal

__all__ = ['MissionParser', 'Goal']

# MissionExecutor requires ROS2 - import conditionally to avoid hard crash
try:
    from .executor import MissionExecutor
    __all__.append('MissionExecutor')
except ImportError:
    pass
