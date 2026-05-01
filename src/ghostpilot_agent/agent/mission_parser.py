#!/usr/bin/env python3
"""Legacy compatibility shim for GhostPilot mission parsing."""

from __future__ import annotations

from typing import Any

try:
    from ghostpilot_agent.mission_parser import HAS_ROS2, MissionParser, main
except Exception:  # noqa: BLE001
    from .mission_parser import HAS_ROS2, MissionParser, main

Goal = dict[str, Any]

__all__ = ['HAS_ROS2', 'Goal', 'MissionParser', 'main']
