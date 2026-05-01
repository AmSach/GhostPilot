#!/usr/bin/env python3
"""Legacy compatibility shim for GhostPilot mission execution."""

from __future__ import annotations

from typing import Any

try:
    from ghostpilot_agent.executor import MissionExecutor, main
except ImportError:
    from .executor import MissionExecutor, main

MissionResult = dict[str, Any]

__all__ = ['MissionExecutor', 'MissionResult', 'main']
