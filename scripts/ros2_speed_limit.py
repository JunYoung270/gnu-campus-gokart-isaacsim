"""Shared validation and clamping for ROS 2 target-speed commands."""

from __future__ import annotations

import math


def parse_speed_limit(value: object) -> float | None:
    """Return a positive limit in m/s, or ``None`` when it is not configured."""
    if value is None or (isinstance(value, str) and not value.strip()):
        return None
    limit = float(value)
    if not math.isfinite(limit) or limit <= 0.0:
        raise ValueError("ROS 2 maximum speed must be finite and positive")
    return limit


def clamp_speed(speed: object, limit: float | None) -> float:
    """Validate a speed command and clamp its magnitude when a limit is set."""
    command = float(speed)
    if not math.isfinite(command):
        raise ValueError("ROS 2 speed command must be finite")
    if limit is None:
        return command
    return max(-limit, min(limit, command))
