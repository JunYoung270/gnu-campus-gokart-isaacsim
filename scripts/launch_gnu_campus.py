#!/usr/bin/env python3
"""Convenience entry point for the GNU campus simulator configuration."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


LAUNCHER = Path(__file__).with_name("launch_sim.py")
CONFIG = Path(__file__).with_name("configs") / "gnu_campus_kar_physx.yaml"

if not any(arg == "--config" or arg.startswith("--config=") for arg in sys.argv[1:]):
    sys.argv.extend(("--config", str(CONFIG)))
runpy.run_path(str(LAUNCHER), run_name="__main__")
