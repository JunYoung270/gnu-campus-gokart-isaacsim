#!/usr/bin/env python3
"""Open the full driven-area visual stage and keep the GUI alive."""

from isaacsim import SimulationApp

APP = SimulationApp(
    {
        "headless": False,
        "renderer": "RaytracedLighting",
        "width": 1440,
        "height": 900,
    }
)

import numpy as np
import omni.usd
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, UsdGeom, UsdLux

USD_PATH = "/workspace/stage_d/usd/gnu_campus_visual.usda"
context = omni.usd.get_context()
if context.open_stage(USD_PATH) is False:
    raise RuntimeError(f"Could not open {USD_PATH}")
for _ in range(180):
    APP.update()

stage = context.get_stage()
if stage is None:
    raise RuntimeError("USD stage is unavailable after open_stage")

# Runtime-only lighting; the source layer is mounted read-only and never saved.
dome = UsdLux.DomeLight.Define(stage, "/World/InteractiveView/Dome")
dome.CreateIntensityAttr(1000.0)
sun = UsdLux.DistantLight.Define(stage, "/World/InteractiveView/Sun")
sun.CreateIntensityAttr(3000.0)
sun.CreateAngleAttr(0.8)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(45.0, -35.0, -30.0))

# Overview of X=-105..325 m, Y=-215..45 m.
set_camera_view(
    eye=np.asarray((560.0, -650.0, 500.0), dtype=np.float64),
    target=np.asarray((110.0, -85.0, 0.0), dtype=np.float64),
    camera_prim_path="/OmniverseKit_Persp",
)
for _ in range(180):
    APP.update()

print("GNU_CAMPUS_FULL_GUI_READY", flush=True)
try:
    while APP.is_running():
        APP.update()
finally:
    APP.close()
