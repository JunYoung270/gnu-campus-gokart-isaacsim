#!/usr/bin/env python3
"""Open map + existing gokart + static physics and keep the GUI alive.

This intentionally does not enable ROS2 or MOZA.  It is the interactive
collision/drop/static-contact gate before full simulator integration.
"""

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
import omni.timeline
import omni.usd
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics


ENVIRONMENT = "/workspace/repo/assets/maps/gnu_campus_stage_D_full_usd/usd/gnu_campus_drivable.usda"
VEHICLE = "/workspace/repo/assets/vehicles/kar_gokart_isaac_physx.usd"
SPAWN_POSITION = (26.242, -3.138, -0.800)
SPAWN_ORIENTATION = (0.0, 0.0, -8.0)

context = omni.usd.get_context()
if context.open_stage(ENVIRONMENT) is False:
    raise RuntimeError(f"Could not open {ENVIRONMENT}")
for _ in range(120):
    APP.update()

stage = context.get_stage()
if stage is None:
    raise RuntimeError("USD stage is unavailable after open_stage")

ego = UsdGeom.Xform.Define(stage, "/World/Ego_Vehicle")
ego.AddRotateXYZOp().Set(Gf.Vec3d(*SPAWN_ORIENTATION))
ego.AddTranslateOp().Set(Gf.Vec3d(*SPAWN_POSITION))
ego.GetPrim().GetReferences().AddReference(VEHICLE)

scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr(9.81)
physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
physx_scene.CreateSolverTypeAttr("PGS")
physx_scene.CreateTimeStepsPerSecondAttr(60)
vehicle_context = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
vehicle_context.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
vehicle_context.CreateVerticalAxisAttr(PhysxSchema.Tokens.posZ)
vehicle_context.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posX)

# Runtime-only lighting and camera. Nothing is saved to the environment layer.
dome = UsdLux.DomeLight.Define(stage, "/World/InteractiveContactView/Dome")
dome.CreateIntensityAttr(1000.0)
sun = UsdLux.DistantLight.Define(stage, "/World/InteractiveContactView/Sun")
sun.CreateIntensityAttr(3000.0)
sun.CreateAngleAttr(0.8)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(45.0, -35.0, -30.0))

set_camera_view(
    eye=np.asarray((18.0, -17.0, 3.0), dtype=np.float64),
    target=np.asarray((25.5, -6.7, -1.35), dtype=np.float64),
    camera_prim_path="/OmniverseKit_Persp",
)
for _ in range(60):
    APP.update()

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(180):
    APP.update()

base = stage.GetPrimAtPath("/World/Ego_Vehicle/Geometry/base_link")
matrix = UsdGeom.Xformable(base).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
position = matrix.ExtractTranslation()
print(
    "GNU_CAMPUS_STATIC_CONTACT_GUI_READY "
    f"base=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f})",
    flush=True,
)

try:
    while APP.is_running():
        APP.update()
finally:
    APP.close()
