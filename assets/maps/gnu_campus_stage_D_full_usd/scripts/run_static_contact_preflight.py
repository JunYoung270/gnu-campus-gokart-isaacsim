#!/usr/bin/env python3
"""Headless, bounded physics preflight before the interactive GUI contact gate."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import yaml
from isaacsim import SimulationApp


APP = SimulationApp({"headless": True})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402


def world_pose(stage: Usd.Stage, path: str) -> dict:
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise RuntimeError(f"missing prim {path}")
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    position = matrix.ExtractTranslation()
    rotation = matrix.ExtractRotation().GetQuaternion()
    return {
        "position_m": [float(position[i]) for i in range(3)],
        "orientation_wxyz": [float(rotation.GetReal()), *[float(v) for v in rotation.GetImaginary()]],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--steps", type=int, default=180)
    args = parser.parse_args()
    repo = args.repo.resolve()
    config = yaml.safe_load(args.config.read_text())
    environment = (repo / config["environment"]["asset"]).resolve()
    vehicle_cfg = next(vehicle for vehicle in config["vehicles"] if vehicle["name"] == "Ego_Vehicle")
    vehicle = (repo / vehicle_cfg["asset"]).resolve()

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    env = UsdGeom.Xform.Define(stage, "/World/Environment")
    env.AddScaleOp().Set(Gf.Vec3d(*config["environment"]["scale"]))
    env.AddRotateXYZOp().Set(Gf.Vec3d(*config["environment"]["rotation_euler"]))
    env.AddTranslateOp().Set(Gf.Vec3d(*config["environment"]["translation"]))
    env.GetPrim().GetReferences().AddReference(str(environment))

    ego = UsdGeom.Xform.Define(stage, "/World/Ego_Vehicle")
    ego.AddRotateXYZOp().Set(Gf.Vec3d(*vehicle_cfg["spawn_orientation"]))
    ego.AddTranslateOp().Set(Gf.Vec3d(*vehicle_cfg["spawn_position"]))
    ego.GetPrim().GetReferences().AddReference(str(vehicle))

    scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateSolverTypeAttr("PGS")
    physx_scene.CreateTimeStepsPerSecondAttr(60)
    context_api = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
    context_api.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    context_api.CreateVerticalAxisAttr(PhysxSchema.Tokens.posZ)
    context_api.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posX)

    base_path = "/World/Ego_Vehicle/Geometry/base_link"
    for _ in range(5):
        APP.update()
    initial = world_pose(stage, base_path)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    samples = [{"step": 0, **initial}]
    checkpoints = {30, 60, 120, args.steps}
    for step in range(1, args.steps + 1):
        APP.update()
        if step in checkpoints:
            samples.append({"step": step, **world_pose(stage, base_path)})
    timeline.pause()
    final = samples[-1]
    initial_z = initial["position_m"][2]
    final_z = final["position_m"][2]
    horizontal_drift = math.hypot(
        final["position_m"][0] - initial["position_m"][0],
        final["position_m"][1] - initial["position_m"][1],
    )
    collision_prims = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.CollisionAPI)]
    physics_scenes = [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    result = {
        "environment": str(environment),
        "vehicle": str(vehicle),
        "steps": args.steps,
        "samples": samples,
        "z_change_m": final_z - initial_z,
        "horizontal_drift_m": horizontal_drift,
        "collision_prim_count": len(collision_prims),
        "physics_scenes": physics_scenes,
        "pass": (
            len(physics_scenes) == 1
            and len(collision_prims) >= 31
            and -0.50 <= final_z - initial_z <= 0.10
            and final_z > -2.50
            and horizontal_drift < 1.0
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if not result["pass"]:
        raise SystemExit("static contact preflight failed")


if __name__ == "__main__":
    try:
        main()
    finally:
        APP.close()
