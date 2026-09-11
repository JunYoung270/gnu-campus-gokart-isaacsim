#!/usr/bin/env python3
"""Compose and validate the Stage D visual-only GNU Campus USD stage."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from isaacsim import SimulationApp


APP = SimulationApp({"headless": True})

from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402


VISUAL_ASSETS = (
    "terrain_visual",
    "road_visual",
    "buildings_visual",
    "sidewalk_visual",
)


def add_box(stage, path, center, size, color, role):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(cube)
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    cube.GetPrim().SetCustomDataByKey("gnuCampus:validationRole", role)
    return cube


def aligned_bbox(cache, prim):
    box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
    return [[float(v) for v in box.GetMin()], [float(v) for v in box.GetMax()]]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-c", required=True, type=Path)
    parser.add_argument("--stage-d", required=True, type=Path)
    args = parser.parse_args()
    stage_c = args.stage_c.resolve()
    stage_d = args.stage_d.resolve()
    output = stage_d / "usd" / "gnu_campus_visual.usda"

    expected = json.loads(
        (stage_c / "intermediate" / "mesh_statistics.json").read_text()
    )
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetMetadata("comment", "GNU Campus Stage D visual-only sample; ENU metres")
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    campus = UsdGeom.Xform.Define(stage, "/World/GNU_Campus")
    campus.GetPrim().SetCustomDataByKey("gnuCampus:coordinateSystem", "right-handed ENU")
    campus.GetPrim().SetCustomDataByKey("gnuCampus:axis", "X East, Y North, Z Up")
    campus.GetPrim().SetCustomDataByKey("gnuCampus:unit", "metre")
    visual = UsdGeom.Scope.Define(stage, "/World/GNU_Campus/Visual")

    for name in VISUAL_ASSETS:
        label = "".join(word.title() for word in name.removesuffix("_visual").split("_"))
        prim = UsdGeom.Xform.Define(stage, f"/World/GNU_Campus/Visual/{label}")
        prim.GetPrim().GetReferences().AddReference(f"components/{name}.usd", "/World")
        # Measured inverse of Stage C's glTF payload mapping:
        # R_x(+90 deg): (E, U, -N) -> (E, N, U).
        prim.AddRotateXOp(opSuffix="gltfPayloadToENU").Set(90.0)
        prim.GetPrim().SetCustomDataByKey("gnuCampus:sourceAsset", f"{name}.glb")
        prim.GetPrim().SetCustomDataByKey("gnuCampus:axisCorrection", "RotateX(+90deg)")

    validation = UsdGeom.Scope.Define(stage, "/World/GNU_Campus/Validation")
    validation.GetPrim().SetCustomDataByKey("gnuCampus:visualOnly", True)
    add_box(stage, "/World/GNU_Campus/Validation/Origin", (0, 0, 0), (0.4, 0.4, 0.4), (1, 1, 1), "origin")
    add_box(stage, "/World/GNU_Campus/Validation/EastAxis", (5, 0, 0), (10, 0.12, 0.12), (1, 0, 0), "east +10m")
    add_box(stage, "/World/GNU_Campus/Validation/NorthAxis", (0, 5, 0), (0.12, 10, 0.12), (0, 1, 0), "north +10m")
    add_box(stage, "/World/GNU_Campus/Validation/UpAxis", (0, 0, 5), (0.12, 0.12, 10), (0, 0.25, 1), "up +10m")
    add_box(stage, "/World/GNU_Campus/Validation/ScaleCube1m", (3, 3, 6), (1, 1, 1), (1, 0.8, 0), "1m cube")
    stage.GetRootLayer().Save()

    reopened = Usd.Stage.Open(str(output))
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=False,
    )
    measurements = {}
    max_error = 0.0
    for name in VISUAL_ASSETS:
        label = "".join(word.title() for word in name.removesuffix("_visual").split("_"))
        bbox = aligned_bbox(cache, reopened.GetPrimAtPath(f"/World/GNU_Campus/Visual/{label}"))
        target = expected[name]["reimport_bounds_enu_m"]
        error = max(abs(bbox[i][j] - target[i][j]) for i in range(2) for j in range(3))
        max_error = max(max_error, error)
        measurements[name] = {"usd_bbox_enu_m": bbox, "expected_bbox_enu_m": target, "max_abs_error_m": error}

    scale_bbox = aligned_bbox(cache, reopened.GetPrimAtPath("/World/GNU_Campus/Validation/ScaleCube1m"))
    scale_extent = [scale_bbox[1][i] - scale_bbox[0][i] for i in range(3)]
    physics_scenes = [str(p.GetPath()) for p in reopened.Traverse() if p.IsA(UsdPhysics.Scene)]
    collision_apis = [str(p.GetPath()) for p in reopened.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    result = {
        "stage": str(output),
        "up_axis": str(UsdGeom.GetStageUpAxis(reopened)),
        "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(reopened)),
        "axis_correction": "RotateX(+90deg): (E,U,-N) -> (E,N,U)",
        "components": measurements,
        "max_component_bbox_error_m": max_error,
        "scale_cube_extent_m": scale_extent,
        "physics_scenes": physics_scenes,
        "collision_api_prims": collision_apis,
        "pass": (
            str(UsdGeom.GetStageUpAxis(reopened)) == "Z"
            and abs(UsdGeom.GetStageMetersPerUnit(reopened) - 1.0) < 1e-12
            and max_error < 2e-5
            and max(abs(v - 1.0) for v in scale_extent) < 1e-9
            and not physics_scenes
            and not collision_apis
        ),
    }
    validation_file = stage_d / "debug" / "visual_usd_validation.json"
    validation_file.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if not result["pass"]:
        raise SystemExit("visual USD validation failed")


if __name__ == "__main__":
    try:
        main()
    finally:
        APP.close()
