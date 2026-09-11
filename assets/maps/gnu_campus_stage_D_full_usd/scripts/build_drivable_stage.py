#!/usr/bin/env python3
"""Build and numerically validate the GNU Campus static-collision environment.

Run with Isaac Sim's bundled python.sh.  The generated environment deliberately
contains no PhysicsScene or vehicle: scripts/launch_sim.py owns the singleton
PhysicsScene and references the existing gokart independently.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from isaacsim import SimulationApp


APP = SimulationApp({"headless": True})

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402


VISUAL_ASSETS = (
    "terrain_visual",
    "road_visual",
    "buildings_visual",
    "sidewalk_visual",
)
COLLISION_ASSETS = {
    "terrain_collision_source": "GrassPhysicsMaterial",
    "road_collision_source": "AsphaltPhysicsMaterial",
    "buildings_collision_source": "BuildingPhysicsMaterial",
}
FRICTION = {
    "GrassPhysicsMaterial": (0.40, 0.60),
    "AsphaltPhysicsMaterial": (0.70, 0.90),
    "BuildingPhysicsMaterial": (0.60, 0.70),
}


def label(name: str) -> str:
    return "".join(word.title() for word in name.removesuffix("_visual").removesuffix("_collision_source").split("_"))


def add_reference(stage: Usd.Stage, path: str, asset: str) -> UsdGeom.Xform:
    prim = UsdGeom.Xform.Define(stage, path)
    prim.GetPrim().GetReferences().AddReference(f"components/{asset}.usd", "/World")
    prim.AddRotateXOp(opSuffix="gltfPayloadToENU").Set(90.0)
    prim.GetPrim().SetCustomDataByKey("gnuCampus:sourceAsset", f"{asset}.glb")
    prim.GetPrim().SetCustomDataByKey("gnuCampus:axisCorrection", "RotateX(+90deg)")
    return prim


def aligned_bbox(cache: UsdGeom.BBoxCache, prim: Usd.Prim) -> list[list[float]]:
    bbox = cache.ComputeWorldBound(prim).ComputeAlignedBox()
    return [[float(value) for value in bbox.GetMin()], [float(value) for value in bbox.GetMax()]]


def max_bbox_error(actual: list[list[float]], expected: list[list[float]]) -> float:
    return max(abs(actual[i][j] - expected[i][j]) for i in range(2) for j in range(3))


def build(stage_c: Path, stage_d: Path) -> dict:
    output = stage_d / "usd" / "gnu_campus_drivable.usda"
    expected = json.loads((stage_c / "intermediate" / "mesh_statistics.json").read_text())

    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetMetadata("comment", "GNU Campus full-area visual plus static collision; no PhysicsScene or vehicle")
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    campus = UsdGeom.Xform.Define(stage, "/World/GNU_Campus")
    campus.GetPrim().SetCustomDataByKey("gnuCampus:coordinateSystem", "right-handed ENU")
    campus.GetPrim().SetCustomDataByKey("gnuCampus:axis", "X East, Y North, Z Up")
    campus.GetPrim().SetCustomDataByKey("gnuCampus:unit", "metre")

    # launch_sim.py intentionally removes Isaac Sim's default light because the
    # established environments own their lighting. Author the same neutral
    # outdoor rig used by the accepted static-contact GUI so RaytracedLighting
    # does not render this environment black.
    UsdGeom.Scope.Define(stage, "/World/GNU_Campus/Lighting")
    dome = UsdLux.DomeLight.Define(stage, "/World/GNU_Campus/Lighting/Dome")
    dome.CreateIntensityAttr(1000.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/GNU_Campus/Lighting/Sun")
    sun.CreateIntensityAttr(3000.0)
    sun.CreateAngleAttr(0.8)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(45.0, -35.0, -30.0))

    UsdGeom.Scope.Define(stage, "/World/GNU_Campus/Visual")
    for asset in VISUAL_ASSETS:
        add_reference(stage, f"/World/GNU_Campus/Visual/{label(asset)}", asset)

    collision_root = UsdGeom.Xform.Define(stage, "/World/GNU_Campus/Collision")
    collision_root.MakeInvisible()
    collision_root.GetPrim().SetCustomDataByKey("gnuCampus:renderHiddenCollision", True)
    for asset in COLLISION_ASSETS:
        add_reference(stage, f"/World/GNU_Campus/Collision/{label(asset)}", asset)

    material_scope = UsdGeom.Scope.Define(stage, "/World/GNU_Campus/PhysicsMaterials")
    material_scope.GetPrim().SetCustomDataByKey("gnuCampus:physicsOnly", True)
    materials = {}
    for name, (dynamic, static) in FRICTION.items():
        material = UsdShade.Material.Define(stage, f"/World/GNU_Campus/PhysicsMaterials/{name}")
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateDynamicFrictionAttr(dynamic)
        physics.CreateStaticFrictionAttr(static)
        physics.CreateRestitutionAttr(0.0)
        materials[name] = material

    # Save once so referenced descendants are composed, then author collision
    # APIs and physics-material bindings as overrides in this root layer.
    stage.GetRootLayer().Save()
    for asset, material_name in COLLISION_ASSETS.items():
        root_path = f"/World/GNU_Campus/Collision/{label(asset)}"
        mesh_prims = [prim for prim in stage.Traverse() if prim.GetPath().HasPrefix(root_path) and prim.IsA(UsdGeom.Mesh)]
        if not mesh_prims:
            raise RuntimeError(f"no composed mesh prims below {root_path}")
        for prim in mesh_prims:
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
            PhysxSchema.PhysxCollisionAPI.Apply(prim)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                materials[material_name],
                UsdShade.Tokens.weakerThanDescendants,
                "physics",
            )
    stage.GetRootLayer().Save()

    reopened = Usd.Stage.Open(str(output))
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=False,
        ignoreVisibility=True,
    )
    measurements = {}
    collision_meshes = []
    maximum_error = 0.0
    for asset, material_name in COLLISION_ASSETS.items():
        root_path = f"/World/GNU_Campus/Collision/{label(asset)}"
        root = reopened.GetPrimAtPath(root_path)
        actual = aligned_bbox(cache, root)
        target = expected[asset]["reimport_bounds_enu_m"]
        error = max_bbox_error(actual, target)
        maximum_error = max(maximum_error, error)
        meshes = [prim for prim in reopened.Traverse() if prim.GetPath().HasPrefix(root_path) and prim.IsA(UsdGeom.Mesh)]
        for prim in meshes:
            relationship = prim.GetRelationship("material:binding:physics")
            collision_meshes.append({
                "path": str(prim.GetPath()),
                "group": asset,
                "collision_api": prim.HasAPI(UsdPhysics.CollisionAPI),
                "mesh_collision_api": prim.HasAPI(UsdPhysics.MeshCollisionAPI),
                "physx_collision_api": prim.HasAPI(PhysxSchema.PhysxCollisionAPI),
                "approximation": prim.GetAttribute("physics:approximation").Get(),
                "material_targets": [str(path) for path in relationship.GetTargets()] if relationship else [],
                "rigid_body_api": prim.HasAPI(UsdPhysics.RigidBodyAPI),
            })
        measurements[asset] = {
            "mesh_count": len(meshes),
            "usd_bbox_enu_m": actual,
            "expected_bbox_enu_m": target,
            "max_abs_error_m": error,
            "physics_material": material_name,
        }

    physics_scenes = [str(prim.GetPath()) for prim in reopened.Traverse() if prim.IsA(UsdPhysics.Scene)]
    material_values = {}
    for name in FRICTION:
        prim = reopened.GetPrimAtPath(f"/World/GNU_Campus/PhysicsMaterials/{name}")
        material_values[name] = {
            "dynamic_friction": prim.GetAttribute("physics:dynamicFriction").Get(),
            "static_friction": prim.GetAttribute("physics:staticFriction").Get(),
            "restitution": prim.GetAttribute("physics:restitution").Get(),
        }

    result = {
        "stage": str(output),
        "default_prim": str(reopened.GetDefaultPrim().GetPath()) if reopened.GetDefaultPrim() else None,
        "up_axis": str(UsdGeom.GetStageUpAxis(reopened)),
        "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(reopened)),
        "axis_correction": "RotateX(+90deg): (E,U,-N) -> (E,N,U)",
        "collision_components": measurements,
        "collision_mesh_count": len(collision_meshes),
        "collision_meshes": collision_meshes,
        "physics_materials": material_values,
        "physics_scenes": physics_scenes,
        "maximum_collision_bbox_error_m": maximum_error,
    }
    expected_counts = {
        "terrain_collision_source": 1,
        "road_collision_source": 1,
        "buildings_collision_source": 27,
    }
    result["pass"] = (
        result["default_prim"] == "/World"
        and result["up_axis"] == "Z"
        and abs(result["meters_per_unit"] - 1.0) < 1e-12
        and not physics_scenes
        and maximum_error < 2e-5
        and all(measurements[name]["mesh_count"] == count for name, count in expected_counts.items())
        and all(
            row["collision_api"]
            and row["mesh_collision_api"]
            and row["physx_collision_api"]
            and row["approximation"] == "none"
            and len(row["material_targets"]) == 1
            and not row["rigid_body_api"]
            for row in collision_meshes
        )
    )
    debug = stage_d / "debug" / "drivable_usd_validation.json"
    debug.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if not result["pass"]:
        raise SystemExit("drivable USD validation failed")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-c", required=True, type=Path)
    parser.add_argument("--stage-d", required=True, type=Path)
    args = parser.parse_args()
    build(args.stage_c.resolve(), args.stage_d.resolve())


if __name__ == "__main__":
    try:
        main()
    finally:
        APP.close()
