#!/usr/bin/env python3
"""Read-only inspection of the existing gokart USD for safe spawn placement."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from isaacsim import SimulationApp


APP = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    stage = Usd.Stage.Open(str(args.vehicle.resolve()))
    if stage is None:
        raise RuntimeError(f"cannot open {args.vehicle}")
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=False,
        ignoreVisibility=True,
    )
    bbox = cache.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedBox()
    result = {
        "vehicle": str(args.vehicle.resolve()),
        "default_prim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
        "bbox_local_m": [[float(value) for value in bbox.GetMin()], [float(value) for value in bbox.GetMax()]],
        "rigid_bodies": [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)],
        "collision_prims": [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.CollisionAPI)],
        "wheel_named_prims": [str(prim.GetPath()) for prim in stage.Traverse() if "wheel" in prim.GetName().lower() or "tire" in prim.GetName().lower()],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    try:
        main()
    finally:
        APP.close()
