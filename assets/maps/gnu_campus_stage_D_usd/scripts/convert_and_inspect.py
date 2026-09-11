#!/usr/bin/env python3
"""Convert Stage C GLBs with Isaac Asset Converter and measure USD results.

Run with Isaac Sim's bundled python.sh. This script only writes below the Stage
D output directory. It intentionally does not compose the final stages or add
physics; measured importer behavior is the gate for those later steps.
"""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path

from isaacsim import SimulationApp


APP = SimulationApp({"headless": True})

from pxr import Gf, Usd, UsdGeom  # noqa: E402
import omni.kit.asset_converter as asset_converter  # noqa: E402


ASSETS = (
    "terrain_visual",
    "road_visual",
    "buildings_visual",
    "sidewalk_visual",
    "terrain_collision_source",
    "road_collision_source",
    "buildings_collision_source",
)


def stage_bbox(stage: Usd.Stage) -> tuple[list[float], list[float]]:
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=False,
    )
    bbox = cache.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedBox()
    lo = bbox.GetMin()
    hi = bbox.GetMax()
    return [float(lo[i]) for i in range(3)], [float(hi[i]) for i in range(3)]


async def convert_one(source: Path, destination: Path) -> dict:
    context = asset_converter.AssetConverterContext()
    context.use_meter_as_world_unit = True
    context.convert_stage_up_z = True
    context.ignore_camera = True
    context.ignore_light = True
    context.ignore_animations = True
    context.export_preview_surface = True
    context.create_world_as_default_root_prim = True

    task = asset_converter.get_instance().create_converter_task(
        str(source), str(destination), None, context
    )
    success = await task.wait_until_finished()
    if not success:
        raise RuntimeError(
            f"conversion failed for {source.name}: "
            f"status={task.get_status()} error={task.get_error_message()}"
        )

    stage = Usd.Stage.Open(str(destination))
    if stage is None:
        raise RuntimeError(f"cannot open converted USD: {destination}")
    lo, hi = stage_bbox(stage)
    mesh_count = sum(prim.IsA(UsdGeom.Mesh) for prim in stage.Traverse())
    return {
        "source": str(source),
        "destination": str(destination),
        "source_bytes": source.stat().st_size,
        "destination_bytes": destination.stat().st_size,
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
        "world_bbox_min": lo,
        "world_bbox_max": hi,
        "mesh_prim_count": mesh_count,
        "default_prim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
    }


async def main_async(stage_c: Path, stage_d: Path, selected: list[str]) -> None:
    components = stage_d / "usd" / "components"
    debug = stage_d / "debug"
    components.mkdir(parents=True, exist_ok=True)
    debug.mkdir(parents=True, exist_ok=True)

    results = []
    for name in selected:
        source = stage_c / "mesh" / f"{name}.glb"
        destination = components / f"{name}.usd"
        if not source.is_file():
            raise FileNotFoundError(source)
        print(f"CONVERT {source.name} -> {destination.name}", flush=True)
        results.append(await convert_one(source, destination))

    output = debug / "asset_converter_measurements.json"
    output.write_text(json.dumps({"assets": results}, indent=2) + "\n")
    print(json.dumps({"measurement_file": str(output), "assets": results}, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-c", required=True, type=Path)
    parser.add_argument("--stage-d", required=True, type=Path)
    parser.add_argument("--asset", action="append", choices=ASSETS)
    args = parser.parse_args()
    selected = args.asset or list(ASSETS)
    try:
        asyncio.get_event_loop().run_until_complete(
            main_async(args.stage_c.resolve(), args.stage_d.resolve(), selected)
        )
    finally:
        APP.close()


if __name__ == "__main__":
    main()
