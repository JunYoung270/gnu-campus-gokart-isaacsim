#!/usr/bin/env python3
"""Render the required visual-only Stage D validation views in Isaac Sim."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
from pathlib import Path

from isaacsim import SimulationApp


APP = SimulationApp(
    {
        "headless": False,
        "renderer": "RaytracedLighting",
        "width": 1280,
        "height": 720,
        "fast_shutdown": True,
    }
)

import numpy as np  # noqa: E402
from PIL import Image, ImageGrab  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402
import omni.usd  # noqa: E402
from omni.kit.viewport.utility import get_active_viewport  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402


VIEWS = {
    "visual_overview.png": ((120, -165, 115), (5, -22, 0), (0, 0, 1), 28.0),
    "visual_top_view.png": ((5, -22, 190), (5, -22, 0), (0, 1, 0), 32.0),
    "axis_validation.png": ((24, -28, 20), (3, 3, 4), (0, 0, 1), 35.0),
    "scale_validation.png": ((11, -5, 10), (3, 3, 6), (0, 0, 1), 48.0),
}


def grab_isaac_window():
    """Capture only the visible Isaac Sim window when SD RGB is unavailable."""
    tree = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
    candidates = []
    for line in tree.splitlines():
        if "Isaac Sim" not in line:
            continue
        match = re.search(r"(0x[0-9a-fA-F]+)\s+\"[^\"]*Isaac Sim[^\"]*\"", line)
        if match:
            candidates.append(match.group(1))
    if not candidates:
        raise RuntimeError("could not find a visible Isaac Sim X11 window")
    info = subprocess.check_output(["xwininfo", "-id", candidates[-1]], text=True)
    def value(label):
        match = re.search(rf"{re.escape(label)}:\s+(-?\d+)", info)
        if not match:
            raise RuntimeError(f"xwininfo is missing {label}")
        return int(match.group(1))
    x = value("Absolute upper-left X")
    y = value("Absolute upper-left Y")
    width = value("Width")
    height = value("Height")
    desktop = ImageGrab.grab(xdisplay=os.environ.get("DISPLAY"), all_screens=True)
    isaac_window = desktop.crop((x, y, x + width, y + height))
    # Default Isaac layout: viewport occupies the upper-left content panel.
    viewport_crop = isaac_window.crop((44, 30, int(width * 0.70), int(height * 0.68)))
    return viewport_crop, candidates[-1]


def main():
    print("STAGE_D_RENDER: imports ready", flush=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-d", required=True, type=Path)
    args = parser.parse_args()
    stage_d = args.stage_d.resolve()
    usd_file = stage_d / "usd" / "gnu_campus_visual.usda"
    validation = stage_d / "validation"
    validation.mkdir(parents=True, exist_ok=True)

    context = omni.usd.get_context()
    opened = context.open_stage(str(usd_file))
    print(f"STAGE_D_RENDER: open_stage returned {opened!r}", flush=True)
    if opened is False:
        raise RuntimeError(f"failed to open {usd_file}")
    for _ in range(20):
        APP.update()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError(f"USD context has no stage after opening {usd_file}")
    print("STAGE_D_RENDER: stage ready", flush=True)

    dome = UsdLux.DomeLight.Define(stage, "/World/StageD_Render/Dome")
    dome.CreateIntensityAttr(900.0)
    distant = UsdLux.DistantLight.Define(stage, "/World/StageD_Render/Sun")
    distant.CreateIntensityAttr(3000.0)
    distant.CreateAngleAttr(0.8)
    UsdGeom.Xformable(distant).AddRotateXYZOp().Set(Gf.Vec3f(45, -35, -30))

    camera_path = "/OmniverseKit_Persp"
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("Isaac Sim has no active viewport")
    viewport.resolution = (1280, 720)
    viewport.resolution_scale = 1
    viewport.set_active_camera(camera_path)
    print("STAGE_D_RENDER: viewport camera ready", flush=True)

    captures = []
    for filename, (eye, target, up, focal_length) in VIEWS.items():
        print(f"STAGE_D_RENDER: capturing {filename}", flush=True)
        try:
            set_camera_view(
                eye=np.asarray(eye, dtype=np.float64),
                target=np.asarray(target, dtype=np.float64),
                camera_prim_path=camera_path,
            )
            print(f"STAGE_D_RENDER: camera set for {filename}", flush=True)
            for frame in range(180):
                APP.update()
            print(f"STAGE_D_RENDER: warmup complete for {filename}", flush=True)
        except BaseException as exc:
            print(f"STAGE_D_RENDER: camera/warmup failed: {type(exc).__name__}: {exc}", flush=True)
            raise
        target_file = validation / filename
        if target_file.exists():
            target_file.unlink()
        screenshot, window_id = grab_isaac_window()
        rgb = np.asarray(screenshot.convert("RGB"))
        capture_method = "x11_crop_of_live_isaac_viewport"
        if rgb.ndim != 3 or rgb.shape[2] != 3 or float(rgb.std()) < 1.0:
            raise RuntimeError(f"blank or invalid RGB capture for {filename}: shape={rgb.shape} std={rgb.std()}")
        Image.fromarray(rgb, mode="RGB").save(target_file)
        scene_rgb = rgb[50:-90, 30:-30]
        scene_luma = scene_rgb.mean(axis=2)
        scene_nonblack_fraction = float(np.mean(scene_luma > 8.0))
        scene_std = float(scene_rgb.std())
        capture_pass = scene_std > 2.0 and scene_nonblack_fraction > 0.01
        captures.append(
            {
                "file": str(target_file),
                "shape": list(rgb.shape),
                "mean_rgb": [float(v) for v in rgb.mean(axis=(0, 1))],
                "std_rgb": [float(v) for v in rgb.std(axis=(0, 1))],
                "eye_enu_m": eye,
                "target_enu_m": target,
                "focal_length_mm": focal_length,
                "capture_method": capture_method,
                "x11_window_id": window_id,
                "scene_crop_std": scene_std,
                "scene_crop_nonblack_fraction": scene_nonblack_fraction,
                "pass": capture_pass,
            }
        )
    result = {
        "stage": str(usd_file),
        "renderer": "RaytracedLighting",
        "captures": captures,
        "pass": all(item["pass"] for item in captures),
        "failure_rule": "scene crop std > 2 and nonblack pixel fraction > 0.01",
    }
    output = stage_d / "debug" / "visual_render_validation.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    try:
        main()
    finally:
        APP.close()
