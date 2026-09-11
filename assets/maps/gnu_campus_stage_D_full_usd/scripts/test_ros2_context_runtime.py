#!/usr/bin/env python3
"""Minimal Isaac Sim 6 ROS2Context runtime preflight."""

import os

from isaacsim import SimulationApp


APP = SimulationApp({"headless": True})

import omni.kit.app
import omni.timeline
import omni.usd


def main():
    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True):
        raise RuntimeError("isaacsim.ros2.bridge could not be enabled")
    template = (
        "/root/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.nodes/"
        "isaacsim/ros2/nodes/ogn/tests/usd/OgnROS2ContextTemplate.usda"
    )
    if not omni.usd.get_context().open_stage(template):
        raise RuntimeError(f"could not open {template}")
    for _ in range(30):
        APP.update()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(60):
        APP.update()
    timeline.pause()
    print(
        "ROS2_CONTEXT_RUNTIME_PASS "
        f"distro={os.environ.get('ROS_DISTRO', '<unset>')} "
        f"rmw={os.environ.get('RMW_IMPLEMENTATION', '<unset>')} "
        f"domain={os.environ.get('ROS_DOMAIN_ID', '<unset>')}",
        flush=True,
    )


if __name__ == "__main__":
    try:
        main()
    finally:
        APP.close()
