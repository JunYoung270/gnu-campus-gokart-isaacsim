# GNU Campus Stage D USD sample

This directory contains generated Stage D artifacts for the approved Stage C
100 m x 95 m sample ROI. Source assets in `gnu_campus_stage_C_sample` and the
existing simulator, vehicle, ROS2, MOZA, and dynamics files are not modified.

Coordinate contract:

- stage: right-handed ENU, X East, Y North, Z Up
- unit: metre (`metersPerUnit = 1`)
- source GLB payload: `(x, y, z)_gltf = (E, U, -N)`

Generated artifacts are separated into `usd`, `validation`, `debug`, `scripts`,
and `report`.
