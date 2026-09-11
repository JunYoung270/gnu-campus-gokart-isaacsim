# GNU Campus Stage D runtime map

This directory contains the full driven-area GNU Gajwa Campus USD used by the
go-kart simulator.

## Runtime asset

- Entry point: `usd/gnu_campus_drivable.usda`
- ROI: ENU X `-105..325 m`, Y `-215..45 m`
- Coordinates: right-handed ENU, X East, Y North, Z Up
- Unit: metre
- Lighting: authored dome and distant lights
- Collision: terrain, road, and 27 building meshes with separate materials
- Visual content: terrain, 53 road features, sidewalks, and 27 buildings

Component references apply the numerically validated `RotateX(+90 deg)` mapping
from converted GLTF `(E,U,-N)` coordinates to Isaac `(E,N,U)`. The maximum
composed-versus-expected bounding-box error is below `4.8e-14 m`.

## Validation

- `debug/drivable_usd_validation.json`: USD metadata, collision APIs, materials,
  and component bounds; current result passes.
- `debug/static_contact_preflight.json`: 180-step vehicle drop/contact check;
  current result passes with approximately 9.8 cm settling and 1.4 cm horizontal drift.
- Interactive integrated GUI: terrain, roads, buildings, go-kart, and HUD accepted.
- ROS 2 low-speed check: measured 0.5 m/s followed by a confirmed stop.

The expanded terrain remains conditional outside directly measured ground
support. A full-route 60 km/h dynamics test has not yet been completed.

## Distribution

The runtime USD components total approximately 342 MB and include a file larger
than GitHub's normal 100 MB limit. They are intentionally tracked through Git
LFS. Raw PCD data and Stage B/C generation intermediates are not part of the
runtime distribution.
