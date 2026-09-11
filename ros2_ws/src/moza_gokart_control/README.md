# moza_gokart_control

This package is the simulator-independent manual/shared-control boundary for the
KAR GoKart. It supports two explicit longitudinal modes without overloading any
`AckermannDriveStamped` field.

- **Speed Control** preserves the verified pipeline and longitudinal PID.
- **Actuator Control** sends normalized throttle and brake independently so
  releasing both pedals coasts instead of commanding a target speed of zero.

The common steering convention remains left-negative/right-positive. Only the
platform adapter changes that convention for the current Isaac vehicle.

## Architecture and topics

```text
MOZA R5
  -> moza_input_node
  -> shared_control_node              # human/Copilot policy boundary
  -> isaac_adapter_node               # Isaac-only sign/topic adaptation
  -> drive_bridge.py
  -> Isaac Sim vehicle
```

| Topic | Type | Meaning |
|---|---|---|
| `/moza/joy_raw` | `sensor_msgs/Joy` | Raw Linux joystick values |
| `/moza/joy` | `sensor_msgs/Joy` | Normalized steering/throttle/brake |
| `/moza/connected` | `std_msgs/Bool` | True only after all required axes arrive |
| `/shared_control/vehicle_command` | `ackermann_msgs/AckermannDriveStamped` | Common target-speed command |
| `/ego/drive` | `ackermann_msgs/AckermannDriveStamped` | Isaac-adapted target-speed command |
| `/shared_control/actuator_command` | `moza_gokart_interfaces/ActuatorCommand` | Common steering/throttle/brake command |
| `/ego/actuator_cmd` | `moza_gokart_interfaces/ActuatorCommand` | Isaac-adapted actuator command |

The normalized MOZA axes are steering `0`, throttle `2`, and brake `5`.
Future Copilot inputs should remain upstream of the platform adapter. A real
vehicle adapter can map the same command to steering CAN `0x105`, APS CAN
`0x205`, and brake CAN `0x206` without changing shared-control algorithms.

## Build

Inside the Isaac/ROS container:

```bash
cd /workspace/autoware_off-road_sim/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select moza_gokart_interfaces moza_gokart_control
source install/setup.bash
```

The workspace must be sourced before starting `launch_sim.py`, because its ROS
bridge subprocess imports the generated `ActuatorCommand` Python type.

## Speed Control (existing behavior)

This mode is the default when `control_mode` is omitted. The original
`moza_r5.yaml`, vehicle USD, and simulator config remain the verified baseline.

Terminal 1:

```bash
cd /workspace/autoware_off-road_sim/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch moza_gokart_control manual_drive.launch.py
```

Terminal 2:

```bash
cd /workspace/autoware_off-road_sim
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
/root/isaacsim/_build/linux-x86_64/release/python.sh scripts/launch_sim.py \
  --config scripts/configs/pennovation_kar_physx.yaml
```

The command path is:

```text
throttle/brake -> target speed -> longitudinal PID -> accelerator/brake
```

Releasing throttle sets target speed to zero, so the PID actively brakes. This
is retained for regression compatibility.

## Actuator Control (new behavior)

Terminal 1:

```bash
cd /workspace/autoware_off-road_sim/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch moza_gokart_control manual_drive.launch.py \
  config:=$(ros2 pkg prefix moza_gokart_control)/share/moza_gokart_control/config/moza_r5_actuator.yaml
```

Terminal 2:

```bash
cd /workspace/autoware_off-road_sim
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
/root/isaacsim/_build/linux-x86_64/release/python.sh scripts/launch_sim.py \
  --config scripts/configs/pennovation_kar_copilot_physx.yaml
```

The command path is:

```text
MOZA throttle -> ActuatorCommand.throttle -> PhysX accelerator writer
MOZA brake    -> ActuatorCommand.brake    -> PhysX brake writer
MOZA steering -> steering angle           -> verified steering normalization/sign path
```

The longitudinal PID remains present in the referenced original USD, but the
Copilot USD layer pre-authors the accelerator/brake writers to direct actuator
constants. `launch_sim.py` only updates those authored values; it does not
disconnect or reconnect the control graph at runtime. The original
speed-control config never loads these overrides. The actuator simulator config
is intended for ROS2/MOZA control; use the original simulator config when
target-speed keyboard control is required.

### Pedal semantics

| Throttle | Brake | PhysX accelerator | PhysX brake | Result |
|---:|---:|---:|---:|---|
| 0.0 | 0.0 | 0.0 | 0.0 | Coast |
| 1.0 | 0.0 | 1.0 | 0.0 | Full throttle |
| 0.0 | 0.5 | 0.0 | 0.5 | Half brake |
| 0.0 | 1.0 | 0.0 | 1.0 | Full brake |

Brake priority is applied independently in shared-control, the Isaac adapter,
and `launch_sim.py`. With the default threshold `0.05`, any brake value above
that threshold forces accelerator to zero.

Do not run `scripts/moza_drive_test.py` at the same time as either pipeline; it
also publishes a vehicle command.

## Actual-speed HUD

The simulator HUD reports physical planar speed, not the target-speed command.
It reads the `IsaacComputeOdometry.outputs:linearVelocity` vector already used
by the vehicle odometry graph and displays `hypot(vx, vy)`. This remains correct
for vehicles spawned at a non-zero yaw and corresponds to the speed magnitude
computed from `/ego/odom.twist.twist.linear`.

Actuator mode displays, for example:

```text
CONTROL: ROS2 / ACTUATOR
Speed:    12.34 m/s
          44.4 km/h
Throttle: 0.60
Brake:    0.00
Steer:    +3.2 deg
```

Speed mode uses the same physical odometry source and is labelled
`CONTROL: ROS2 / SPEED`. The existing `/ego/odom` publisher is unchanged.

## Vehicle dynamics logger

Start the logger in a third terminal after sourcing the ROS workspace:

```bash
cd /workspace/autoware_off-road_sim/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone_hil.xml
export ROS_DOMAIN_ID=0
ros2 run moza_gokart_control dynamics_logger_node
```

The logger subscribes to `/ego/odom` and `/ego/actuator_cmd`. It writes one CSV
row per odometry sample. By default, CSV files are stored outside the repository
under `~/.ros/moza_gokart_dynamics/`. To save under the repository's already
ignored `data/` directory instead:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclone_hil.xml
export ROS_DOMAIN_ID=0
ros2 run moza_gokart_control dynamics_logger_node --ros-args \
  -p output_directory:=/workspace/autoware_off-road_sim/data/dynamics
```

The DDS settings must match `pennovation_kar_copilot_physx.yaml`. The URI file
is created by `launch_sim.py`, so start the simulator before the logger. If the
simulator config uses a different domain ID or network interface, use the same
values in the logger terminal.

CSV columns are:

```text
ros_timestamp_s
logger_elapsed_s
run_elapsed_s
actual_speed_m_s
actual_speed_km_h
longitudinal_acceleration_m_s2
throttle
brake
steering_angle_rad
steering_angle_deg
run_event
run_peak_speed_km_h
```

`ros_timestamp_s` comes from the odometry message header.
`logger_elapsed_s` and `run_elapsed_s` use the local monotonic clock and are not
affected by ROS clock resets. Actual speed is the planar magnitude
`hypot(linear.x, linear.y)`. Longitudinal acceleration is the finite difference
of consecutive actual-speed samples divided by their ROS timestamp interval;
if ROS time does not advance, the monotonic callback interval is used.

Each logger process measures one acceleration run. It arms after observing
physical speed at or below `stationary_speed_m_s` (default `0.20 m/s`) and sets
run time zero on the first odometry sample where throttle is at least
`run_start_throttle` (default `0.10`) and brake is at most `0.05`. The first
crossings of 20, 40, 60, and 80 km/h are printed and recorded through
`run_event` and `run_elapsed_s`; peak speed is tracked without a clamp in
`run_peak_speed_km_h`. Stop the logger with
Ctrl-C after the run to print the complete milestone and peak-speed summary.

Because every row contains physical speed, acceleration, throttle, and brake,
the same CSV can be filtered later to compare coast (`throttle=0, brake=0`) and
braking at values such as `0.10`, `0.20`, and `0.50`. The logger only measures;
it does not publish commands or alter vehicle dynamics.

## Safety and timeout policy

The actuator pipeline has three stale-command boundaries:

1. `shared_control_node`: MOZA disconnect, invalid Joy data, or a 0.25 s Joy
   timeout publishes throttle zero.
2. `isaac_adapter_node`: a 0.25 s shared-command timeout publishes throttle
   zero.
3. `launch_sim.py`: a 0.25 s `/ego/actuator_cmd` timeout writes accelerator
   zero directly to the PhysX command path.

Normal node shutdown publishes five zero-throttle commands. If a ROS process is
killed before those messages can be delivered, the independent downstream
timeout still clears stale throttle.

The simulation fail-safe brake is deliberately `0.0`: timeout removes drive
torque and allows coast, avoiding an unexpected full-brake event during a
research run. A real-vehicle adapter must choose a separately validated policy,
for example a controlled brake value or a supervisory emergency-stop path. It
must not silently inherit the simulation value.

## High-speed vehicle model

| Role | Vehicle asset | Simulator config |
|---|---|---|
| Verified baseline | `assets/vehicles/kar_gokart_isaac_physx.usd` | `scripts/configs/pennovation_kar_physx.yaml` |
| Copilot actuator/high-speed | `assets/vehicles/kar_gokart_copilot_physx.usda` | `scripts/configs/pennovation_kar_copilot_physx.yaml` |

The Copilot asset is a small USD reference layer; it does not copy or modify the
verified vehicle. It overrides:

- engine `peakTorque`: `5.340567 -> 40.0 Nm`
- `highForwardSpeedSubStepCount`: `1 -> 2`
- deterministic `ActuatorAccelerator` and `ActuatorBrake` inputs connected
  directly to the existing PhysX attribute writers
- `ActuatorSteering` connected to the inherited
  `divide -> clamp -> negate -> steer writer` normalization/sign path

Mass, 6000 RPM limit, gear ratio, final ratio, wheel radius, tire model, steering
normalization/sign nodes, sensors, and geometry remain inherited. The unchanged
gearing has a no-slip theoretical limit near 146 km/h, so RPM and gear ratio
were not changed for the 100 km/h target.

With 198 kg mass, 2.54545 final ratio, and 0.165 m wheel radius, 40 Nm produces
an ideal peak tractive acceleration near 3.1 m/s². Applying the existing torque
curve's approximate average factor gives about 2.8 m/s² before drivetrain loss
and slip, making it a reasonable initial value for a 0-100 km/h target near 10
seconds. The actual result must be measured from `/ego/odom`.

The new model keeps 60 Hz physics and increases only high-speed vehicle
substeps. Raise physics frequency or solver iterations only if the 100 km/h
runtime test shows contact, slip, or chassis instability.

## Required runtime validation

Use a flat, sufficiently long straight and record at least:

```text
/ego/actuator_cmd
/ego/odom
/moza/connected
```

Verify coast with throttle/brake both zero, brake priority, 0-100 km/h time,
settled maximum speed, braking distance, steering stability, wheel slip, and
real-time factor. Isaac Sim GUI and physical MOZA hardware validation are not
part of the static/build checks in this repository.
