#!/usr/bin/env python3
"""ROS2 drive command bridge for Isaac Sim.

Isaac Sim runs Python 3.12 but rclpy is compiled for Python 3.10 on ROS Humble.
This script runs as a Python 3.10 subprocess to handle ROS2 drive subscriptions
and optional map / static-TF publishing — identical pattern to gnss_bridge.py.

stdin protocol (one command per line):
  Before 'start':
    sub<TAB>veh_name<TAB>drive_topic<TAB>control_topic
    act_sub<TAB>veh_name<TAB>actuator_topic
  After 'start' (late commands, e.g. map data ready after physics warmup):
    map<TAB>width<TAB>height<TAB>resolution<TAB>orig_x<TAB>orig_y<TAB>data_b64
    tf<TAB>parent_frame<TAB>child_frame
    ctrl_mode<TAB>veh_name<TAB>value  -- publish control mode (0=KEYBOARD_CONTROL, 1=ROS2_CONTROL)

stdout protocol:
  ready                                           -- node is up and subscriptions registered
  cmd<TAB>veh_name<TAB>speed<TAB>steer<TAB>source -- per received drive command
  act<TAB>veh_name<TAB>steer<TAB>throttle<TAB>brake -- per actuator command

Exits cleanly when stdin is closed (parent process exits).
"""

import sys
import os
import signal
import struct
import base64
import threading

from ros2_speed_limit import clamp_speed, parse_speed_limit

# Ignore SIGINT — the parent (Isaac Sim) owns the shutdown sequence.
signal.signal(signal.SIGINT, signal.SIG_IGN)

try:
    _max_speed_m_s = parse_speed_limit(os.environ.get("ISAACSIM_ROS2_MAX_SPEED_M_S"))
except (TypeError, ValueError) as _speed_limit_error:
    raise SystemExit(f"[drive_bridge] Invalid ROS 2 speed limit: {_speed_limit_error}")

import rclpy
from rclpy.node import Node

rclpy.init()
_node = Node("isaacsim_drive_bridge")

# Keep the tf2 buffer to 2 s (default is 10 s).
# Must stay below the set_current_time offset used on restart (3.0 s in
# launch_sim.py) so that when the sim clock jumps forward by 3 s all old TF
# entries (at most 2 s old) are immediately evicted, giving Rviz clean TF
# data from the very first physics tick with no TF_OLD_DATA blackout.
try:
    from tf2_ros import Buffer as _TF2Buffer, TransformListener as _TFListener
    from rclpy.duration import Duration as _Duration
    _tf_buffer   = _TF2Buffer(cache_time=_Duration(seconds=2.0))
    _tf_listener = _TFListener(_tf_buffer, _node)
except Exception as _e:
    sys.stderr.write(f"[drive_bridge] tf2 buffer init skipped: {_e}\n")

_pending_subs = []   # list of (veh_name, drive_topic, control_topic)
_pending_actuator_subs = []  # list of (veh_name, actuator_topic)
_pending_maps = []   # list of raw parts [width, height, res, ox, oy, b64]
_pending_tfs  = []   # list of (parent_frame, child_frame)
_ctrl_mode_pubs = {}  # veh_name -> rclpy Publisher<Int32>

# ── Phase 1: read registrations from stdin until "start" ──────────────────────
for _line in sys.stdin:
    _line = _line.rstrip("\n")
    if _line == "start":
        break
    _parts = _line.split("\t")
    if not _parts:
        continue
    _cmd = _parts[0]
    if _cmd == "sub" and len(_parts) == 4:
        _pending_subs.append((_parts[1], _parts[2], _parts[3]))
    elif _cmd == "act_sub" and len(_parts) == 3:
        _pending_actuator_subs.append((_parts[1], _parts[2]))
    elif _cmd == "map" and len(_parts) == 7:
        _pending_maps.append(_parts[1:])
    elif _cmd == "tf" and len(_parts) == 3:
        _pending_tfs.append((_parts[1], _parts[2]))

# ── Phase 2: register drive subscriptions ─────────────────────────────────────
def _pub_cmd(veh_name, speed, steer, source):
    try:
        speed = clamp_speed(speed, _max_speed_m_s)
    except (TypeError, ValueError) as error:
        sys.stderr.write(f"[drive_bridge] Rejected {source} speed for {veh_name}: {error}\n")
        return None
    sys.stdout.write(f"cmd\t{veh_name}\t{speed}\t{steer}\t{source}\n")
    sys.stdout.flush()
    return speed


def _pub_actuator(veh_name, steer, throttle, brake):
    sys.stdout.write(f"act\t{veh_name}\t{steer}\t{throttle}\t{brake}\n")
    sys.stdout.flush()

for _vn, _dt, _ct in _pending_subs:
    try:
        from ackermann_msgs.msg import AckermannDriveStamped
        def _make_ack_cb(v):
            def _cb(msg):
                _pub_cmd(v, msg.drive.speed, msg.drive.steering_angle, "ackermann")
            return _cb
        _node.create_subscription(AckermannDriveStamped, _dt, _make_ack_cb(_vn), 10)
        sys.stderr.write(f"[drive_bridge] {_vn}: subscribed '{_dt}' [AckermannDriveStamped]\n")
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] {_vn}: AckermannDriveStamped failed: {_e}\n")
    try:
        from autoware_control_msgs.msg import Control
        from ackermann_msgs.msg import AckermannDriveStamped
        _ctrl_pub = _node.create_publisher(AckermannDriveStamped, _dt, 10)
        def _make_ctrl_cb(v, pub, dt):
            def _cb(msg):
                spd = _pub_cmd(
                    v,
                    msg.longitudinal.velocity,
                    msg.lateral.steering_tire_angle,
                    "autoware",
                )
                if spd is None:
                    return
                steer = msg.lateral.steering_tire_angle
                # Republish as AckermannDriveStamped so the built-in OmniGraph
                # subscriber on the drive topic receives the command directly.
                ack = AckermannDriveStamped()
                ack.header.stamp = _node.get_clock().now().to_msg()
                ack.drive.speed = float(spd)
                ack.drive.steering_angle = float(steer)
                pub.publish(ack)
            return _cb
        _node.create_subscription(Control, _ct, _make_ctrl_cb(_vn, _ctrl_pub, _dt), 10)
        sys.stderr.write(f"[drive_bridge] {_vn}: subscribed '{_ct}' [autoware_control_msgs/Control] → republish on '{_dt}'\n")
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] {_vn}: autoware_control_msgs/Control failed: {_e}\n")

for _vn, _at in _pending_actuator_subs:
    try:
        from moza_gokart_interfaces.msg import ActuatorCommand

        def _make_actuator_cb(v):
            def _cb(msg):
                _pub_actuator(v, msg.steering_angle, msg.throttle, msg.brake)
            return _cb

        _node.create_subscription(ActuatorCommand, _at, _make_actuator_cb(_vn), 10)
        sys.stderr.write(f"[drive_bridge] {_vn}: subscribed '{_at}' [ActuatorCommand]\n")
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] {_vn}: ActuatorCommand failed: {_e}\n")

_control_mode_topics = {}
for _vn, _dt, _ct in _pending_subs:
    _control_mode_topics[_vn] = _dt.rsplit("/", 1)[0] + "/control_mode"
for _vn, _at in _pending_actuator_subs:
    _control_mode_topics[_vn] = _at.rsplit("/", 1)[0] + "/control_mode"
for _vn, _cm_topic in _control_mode_topics.items():
    try:
        from std_msgs.msg import Int32 as _Int32
        _ctrl_mode_pubs[_vn] = _node.create_publisher(_Int32, _cm_topic, 10)
        sys.stderr.write(f"[drive_bridge] {_vn}: control_mode publisher on '{_cm_topic}'\n")
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] {_vn}: control_mode publisher failed: {_e}\n")

if _max_speed_m_s is not None:
    sys.stderr.write(
        f"[drive_bridge] ROS 2 target-speed limit: {_max_speed_m_s:.6f} m/s "
        f"({_max_speed_m_s * 3.6:.1f} km/h)\n"
    )

# ── Phase 3: start spinning in background thread ───────────────────────────────
_spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
_spin_thread.start()

# ── Phase 4: signal readiness to parent ───────────────────────────────────────
sys.stdout.write("ready\n")
sys.stdout.flush()


# ── Helper: publish /map and static TF ────────────────────────────────────────
def _publish_map(parts):
    try:
        import struct as _s, base64 as _b
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
        _w, _h, _res, _ox, _oy, _db64 = parts
        _w, _h = int(_w), int(_h)
        _res, _ox, _oy = float(_res), float(_ox), float(_oy)
        _raw  = _b.b64decode(_db64)
        _data = list(_s.unpack(f"{len(_raw)}b", _raw))
        _qos  = QoSProfile(depth=1,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST)
        _pub  = _node.create_publisher(OccupancyGrid, "/map", _qos)
        _msg  = OccupancyGrid()
        _msg.header.frame_id            = "map"
        _msg.header.stamp               = _node.get_clock().now().to_msg()
        _msg.info.width                 = _w
        _msg.info.height                = _h
        _msg.info.resolution            = _res
        _msg.info.origin.position.x     = _ox
        _msg.info.origin.position.y     = _oy
        _msg.info.origin.orientation.w  = 1.0
        _msg.data                       = _data
        _pub.publish(_msg)
        sys.stderr.write(f"[drive_bridge] Published /map ({_w}×{_h} @ {_res} m/cell)\n")
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] map publish failed: {_e}\n")


_stf_broadcaster = None  # created lazily on first tf command

def _publish_tf(parent, child):
    global _stf_broadcaster
    try:
        from tf2_ros import StaticTransformBroadcaster
        from geometry_msgs.msg import TransformStamped
        if _stf_broadcaster is None:
            _stf_broadcaster = StaticTransformBroadcaster(_node)
        _t = TransformStamped()
        _t.header.stamp     = _node.get_clock().now().to_msg()
        _t.header.frame_id  = parent
        _t.child_frame_id   = child
        _t.transform.rotation.w = 1.0
        _stf_broadcaster.sendTransform([_t])
        sys.stderr.write(f"[drive_bridge] Published static TF: {parent} → {child}\n")
    except Exception as _e:
        sys.stderr.write(f"[drive_bridge] TF broadcast failed: {_e}\n")


# Process any map/TF that arrived before start
for _mp in _pending_maps:
    _publish_map(_mp)
for _parent, _child in _pending_tfs:
    _publish_tf(_parent, _child)


# ── Phase 5: keep alive, handle late stdin commands ───────────────────────────
try:
    while True:
        _line = sys.stdin.readline()
        if not _line:
            break
        _line = _line.rstrip("\n")
        if not _line:
            continue
        _parts = _line.split("\t")
        _cmd = _parts[0] if _parts else ""
        if _cmd == "map" and len(_parts) == 7:
            _publish_map(_parts[1:])
        elif _cmd == "tf" and len(_parts) == 3:
            _publish_tf(_parts[1], _parts[2])
        elif _cmd == "ctrl_mode" and len(_parts) == 3:
            try:
                from std_msgs.msg import Int32 as _Int32
                _cmvn, _cmval = _parts[1], int(_parts[2])
                if _cmvn in _ctrl_mode_pubs:
                    _cm_msg = _Int32()
                    _cm_msg.data = _cmval
                    _ctrl_mode_pubs[_cmvn].publish(_cm_msg)
            except Exception as _e:
                sys.stderr.write(f"[drive_bridge] ctrl_mode publish failed: {_e}\n")
except Exception:
    pass

try:
    rclpy.shutdown()
except Exception:
    pass
try:
    _spin_thread.join(timeout=2.0)
except Exception:
    pass
