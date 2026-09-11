#!/usr/bin/env python3

import os
import sys
import time
import struct
import argparse

# ROS2 DDS environment must match Isaac Sim drive_bridge.
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
os.environ.setdefault("ROS_DOMAIN_ID", "0")

if os.path.exists("/tmp/cyclone_hil.xml"):
    os.environ.setdefault(
        "CYCLONEDDS_URI",
        "file:///tmp/cyclone_hil.xml"
    )

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


DEVICE = "/dev/input/moza-js"

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

EVENT_FORMAT = "IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

# Measured MOZA R5 calibration
STEER_AXIS = 0
THROTTLE_AXIS = 2
BRAKE_AXIS = 5

STEER_MIN = -32767
STEER_CENTER = -144
STEER_MAX = 32767

PEDAL_MIN = -32767
PEDAL_MAX = 32767

# Small steering-center deadzone
STEER_DEADZONE = 300


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def normalize_steering(raw):
    """Return steering input in [-1.0, +1.0]."""

    if abs(raw - STEER_CENTER) <= STEER_DEADZONE:
        return 0.0

    if raw < STEER_CENTER:
        value = (raw - STEER_CENTER) / (STEER_CENTER - STEER_MIN)
    else:
        value = (raw - STEER_CENTER) / (STEER_MAX - STEER_CENTER)

    return clamp(value, -1.0, 1.0)


def normalize_pedal(raw):
    """Released=-32767 -> 0.0, fully pressed=32767 -> 1.0."""

    value = (raw - PEDAL_MIN) / (PEDAL_MAX - PEDAL_MIN)
    return clamp(value, 0.0, 1.0)


class MozaDriveNode(Node):

    def __init__(self, max_speed, max_steer_deg):
        super().__init__("moza_drive_test")

        self.max_speed = float(max_speed)
        self.max_steer_rad = float(max_steer_deg) * 3.141592653589793 / 180.0

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            "/ego/drive",
            10
        )

        self.axes = {
            STEER_AXIS: STEER_CENTER,
            THROTTLE_AXIS: PEDAL_MIN,
            BRAKE_AXIS: PEDAL_MIN,
        }

        try:
            self.fd = os.open(
                DEVICE,
                os.O_RDONLY | os.O_NONBLOCK
            )
        except OSError as e:
            self.get_logger().error(
                f"Cannot open {DEVICE}: {e}"
            )
            raise

        self.disconnected = False
        self.print_counter = 0

        # 50 Hz
        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info("MOZA R5 drive test started")
        self.get_logger().info(f"Device: {DEVICE}")
        self.get_logger().info(
            f"Max speed: {self.max_speed:.2f} m/s"
        )
        self.get_logger().info(
            f"Max tire steer: +/-{max_steer_deg:.1f} deg"
        )
        self.get_logger().info(
            "AXIS 0=steering, AXIS 2=throttle, AXIS 5=brake"
        )

    def read_events(self):
        while True:
            try:
                data = os.read(self.fd, EVENT_SIZE)

                if len(data) == 0:
                    self.disconnected = True
                    return

                if len(data) != EVENT_SIZE:
                    return

                _, value, event_type, number = struct.unpack(
                    EVENT_FORMAT,
                    data
                )

                event_type &= ~JS_EVENT_INIT

                if event_type == JS_EVENT_AXIS:
                    self.axes[number] = value

            except BlockingIOError:
                return

            except OSError as e:
                self.get_logger().error(
                    f"MOZA device read error: {e}"
                )
                self.disconnected = True
                return

    def publish_stop(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.publisher.publish(msg)

    def update(self):
        self.read_events()

        if self.disconnected:
            self.publish_stop()
            return

        steer_raw = self.axes.get(STEER_AXIS, STEER_CENTER)
        throttle_raw = self.axes.get(THROTTLE_AXIS, PEDAL_MIN)
        brake_raw = self.axes.get(BRAKE_AXIS, PEDAL_MIN)

        steering = normalize_steering(steer_raw)
        throttle = normalize_pedal(throttle_raw)
        brake = normalize_pedal(brake_raw)

        steering_angle = -steering * self.max_steer_rad

        # First test:
        # throttle determines target speed.
        # brake always overrides/reduces the speed command.
        speed = self.max_speed * throttle * (1.0 - brake)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(speed)

        self.publisher.publish(msg)

        # Show status at about 5 Hz
        self.print_counter += 1

        if self.print_counter >= 10:
            self.print_counter = 0

            print(
                f"\r"
                f"STEER {steering:+.3f} "
                f"({steering_angle * 180.0 / 3.141592653589793:+5.1f} deg) | "
                f"THR {throttle:.2f} | "
                f"BRK {brake:.2f} | "
                f"SPEED {speed:.2f} m/s",
                end="",
                flush=True
            )

    def safe_shutdown(self):
        print()

        # Send several stop commands before closing.
        for _ in range(5):
            self.publish_stop()
            time.sleep(0.02)

        try:
            os.close(self.fd)
        except Exception:
            pass

        self.get_logger().info("STOP command sent. MOZA node closed.")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--max-speed",
        type=float,
        default=0.5
    )

    parser.add_argument(
        "--max-steer-deg",
        type=float,
        default=15.0
    )

    args = parser.parse_args()

    rclpy.init()

    node = MozaDriveNode(
        max_speed=args.max_speed,
        max_steer_deg=args.max_steer_deg
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.safe_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
