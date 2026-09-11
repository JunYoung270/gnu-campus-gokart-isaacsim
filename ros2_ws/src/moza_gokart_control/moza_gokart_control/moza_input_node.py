#!/usr/bin/env python3
"""Read Linux joystick events and publish raw and normalized MOZA state."""

import math
import os
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
EVENT_FORMAT = "IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class MozaInputNode(Node):
    def __init__(self):
        super().__init__("moza_input_node")
        self.declare_parameter("device_path", "/dev/input/moza-js")
        self.declare_parameter("steering_axis", 0)
        self.declare_parameter("steering_min", -32767)
        self.declare_parameter("steering_center", -144)
        self.declare_parameter("steering_max", 32767)
        self.declare_parameter("steering_deadzone", 300)
        self.declare_parameter("throttle_axis", 2)
        self.declare_parameter("throttle_min", -32767)
        self.declare_parameter("throttle_max", 32767)
        self.declare_parameter("brake_axis", 5)
        self.declare_parameter("brake_min", -32767)
        self.declare_parameter("brake_max", 32767)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("axis_count", 8)
        self.declare_parameter("button_count", 32)
        self.declare_parameter("reconnect_interval", 1.0)

        self.device_path = str(self.get_parameter("device_path").value)
        self.steering_axis = int(self.get_parameter("steering_axis").value)
        self.steering_min = int(self.get_parameter("steering_min").value)
        self.steering_center = int(self.get_parameter("steering_center").value)
        self.steering_max = int(self.get_parameter("steering_max").value)
        self.steering_deadzone = int(self.get_parameter("steering_deadzone").value)
        self.throttle_axis = int(self.get_parameter("throttle_axis").value)
        self.throttle_min = int(self.get_parameter("throttle_min").value)
        self.throttle_max = int(self.get_parameter("throttle_max").value)
        self.brake_axis = int(self.get_parameter("brake_axis").value)
        self.brake_min = int(self.get_parameter("brake_min").value)
        self.brake_max = int(self.get_parameter("brake_max").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.reconnect_interval = float(self.get_parameter("reconnect_interval").value)

        requested_axes = max(0, int(self.get_parameter("axis_count").value))
        requested_buttons = max(0, int(self.get_parameter("button_count").value))
        configured_axes = (self.steering_axis, self.throttle_axis, self.brake_axis)
        required_axis_count = max((axis for axis in configured_axes if axis >= 0), default=0) + 1
        self.axes = [0] * max(1, requested_axes, required_axis_count)
        self.buttons = [0] * requested_buttons
        self.seen_required_axes = set()
        self.required_axes = set(configured_axes)
        if self.steering_axis >= 0:
            self.axes[self.steering_axis] = self.steering_center
        if self.throttle_axis >= 0:
            self.axes[self.throttle_axis] = self.throttle_min
        if self.brake_axis >= 0:
            self.axes[self.brake_axis] = self.brake_min

        self.parameters_valid = self._validate_parameters(rate)
        self.fd = None
        self.connected = False
        self.last_open_attempt = 0.0

        self.raw_publisher = self.create_publisher(Joy, "/moza/joy_raw", 10)
        self.joy_publisher = self.create_publisher(Joy, "/moza/joy", 10)
        self.connected_publisher = self.create_publisher(Bool, "/moza/connected", 10)
        safe_rate = rate if math.isfinite(rate) and rate > 0.0 else 10.0
        self.timer = self.create_timer(1.0 / safe_rate, self.update)

        if self.parameters_valid:
            self._try_open()
        else:
            self.get_logger().error("Invalid MOZA parameters; device will remain disconnected")

    def _validate_parameters(self, rate):
        valid = True
        if not self.device_path:
            valid = False
        if min(self.required_axes) < 0:
            valid = False
        if not self.steering_min < self.steering_center < self.steering_max:
            valid = False
        if self.steering_deadzone < 0 or self.steering_deadzone >= min(
            self.steering_center - self.steering_min,
            self.steering_max - self.steering_center,
        ):
            valid = False
        if not self.throttle_min < self.throttle_max:
            valid = False
        if not self.brake_min < self.brake_max:
            valid = False
        if not math.isfinite(rate) or rate <= 0.0:
            valid = False
        if not math.isfinite(self.reconnect_interval) or self.reconnect_interval <= 0.0:
            valid = False
        return valid

    def _try_open(self):
        now = time.monotonic()
        if now - self.last_open_attempt < self.reconnect_interval:
            return
        self.last_open_attempt = now
        try:
            self.fd = os.open(self.device_path, os.O_RDONLY | os.O_NONBLOCK)
            self.seen_required_axes.clear()
            self.connected = False
            self.get_logger().info(f"Opened MOZA joystick: {self.device_path}")
        except OSError as error:
            self.fd = None
            self.connected = False
            self.get_logger().warning(f"Cannot open {self.device_path}: {error}")

    def _disconnect(self, reason):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None
        self.connected = False
        self.seen_required_axes.clear()
        self.get_logger().error(reason)

    def _read_events(self):
        if self.fd is None:
            return
        while True:
            try:
                data = os.read(self.fd, EVENT_SIZE)
                if not data:
                    self._disconnect("MOZA device disconnected (EOF)")
                    return
                if len(data) != EVENT_SIZE:
                    self._disconnect("MOZA device returned a partial joystick event")
                    return
                _, value, event_type, number = struct.unpack(EVENT_FORMAT, data)
                event_type &= ~JS_EVENT_INIT
                if event_type == JS_EVENT_AXIS:
                    if number >= len(self.axes):
                        self.axes.extend([0] * (number + 1 - len(self.axes)))
                    self.axes[number] = int(value)
                    if number in self.required_axes:
                        self.seen_required_axes.add(number)
                elif event_type == JS_EVENT_BUTTON:
                    if number >= len(self.buttons):
                        self.buttons.extend([0] * (number + 1 - len(self.buttons)))
                    self.buttons[number] = 1 if value else 0
            except BlockingIOError:
                break
            except OSError as error:
                self._disconnect(f"MOZA device read failed: {error}")
                return
        self.connected = self.required_axes.issubset(self.seen_required_axes)

    def _normalize_steering(self, raw):
        if abs(raw - self.steering_center) <= self.steering_deadzone:
            return 0.0
        if raw < self.steering_center:
            value = (raw - self.steering_center) / (self.steering_center - self.steering_min)
        else:
            value = (raw - self.steering_center) / (self.steering_max - self.steering_center)
        return clamp(value, -1.0, 1.0)

    @staticmethod
    def _normalize_pedal(raw, raw_min, raw_max):
        return clamp((raw - raw_min) / (raw_max - raw_min), 0.0, 1.0)

    def update(self):
        if self.fd is None and self.parameters_valid:
            self._try_open()
        self._read_events()

        stamp = self.get_clock().now().to_msg()
        raw = Joy()
        raw.header.stamp = stamp
        raw.header.frame_id = "moza_r5"
        raw.axes = [float(value) for value in self.axes]
        raw.buttons = list(self.buttons)

        normalized = Joy()
        normalized.header.stamp = stamp
        normalized.header.frame_id = "moza_r5"
        normalized.axes = [0.0] * len(self.axes)
        normalized.buttons = list(self.buttons)
        if self.parameters_valid:
            normalized.axes[self.steering_axis] = self._normalize_steering(
                self.axes[self.steering_axis]
            )
            normalized.axes[self.throttle_axis] = self._normalize_pedal(
                self.axes[self.throttle_axis], self.throttle_min, self.throttle_max
            )
            normalized.axes[self.brake_axis] = self._normalize_pedal(
                self.axes[self.brake_axis], self.brake_min, self.brake_max
            )

        self.raw_publisher.publish(raw)
        self.joy_publisher.publish(normalized)
        connected = Bool()
        connected.data = bool(self.connected and self.parameters_valid)
        self.connected_publisher.publish(connected)

    def close(self):
        disconnected = Bool()
        disconnected.data = False
        for _ in range(3):
            self.connected_publisher.publish(disconnected)
            time.sleep(0.02)
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None


def main(args=None):
    # Keep the ROS context alive until the final fail-safe command is published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MozaInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
