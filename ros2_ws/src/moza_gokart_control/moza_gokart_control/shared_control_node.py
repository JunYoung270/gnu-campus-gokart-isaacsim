#!/usr/bin/env python3
"""Fail-safe human command layer and future AI Copilot integration boundary."""

import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from moza_gokart_interfaces.msg import ActuatorCommand
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class SharedControlNode(Node):
    def __init__(self):
        super().__init__("shared_control_node")
        self.declare_parameter("steering_axis", 0)
        self.declare_parameter("throttle_axis", 2)
        self.declare_parameter("brake_axis", 5)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("max_speed", 0.5)
        self.declare_parameter("max_steering", math.radians(15.0))
        self.declare_parameter("control_mode", "speed")
        self.declare_parameter("brake_priority_threshold", 0.05)
        self.declare_parameter("failsafe_brake", 0.0)

        self.steering_axis = int(self.get_parameter("steering_axis").value)
        self.throttle_axis = int(self.get_parameter("throttle_axis").value)
        self.brake_axis = int(self.get_parameter("brake_axis").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_steering = float(self.get_parameter("max_steering").value)
        self.control_mode = str(self.get_parameter("control_mode").value).strip().lower()
        self.brake_priority_threshold = float(
            self.get_parameter("brake_priority_threshold").value
        )
        self.failsafe_brake = float(self.get_parameter("failsafe_brake").value)
        self.parameters_valid = self._validate_parameters(rate)
        self.actuator_mode = self.control_mode == "actuator"

        self.connected = False
        self.last_joy = None
        self.last_joy_time = None
        self.speed_publisher = None
        self.actuator_publisher = None
        if self.actuator_mode:
            self.actuator_publisher = self.create_publisher(
                ActuatorCommand, "/shared_control/actuator_command", 10
            )
        else:
            self.speed_publisher = self.create_publisher(
                AckermannDriveStamped, "/shared_control/vehicle_command", 10
            )
        self.create_subscription(Joy, "/moza/joy", self.joy_callback, 10)
        self.create_subscription(Bool, "/moza/connected", self.connected_callback, 10)
        safe_rate = rate if math.isfinite(rate) and rate > 0.0 else 10.0
        self.timer = self.create_timer(1.0 / safe_rate, self.update)
        if not self.parameters_valid:
            self.get_logger().error("Invalid shared-control parameters; output is locked at stop")
        else:
            self.get_logger().info(
                f"Shared-control longitudinal mode: {self.control_mode}"
            )

    def _validate_parameters(self, rate):
        return (
            min(self.steering_axis, self.throttle_axis, self.brake_axis) >= 0
            and math.isfinite(rate)
            and rate > 0.0
            and math.isfinite(self.command_timeout)
            and self.command_timeout > 0.0
            and math.isfinite(self.max_speed)
            and self.max_speed >= 0.0
            and math.isfinite(self.max_steering)
            and self.max_steering >= 0.0
            and self.control_mode in ("speed", "actuator")
            and math.isfinite(self.brake_priority_threshold)
            and 0.0 <= self.brake_priority_threshold <= 1.0
            and math.isfinite(self.failsafe_brake)
            and 0.0 <= self.failsafe_brake <= 1.0
        )

    def joy_callback(self, message):
        if not self.parameters_valid:
            return
        required = max(self.steering_axis, self.throttle_axis, self.brake_axis)
        if len(message.axes) <= required:
            self.get_logger().error("Rejected Joy message with missing required axes")
            self.last_joy = None
            return
        values = (
            message.axes[self.steering_axis],
            message.axes[self.throttle_axis],
            message.axes[self.brake_axis],
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("Rejected Joy message containing NaN/Inf")
            self.last_joy = None
            return
        self.last_joy = values
        self.last_joy_time = time.monotonic()

    def connected_callback(self, message):
        self.connected = bool(message.data)
        if not self.connected:
            self.last_joy = None
            self.last_joy_time = None

    def _publish_speed(self, speed, steering):
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed = float(clamp(speed, 0.0, self.max_speed))
        message.drive.steering_angle = float(
            clamp(steering, -self.max_steering, self.max_steering)
        )
        self.speed_publisher.publish(message)

    def _publish_actuator(self, steering, throttle, brake):
        steering = clamp(steering, -self.max_steering, self.max_steering)
        throttle = clamp(throttle, 0.0, 1.0)
        brake = clamp(brake, 0.0, 1.0)
        if brake > self.brake_priority_threshold:
            throttle = 0.0

        message = ActuatorCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.steering_angle = float(steering)
        message.throttle = float(throttle)
        message.brake = float(brake)
        self.actuator_publisher.publish(message)

    def _publish_failsafe(self):
        if self.actuator_mode:
            self._publish_actuator(0.0, 0.0, self.failsafe_brake)
        else:
            self._publish_speed(0.0, 0.0)

    def update(self):
        fresh = (
            self.last_joy is not None
            and self.last_joy_time is not None
            and time.monotonic() - self.last_joy_time <= self.command_timeout
        )
        if not self.parameters_valid or not self.connected or not fresh:
            self._publish_failsafe()
            return

        steering, throttle, brake = self.last_joy
        steering = clamp(steering, -1.0, 1.0)
        throttle = clamp(throttle, 0.0, 1.0)
        brake = clamp(brake, 0.0, 1.0)
        if self.actuator_mode:
            self._publish_actuator(steering * self.max_steering, throttle, brake)
        else:
            speed = self.max_speed * throttle * (1.0 - brake)
            self._publish_speed(speed, steering * self.max_steering)

    def publish_stop(self):
        for _ in range(5):
            self._publish_failsafe()
            time.sleep(0.02)


def main(args=None):
    # Keep the ROS context alive until the final fail-safe command is published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SharedControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
