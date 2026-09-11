#!/usr/bin/env python3
"""Adapt common vehicle commands to the current Isaac Sim steering convention."""

import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from moza_gokart_interfaces.msg import ActuatorCommand
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class IsaacAdapterNode(Node):
    def __init__(self):
        super().__init__("isaac_adapter_node")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("max_speed", 0.5)
        self.declare_parameter("max_steering", math.radians(15.0))
        self.declare_parameter("output_topic", "/ego/drive")
        self.declare_parameter("actuator_output_topic", "/ego/actuator_cmd")
        self.declare_parameter("control_mode", "speed")
        self.declare_parameter("brake_priority_threshold", 0.05)
        self.declare_parameter("failsafe_brake", 0.0)

        rate = float(self.get_parameter("publish_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_steering = float(self.get_parameter("max_steering").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.actuator_output_topic = str(
            self.get_parameter("actuator_output_topic").value
        )
        self.control_mode = str(self.get_parameter("control_mode").value).strip().lower()
        self.brake_priority_threshold = float(
            self.get_parameter("brake_priority_threshold").value
        )
        self.failsafe_brake = float(self.get_parameter("failsafe_brake").value)
        self.parameters_valid = (
            math.isfinite(rate)
            and rate > 0.0
            and math.isfinite(self.command_timeout)
            and self.command_timeout > 0.0
            and math.isfinite(self.max_speed)
            and self.max_speed >= 0.0
            and math.isfinite(self.max_steering)
            and self.max_steering >= 0.0
            and self.output_topic.startswith("/")
            and self.actuator_output_topic.startswith("/")
            and self.control_mode in ("speed", "actuator")
            and math.isfinite(self.brake_priority_threshold)
            and 0.0 <= self.brake_priority_threshold <= 1.0
            and math.isfinite(self.failsafe_brake)
            and 0.0 <= self.failsafe_brake <= 1.0
        )
        self.actuator_mode = self.control_mode == "actuator"
        self.last_command = None
        self.last_command_time = None
        self.speed_publisher = None
        self.actuator_publisher = None
        if self.actuator_mode:
            safe_topic = (
                self.actuator_output_topic
                if self.actuator_output_topic.startswith("/")
                else "/ego/actuator_cmd"
            )
            self.actuator_publisher = self.create_publisher(
                ActuatorCommand, safe_topic, 10
            )
            self.create_subscription(
                ActuatorCommand,
                "/shared_control/actuator_command",
                self.actuator_command_callback,
                10,
            )
        else:
            safe_topic = self.output_topic if self.output_topic.startswith("/") else "/ego/drive"
            self.speed_publisher = self.create_publisher(
                AckermannDriveStamped, safe_topic, 10
            )
            self.create_subscription(
                AckermannDriveStamped,
                "/shared_control/vehicle_command",
                self.speed_command_callback,
                10,
            )
        safe_rate = rate if math.isfinite(rate) and rate > 0.0 else 10.0
        self.timer = self.create_timer(1.0 / safe_rate, self.update)
        if not self.parameters_valid:
            self.get_logger().error("Invalid Isaac adapter parameters; output is locked at stop")
        else:
            self.get_logger().info(f"Isaac adapter longitudinal mode: {self.control_mode}")

    def speed_command_callback(self, message):
        speed = float(message.drive.speed)
        steering = float(message.drive.steering_angle)
        if not math.isfinite(speed) or not math.isfinite(steering):
            self.get_logger().error("Rejected vehicle command containing NaN/Inf")
            self.last_command = None
            self.last_command_time = None
            return
        self.last_command = (speed, steering)
        self.last_command_time = time.monotonic()

    def actuator_command_callback(self, message):
        steering = float(message.steering_angle)
        throttle = float(message.throttle)
        brake = float(message.brake)
        if not all(math.isfinite(value) for value in (steering, throttle, brake)):
            self.get_logger().error("Rejected actuator command containing NaN/Inf")
            self.last_command = None
            self.last_command_time = None
            return
        self.last_command = (steering, throttle, brake)
        self.last_command_time = time.monotonic()

    def _adapt_steering(self, common_steering):
        # This remains the only convention boundary: common left(-)/right(+) -> Isaac.
        common_steering = clamp(common_steering, -self.max_steering, self.max_steering)
        return -common_steering

    def _publish_speed(self, speed, common_steering):
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed = float(clamp(speed, 0.0, self.max_speed))
        message.drive.steering_angle = float(self._adapt_steering(common_steering))
        self.speed_publisher.publish(message)

    def _publish_actuator(self, common_steering, throttle, brake):
        throttle = clamp(throttle, 0.0, 1.0)
        brake = clamp(brake, 0.0, 1.0)
        if brake > self.brake_priority_threshold:
            throttle = 0.0

        message = ActuatorCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.steering_angle = float(self._adapt_steering(common_steering))
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
            self.last_command is not None
            and self.last_command_time is not None
            and time.monotonic() - self.last_command_time <= self.command_timeout
        )
        if not self.parameters_valid or not fresh:
            self._publish_failsafe()
            return
        if self.actuator_mode:
            self._publish_actuator(*self.last_command)
        else:
            self._publish_speed(*self.last_command)

    def publish_stop(self):
        for _ in range(5):
            self._publish_failsafe()
            time.sleep(0.02)


def main(args=None):
    # Keep the ROS context alive until the final fail-safe command is published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = IsaacAdapterNode()
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
