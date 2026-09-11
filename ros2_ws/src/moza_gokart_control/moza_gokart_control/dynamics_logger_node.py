#!/usr/bin/env python3
"""Record physical odometry and actuator commands for one dynamics run."""

import csv
import math
import os
import time
from datetime import datetime, timezone

import rclpy
from moza_gokart_interfaces.msg import ActuatorCommand
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


KMH_PER_MPS = 3.6


class DynamicsLoggerNode(Node):
    """Write synchronized odometry/actuator samples and acceleration milestones."""

    def __init__(self):
        super().__init__("dynamics_logger_node")
        self.declare_parameter("odom_topic", "/ego/odom")
        self.declare_parameter("actuator_topic", "/ego/actuator_cmd")
        self.declare_parameter(
            "output_directory", "~/.ros/moza_gokart_dynamics"
        )
        self.declare_parameter("file_prefix", "dynamics_run")
        self.declare_parameter("run_start_throttle", 0.10)
        self.declare_parameter("stationary_speed_m_s", 0.20)
        self.declare_parameter("start_brake_threshold", 0.05)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.actuator_topic = str(self.get_parameter("actuator_topic").value)
        output_directory = os.path.abspath(
            os.path.expanduser(
                str(self.get_parameter("output_directory").value)
            )
        )
        file_prefix = str(self.get_parameter("file_prefix").value).strip()
        self.run_start_throttle = float(
            self.get_parameter("run_start_throttle").value
        )
        self.stationary_speed_m_s = float(
            self.get_parameter("stationary_speed_m_s").value
        )
        self.start_brake_threshold = float(
            self.get_parameter("start_brake_threshold").value
        )

        if not self.odom_topic.startswith("/"):
            raise ValueError("odom_topic must be absolute")
        if not self.actuator_topic.startswith("/"):
            raise ValueError("actuator_topic must be absolute")
        if not file_prefix or any(ch in file_prefix for ch in "/\\"):
            raise ValueError("file_prefix must be a non-empty filename prefix")
        if not 0.0 <= self.run_start_throttle <= 1.0:
            raise ValueError("run_start_throttle must be in [0, 1]")
        if not math.isfinite(self.stationary_speed_m_s) or self.stationary_speed_m_s < 0.0:
            raise ValueError("stationary_speed_m_s must be finite and non-negative")
        if not 0.0 <= self.start_brake_threshold <= 1.0:
            raise ValueError("start_brake_threshold must be in [0, 1]")

        os.makedirs(output_directory, exist_ok=True)
        wall_stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self.csv_path = os.path.join(
            output_directory, f"{file_prefix}_{wall_stamp}.csv"
        )
        self.csv_file = open(self.csv_path, "x", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "ros_timestamp_s",
                "logger_elapsed_s",
                "run_elapsed_s",
                "actual_speed_m_s",
                "actual_speed_km_h",
                "longitudinal_acceleration_m_s2",
                "throttle",
                "brake",
                "steering_angle_rad",
                "steering_angle_deg",
                "run_event",
                "run_peak_speed_km_h",
            ]
        )
        self.csv_file.flush()

        self.logger_start_monotonic = time.monotonic()
        self.latest_actuator = (0.0, 0.0, 0.0)
        self.run_armed = False
        self.run_start_monotonic = None
        self.run_start_ros_s = None
        self.peak_speed_km_h = 0.0
        self.milestone_times = {}
        self.milestones_km_h = (20.0, 40.0, 60.0, 80.0)
        self.previous_speed_m_s = None
        self.previous_odom_ros_s = None
        self.previous_odom_monotonic = None
        self.sample_count = 0

        self.create_subscription(
            ActuatorCommand,
            self.actuator_topic,
            self._actuator_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            50,
        )
        self.get_logger().info(
            f"Dynamics CSV: {self.csv_path}; waiting for stationary vehicle and "
            f"throttle >= {self.run_start_throttle:.2f}"
        )

    def _actuator_callback(self, message):
        steering = float(message.steering_angle)
        throttle = float(message.throttle)
        brake = float(message.brake)
        if all(math.isfinite(value) for value in (steering, throttle, brake)):
            self.latest_actuator = (steering, throttle, brake)

    @staticmethod
    def _ros_timestamp_s(message):
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _calculate_acceleration(self, speed_m_s, ros_time_s, monotonic_now):
        acceleration = 0.0
        if self.previous_speed_m_s is not None:
            ros_dt = ros_time_s - self.previous_odom_ros_s
            monotonic_dt = monotonic_now - self.previous_odom_monotonic
            dt = ros_dt if ros_dt > 0.0 else monotonic_dt
            if dt > 0.0:
                acceleration = (speed_m_s - self.previous_speed_m_s) / dt
        self.previous_speed_m_s = speed_m_s
        self.previous_odom_ros_s = ros_time_s
        self.previous_odom_monotonic = monotonic_now
        return acceleration

    def _update_run(self, speed_m_s, speed_km_h, throttle, brake, ros_time_s, now):
        events = []
        if not self.run_armed and self.run_start_monotonic is None:
            if speed_m_s <= self.stationary_speed_m_s:
                self.run_armed = True

        if (
            self.run_armed
            and self.run_start_monotonic is None
            and throttle >= self.run_start_throttle
            and brake <= self.start_brake_threshold
        ):
            self.run_start_monotonic = now
            self.run_start_ros_s = ros_time_s
            events.append("run_started")
            self.get_logger().info("Dynamics run started")

        if self.run_start_monotonic is None:
            return None, events

        run_elapsed_s = now - self.run_start_monotonic
        self.peak_speed_km_h = max(self.peak_speed_km_h, speed_km_h)
        for threshold in self.milestones_km_h:
            if threshold not in self.milestone_times and speed_km_h >= threshold:
                self.milestone_times[threshold] = run_elapsed_s
                events.append(f"0_{threshold:.0f}_km_h")
                self.get_logger().info(
                    f"0-{threshold:.0f} km/h: {run_elapsed_s:.2f} s"
                )
        return run_elapsed_s, events

    def _odom_callback(self, message):
        linear = message.twist.twist.linear
        speed_m_s = math.hypot(float(linear.x), float(linear.y))
        if not math.isfinite(speed_m_s):
            self.get_logger().warning("Ignored odometry sample containing NaN/Inf")
            return

        monotonic_now = time.monotonic()
        ros_time_s = self._ros_timestamp_s(message)
        acceleration = self._calculate_acceleration(
            speed_m_s, ros_time_s, monotonic_now
        )
        steering, throttle, brake = self.latest_actuator
        speed_km_h = speed_m_s * KMH_PER_MPS
        run_elapsed_s, run_events = self._update_run(
            speed_m_s,
            speed_km_h,
            throttle,
            brake,
            ros_time_s,
            monotonic_now,
        )
        self.csv_writer.writerow(
            [
                f"{ros_time_s:.9f}",
                f"{monotonic_now - self.logger_start_monotonic:.6f}",
                "" if run_elapsed_s is None else f"{run_elapsed_s:.6f}",
                f"{speed_m_s:.6f}",
                f"{speed_km_h:.6f}",
                f"{acceleration:.6f}",
                f"{throttle:.6f}",
                f"{brake:.6f}",
                f"{steering:.6f}",
                f"{math.degrees(steering):.6f}",
                ";".join(run_events),
                f"{self.peak_speed_km_h:.6f}",
            ]
        )
        self.sample_count += 1
        if self.sample_count % 50 == 0:
            self.csv_file.flush()

    def close(self):
        if self.csv_file.closed:
            return
        self.csv_file.flush()
        self.csv_file.close()

        def report(message):
            if rclpy.ok():
                self.get_logger().info(message)
            else:
                print(f"[dynamics_logger_node] {message}", flush=True)

        if self.run_start_monotonic is None:
            report("No dynamics run was started")
        else:
            for threshold in self.milestones_km_h:
                elapsed = self.milestone_times.get(threshold)
                result = "not reached" if elapsed is None else f"{elapsed:.2f} s"
                report(f"0-{threshold:.0f} km/h: {result}")
            report(f"Peak speed: {self.peak_speed_km_h:.1f} km/h")
        report(
            f"Wrote {self.sample_count} samples to {self.csv_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DynamicsLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
