from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("moza_gokart_control"), "config", "moza_r5.yaml"
    )
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="moza_gokart_control",
                executable="moza_input_node",
                name="moza_input_node",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="moza_gokart_control",
                executable="shared_control_node",
                name="shared_control_node",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="moza_gokart_control",
                executable="isaac_adapter_node",
                name="isaac_adapter_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
