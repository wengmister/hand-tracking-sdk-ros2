"""Launch the hand tracking ROS 2 bridge node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate bridge launch description."""
    share_dir = Path(get_package_share_directory("hand_tracking_sdk_ros2"))
    default_params = str(share_dir / "config" / "bridge.params.yaml")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to YAML parameters file.",
            ),
            Node(
                package="hand_tracking_sdk_ros2",
                executable="hand_tracking_bridge",
                name="hand_tracking_bridge",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
