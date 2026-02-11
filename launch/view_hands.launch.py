"""Launch bridge and RViz view for live hand telemetry."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for bridge + RViz."""
    share_dir = Path(get_package_share_directory("hand_tracking_sdk_ros2"))
    default_params = str(share_dir / "config" / "bridge.params.yaml")
    default_rviz = str(share_dir / "rviz" / "hand_tracking.rviz")

    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            Node(
                package="hand_tracking_sdk_ros2",
                executable="hand_tracking_bridge",
                name="hand_tracking_bridge",
                output="screen",
                parameters=[
                    params_file,
                    {"qos_reliability": "reliable"},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
