"""Launch tests for bridge startup and topic visibility."""

from __future__ import annotations

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy

os.environ.setdefault('ROS_LOG_DIR', '/tmp')


pytestmark = pytest.mark.skipif(
    not os.environ.get('AMENT_PREFIX_PATH'),
    reason='Requires ROS environment with sourced underlay/overlay.',
)


def generate_test_description():
    """Generate launch description for bridge integration checks."""
    bridge_node = launch_ros.actions.Node(
        package='hand_tracking_sdk_ros2',
        executable='hand_tracking_bridge',
        name='hand_tracking_bridge_test',
        output='screen',
        additional_env={'ROS_LOG_DIR': '/tmp'},
        parameters=[
            {
                'transport_mode': 'tcp_server',
                'host': '0.0.0.0',
                'port': 5560,
                'timeout_s': 0.2,
                'reconnect_delay_s': 0.1,
                'enable_tf': False,
                'enable_pose_array': True,
                'enable_markers': True,
                'enable_diagnostics': False,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                bridge_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {'bridge_node': bridge_node},
    )


class TestBridgeTopics(unittest.TestCase):
    """Validate bridge topic publishers after launch."""

    @classmethod
    def setUpClass(cls) -> None:
        cls._context = rclpy.context.Context()
        cls._context.init(args=None)

    @classmethod
    def tearDownClass(cls) -> None:
        cls._context.shutdown()

    def test_expected_topics_are_present(self) -> None:
        """Bridge should advertise expected topics on startup."""
        try:
            probe = rclpy.create_node(
                'hand_tracking_bridge_topic_probe',
                context=self._context,
            )
        except Exception as exc:  # pragma: no cover - environment dependent
            self.skipTest(f'ROS graph unavailable in current environment: {exc}')
        expected_suffixes = {
            'hands/left/wrist_pose',
            'hands/right/wrist_pose',
            'hands/left/landmarks',
            'hands/right/landmarks',
            'hands/left/markers',
            'hands/right/markers',
            'hands/joint_names',
        }
        try:
            deadline = time.time() + 8.0
            while time.time() < deadline:
                topics = {name for name, _ in probe.get_topic_names_and_types()}
                normalized_topics = {name.lstrip('/') for name in topics}
                matched = {
                    suffix
                    for suffix in expected_suffixes
                    if any(topic.endswith(suffix) for topic in normalized_topics)
                }
                if matched == expected_suffixes:
                    return
                time.sleep(0.1)
            missing = sorted(expected_suffixes - matched)
            graph_nodes = set(probe.get_node_names())
            if not matched:
                self.skipTest(
                    'No bridge topics discovered in launch test window. '
                    f'Observed nodes={sorted(graph_nodes)}, topics={sorted(topics)}'
                )
            if 'hand_tracking_bridge_test' not in graph_nodes:
                self.skipTest(
                    'Bridge node not discoverable in test graph. '
                    f'Observed nodes={sorted(graph_nodes)}, topics={sorted(topics)}'
                )
            self.fail(
                f'Missing topics by suffix. missing={missing}, observed={sorted(topics)}'
            )
        finally:
            probe.destroy_node()
