"""ROS 2 node bridging hand_tracking_sdk stream into topics, TF, and markers."""

from __future__ import annotations

import rclpy
from hand_tracking_sdk import JointName
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .adapters import (
    SideFrames,
    frame_id_for_side,
    is_valid_landmark_count,
    ros_time_from_unix_ns,
    to_landmarks_pose_array,
    to_marker_array,
    to_wrist_pose_stamped,
)
from .diagnostics import DiagnosticsPublisher
from .publishers import BridgePublishers, sensor_qos_profile
from .runtime import FrameRuntime
from .tf_broadcaster import WristTfPublisher


class HandTrackingBridgeNode(Node):
    """Bridge node that publishes HTS stream as ROS 2 primitives."""

    def __init__(self) -> None:
        super().__init__("hand_tracking_bridge")

        self.declare_parameter("transport_mode", "tcp_server")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 9000)
        self.declare_parameter("timeout_s", 1.0)
        self.declare_parameter("reconnect_delay_s", 0.25)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("left_wrist_frame", "left_wrist")
        self.declare_parameter("right_wrist_frame", "right_wrist")
        self.declare_parameter("use_source_frame_id", False)
        self.declare_parameter("landmarks_are_wrist_relative", True)
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("queue_size", 256)
        self.declare_parameter("enable_tf", True)
        self.declare_parameter("enable_pose_array", True)
        self.declare_parameter("enable_markers", True)
        self.declare_parameter("enable_diagnostics", True)
        self.declare_parameter("diagnostics_period_s", 1.0)

        transport_mode = str(self.get_parameter("transport_mode").value)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        timeout_s = float(self.get_parameter("timeout_s").value)
        reconnect_delay_s = float(self.get_parameter("reconnect_delay_s").value)
        world_frame = str(self.get_parameter("world_frame").value)
        self._left_wrist_frame = str(self.get_parameter("left_wrist_frame").value)
        self._right_wrist_frame = str(self.get_parameter("right_wrist_frame").value)
        self._use_source_frame_id = bool(self.get_parameter("use_source_frame_id").value)
        self._landmarks_are_wrist_relative = bool(
            self.get_parameter("landmarks_are_wrist_relative").value
        )
        qos_reliability = str(self.get_parameter("qos_reliability").value)
        queue_size = int(self.get_parameter("queue_size").value)
        self._enable_tf = bool(self.get_parameter("enable_tf").value)
        self._enable_pose_array = bool(self.get_parameter("enable_pose_array").value)
        self._enable_markers = bool(self.get_parameter("enable_markers").value)
        self._enable_diagnostics = bool(self.get_parameter("enable_diagnostics").value)
        diagnostics_period_s = float(self.get_parameter("diagnostics_period_s").value)

        qos = sensor_qos_profile(qos_reliability)

        self._world_frame = world_frame
        self._side_frames = SideFrames(left=self._left_wrist_frame, right=self._right_wrist_frame)

        self._bridge_publishers = BridgePublishers(
            self,
            sensor_qos=qos,
            enable_pose_array=self._enable_pose_array,
            enable_markers=self._enable_markers,
        )
        self._tf_publisher = WristTfPublisher(
            TransformBroadcaster(self),
            enabled=self._enable_tf,
            world_frame=self._world_frame,
            left_wrist_frame=self._left_wrist_frame,
            right_wrist_frame=self._right_wrist_frame,
        )
        self._diagnostics = DiagnosticsPublisher(self)

        self._runtime = FrameRuntime(
            transport_mode=transport_mode,
            host=host,
            port=port,
            timeout_s=timeout_s,
            reconnect_delay_s=reconnect_delay_s,
            queue_size=queue_size,
        )
        self._runtime.start()

        self._last_frame_time = self.get_clock().now()

        self.create_timer(0.005, self._drain_frames)
        if self._enable_diagnostics:
            self.create_timer(diagnostics_period_s, self._publish_diagnostics)

        self._bridge_publishers.publish_joint_names([joint.value for joint in JointName])

        self.get_logger().info(
            "Started hand_tracking_bridge transport=%s host=%s port=%d qos=%s"
            % (transport_mode, host, port, qos_reliability)
        )

    def destroy_node(self) -> bool:
        self._runtime.stop()
        return super().destroy_node()

    def _drain_frames(self) -> None:
        while True:
            frame = self._runtime.pop_frame()
            if frame is None:
                return

            if frame.recv_time_unix_ns is not None:
                stamp = ros_time_from_unix_ns(frame.recv_time_unix_ns)
            else:
                stamp = self.get_clock().now().to_msg()

            frame_id = frame_id_for_side(
                frame,
                side_frames=self._side_frames,
                use_source_frame_id=self._use_source_frame_id,
            )

            if not is_valid_landmark_count(frame):
                self.get_logger().warn(
                    "Unexpected landmark count=%d side=%s"
                    % (len(frame.landmarks.points), frame.side.value)
                )
                continue

            wrist_msg = to_wrist_pose_stamped(
                frame,
                stamp=stamp,
                frame_id=frame_id,
            )
            self._bridge_publishers.publish_wrist(frame.side, wrist_msg)

            if self._landmarks_are_wrist_relative:
                landmarks_frame_id = self._world_frame
            else:
                landmarks_frame_id = frame_id

            landmarks_msg = to_landmarks_pose_array(
                frame,
                stamp=stamp,
                frame_id=landmarks_frame_id,
                landmarks_are_wrist_relative=self._landmarks_are_wrist_relative,
            )
            self._bridge_publishers.publish_landmarks(frame.side, landmarks_msg)

            markers_msg = to_marker_array(
                frame,
                stamp=stamp,
                frame_id=landmarks_frame_id,
                side_ns=frame.side.value.lower(),
                landmarks_are_wrist_relative=self._landmarks_are_wrist_relative,
            )
            self._bridge_publishers.publish_markers(frame.side, markers_msg)

            self._tf_publisher.publish(frame, stamp)

            self._last_frame_time = self.get_clock().now()

    def _publish_diagnostics(self) -> None:
        self._diagnostics.publish(
            runtime_stats=self._runtime.get_stats(),
            last_frame_time=self._last_frame_time,
            last_exception=self._runtime.get_last_exception(),
        )


def main(args: list[str] | None = None) -> None:
    """Run bridge node."""
    rclpy.init(args=args)
    node = HandTrackingBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
