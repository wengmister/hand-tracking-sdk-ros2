"""ROS 2 node bridging hand_tracking_sdk stream into topics, TF, and markers."""

from __future__ import annotations

from typing import Final

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseArray, PoseStamped
from hand_tracking_sdk import HandSide, JointName
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray

from .adapters import (
    SideFrames,
    frame_id_for_side,
    is_valid_landmark_count,
    ros_time_from_unix_ns,
    to_landmarks_pose_array,
    to_marker_array,
    to_wrist_pose_stamped,
    to_wrist_transform,
)
from .runtime import FrameRuntime

JOINT_NAMES_TOPIC: Final[str] = "hands/joint_names"


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
        self.declare_parameter("convert_to_right_handed", True)
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
        convert_to_right_handed = bool(self.get_parameter("convert_to_right_handed").value)
        qos_reliability = str(self.get_parameter("qos_reliability").value)
        queue_size = int(self.get_parameter("queue_size").value)
        self._enable_tf = bool(self.get_parameter("enable_tf").value)
        self._enable_pose_array = bool(self.get_parameter("enable_pose_array").value)
        self._enable_markers = bool(self.get_parameter("enable_markers").value)
        self._enable_diagnostics = bool(self.get_parameter("enable_diagnostics").value)
        diagnostics_period_s = float(self.get_parameter("diagnostics_period_s").value)

        if qos_reliability == "reliable":
            reliability = ReliabilityPolicy.RELIABLE
        else:
            reliability = ReliabilityPolicy.BEST_EFFORT

        qos = QoSProfile(
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._world_frame = world_frame
        self._side_frames = SideFrames(left=self._left_wrist_frame, right=self._right_wrist_frame)

        self._left_wrist_pub = self.create_publisher(PoseStamped, "hands/left/wrist_pose", qos)
        self._right_wrist_pub = self.create_publisher(PoseStamped, "hands/right/wrist_pose", qos)

        self._left_landmarks_pub = self.create_publisher(PoseArray, "hands/left/landmarks", qos)
        self._right_landmarks_pub = self.create_publisher(PoseArray, "hands/right/landmarks", qos)

        self._left_markers_pub = self.create_publisher(MarkerArray, "hands/left/markers", qos)
        self._right_markers_pub = self.create_publisher(MarkerArray, "hands/right/markers", qos)

        self._joint_names_pub = self.create_publisher(
            String,
            JOINT_NAMES_TOPIC,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        self._diag_pub = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            QoSProfile(depth=10),
        )

        self._runtime = FrameRuntime(
            transport_mode=transport_mode,
            host=host,
            port=port,
            timeout_s=timeout_s,
            reconnect_delay_s=reconnect_delay_s,
            convert_to_right_handed=convert_to_right_handed,
            queue_size=queue_size,
        )
        self._runtime.start()

        self._last_frame_time = self.get_clock().now()

        self.create_timer(0.005, self._drain_frames)
        if self._enable_diagnostics:
            self.create_timer(diagnostics_period_s, self._publish_diagnostics)

        self._publish_joint_names_once()

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

            wrist_msg = to_wrist_pose_stamped(frame, stamp=stamp, frame_id=frame_id)

            if frame.side == HandSide.LEFT:
                self._left_wrist_pub.publish(wrist_msg)
            else:
                self._right_wrist_pub.publish(wrist_msg)

            if self._enable_pose_array:
                landmarks_msg = to_landmarks_pose_array(frame, stamp=stamp, frame_id=frame_id)
                if frame.side == HandSide.LEFT:
                    self._left_landmarks_pub.publish(landmarks_msg)
                else:
                    self._right_landmarks_pub.publish(landmarks_msg)

            if self._enable_markers:
                markers_msg = to_marker_array(
                    frame,
                    stamp=stamp,
                    frame_id=frame_id,
                    side_ns=frame.side.value.lower(),
                )
                if frame.side == HandSide.LEFT:
                    self._left_markers_pub.publish(markers_msg)
                else:
                    self._right_markers_pub.publish(markers_msg)

            if self._enable_tf:
                if frame.side == HandSide.LEFT:
                    child_frame_id = self._left_wrist_frame
                else:
                    child_frame_id = self._right_wrist_frame
                tf_msg = to_wrist_transform(
                    frame,
                    stamp=stamp,
                    world_frame=self._world_frame,
                    child_frame_id=child_frame_id,
                )
                self._tf_broadcaster.sendTransform(tf_msg)

            self._last_frame_time = self.get_clock().now()

    def _publish_joint_names_once(self) -> None:
        names = [joint.value for joint in JointName]
        message = String(data=",".join(names))
        self._joint_names_pub.publish(message)

    def _publish_diagnostics(self) -> None:
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        runtime_stats = self._runtime.get_stats()
        elapsed = self.get_clock().now() - self._last_frame_time
        since_last_frame = elapsed.nanoseconds / 1_000_000.0

        status = DiagnosticStatus()
        status.name = "hand_tracking_sdk_ros2:stream"
        status.hardware_id = "hts_bridge"

        if since_last_frame > 2_000.0:
            status.level = DiagnosticStatus.WARN
            status.message = "stream stale"
        elif self._runtime.get_last_exception() is not None:
            status.level = DiagnosticStatus.ERROR
            status.message = "runtime error"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ok"

        status.values = [
            KeyValue(key="frames_in", value=str(runtime_stats.frames_in)),
            KeyValue(key="frames_out", value=str(runtime_stats.frames_out)),
            KeyValue(key="frames_dropped_queue", value=str(runtime_stats.frames_dropped_queue)),
            KeyValue(key="loop_errors", value=str(runtime_stats.loop_errors)),
            KeyValue(key="last_frame_age_ms", value=f"{since_last_frame:.2f}"),
        ]

        diag.status.append(status)
        self._diag_pub.publish(diag)


def main(args: list[str] | None = None) -> None:
    """Run bridge node."""
    rclpy.init(args=args)
    node = HandTrackingBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
