"""Diagnostics message helpers for bridge runtime health."""

from __future__ import annotations

from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .runtime import RuntimeStats


class DiagnosticsPublisher:
    """Publish bridge health into ROS diagnostics."""

    def __init__(self, node: Node, *, topic_name: str = "/diagnostics") -> None:
        """Create diagnostics publisher."""
        self._node = node
        self._publisher = node.create_publisher(DiagnosticArray, topic_name, QoSProfile(depth=10))

    def publish(
        self,
        *,
        runtime_stats: RuntimeStats,
        last_frame_time: Time,
        last_exception: Exception | None,
    ) -> None:
        """Publish one diagnostics snapshot."""
        now = self._node.get_clock().now()
        elapsed = now - last_frame_time
        since_last_frame = elapsed.nanoseconds / 1_000_000.0

        status = DiagnosticStatus()
        status.name = "hand_tracking_sdk_ros2:stream"
        status.hardware_id = "hts_bridge"

        if since_last_frame > 2_000.0:
            status.level = DiagnosticStatus.WARN
            status.message = "stream stale"
        elif last_exception is not None:
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
            KeyValue(key="stream_connected", value=str(since_last_frame <= 2_000.0).lower()),
        ]

        message = DiagnosticArray()
        message.header.stamp = now.to_msg()
        message.status.append(status)
        self._publisher.publish(message)
