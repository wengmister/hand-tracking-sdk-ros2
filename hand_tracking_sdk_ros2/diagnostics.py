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
        self._previous_stats: RuntimeStats | None = None
        self._previous_publish_time: Time | None = None

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
        stream_connected = since_last_frame <= 2_000.0

        input_fps = 0.0
        output_fps = 0.0
        if self._previous_stats is not None and self._previous_publish_time is not None:
            dt_s = (now - self._previous_publish_time).nanoseconds / 1_000_000_000.0
            if dt_s > 0.0:
                input_fps = max(runtime_stats.frames_in - self._previous_stats.frames_in, 0) / dt_s
                output_fps = max(runtime_stats.frames_out - self._previous_stats.frames_out, 0) / dt_s

        self._previous_stats = runtime_stats
        self._previous_publish_time = now

        status = DiagnosticStatus()
        status.name = "hand_tracking_sdk_ros2:stream"
        status.hardware_id = "hts_bridge"

        if last_exception is not None:
            status.level = DiagnosticStatus.ERROR
            status.message = "runtime error"
        elif not stream_connected:
            status.level = DiagnosticStatus.WARN
            status.message = "stream stale"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ok"

        status.values = [
            KeyValue(key="frames_in", value=str(runtime_stats.frames_in)),
            KeyValue(key="frames_out", value=str(runtime_stats.frames_out)),
            KeyValue(key="frames_dropped_queue", value=str(runtime_stats.frames_dropped_queue)),
            KeyValue(key="dropped_lines", value=str(runtime_stats.dropped_lines)),
            KeyValue(key="parse_errors", value=str(runtime_stats.parse_errors)),
            KeyValue(key="callback_errors", value=str(runtime_stats.callback_errors)),
            KeyValue(key="loop_errors", value=str(runtime_stats.loop_errors)),
            KeyValue(key="input_fps", value=f"{input_fps:.2f}"),
            KeyValue(key="output_fps", value=f"{output_fps:.2f}"),
            KeyValue(key="last_frame_age_ms", value=f"{since_last_frame:.2f}"),
            KeyValue(key="stream_connected", value=str(stream_connected).lower()),
            KeyValue(key="last_exception", value=repr(last_exception) if last_exception else ""),
        ]

        message = DiagnosticArray()
        message.header.stamp = now.to_msg()
        message.status.append(status)
        self._publisher.publish(message)
