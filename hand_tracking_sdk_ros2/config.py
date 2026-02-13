"""Configuration structures for the hand tracking ROS 2 bridge."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class BridgeConfig:
    """Resolved runtime configuration for :class:`HandTrackingBridgeNode`."""

    transport_mode: str
    host: str
    port: int
    timeout_s: float
    reconnect_delay_s: float
    world_frame: str
    left_wrist_frame: str
    right_wrist_frame: str
    use_source_frame_id: bool
    landmarks_are_wrist_relative: bool
    qos_reliability: str
    queue_size: int
    enable_tf: bool
    enable_pose_array: bool
    enable_markers: bool
    enable_diagnostics: bool
    diagnostics_period_s: float
