"""TF broadcasting helpers for wrist transforms."""

from __future__ import annotations

from builtin_interfaces.msg import Time
from hand_tracking_sdk import HandFrame, HandSide
from tf2_ros import TransformBroadcaster

from .adapters import to_wrist_transform


class WristTfPublisher:
    """Publish wrist transforms from SDK frames."""

    def __init__(
        self,
        broadcaster: TransformBroadcaster,
        *,
        enabled: bool,
        world_frame: str,
        left_wrist_frame: str,
        right_wrist_frame: str,
        map_to_flu: bool,
    ) -> None:
        """Create TF publisher with configured frame names."""
        self._broadcaster = broadcaster
        self._enabled = enabled
        self._world_frame = world_frame
        self._left_wrist_frame = left_wrist_frame
        self._right_wrist_frame = right_wrist_frame
        self._map_to_flu = map_to_flu

    def publish(self, frame: HandFrame, stamp: Time) -> None:
        """Publish one transform if TF output is enabled."""
        if not self._enabled:
            return

        if frame.side == HandSide.LEFT:
            child_frame_id = self._left_wrist_frame
        else:
            child_frame_id = self._right_wrist_frame

        transform = to_wrist_transform(
            frame,
            stamp=stamp,
            world_frame=self._world_frame,
            child_frame_id=child_frame_id,
            map_to_flu=self._map_to_flu,
        )
        self._broadcaster.sendTransform(transform)
