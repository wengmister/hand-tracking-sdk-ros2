"""Unit tests for adapter conversion helpers."""

from __future__ import annotations

from math import isclose, sqrt

from builtin_interfaces.msg import Time
from hand_tracking_sdk import HandFrame, HandLandmarks, HandSide, WristPose

from hand_tracking_sdk_ros2.adapters import to_landmarks_pose_array


def _frame_with_single_point(
    *,
    wrist: WristPose,
    point: tuple[float, float, float],
) -> HandFrame:
    return HandFrame(
        side=HandSide.LEFT,
        frame_id="left_wrist",
        wrist=wrist,
        landmarks=HandLandmarks(points=(point,)),
        sequence_id=1,
        recv_ts_ns=1,
        recv_time_unix_ns=1,
        source_ts_ns=None,
        wrist_recv_ts_ns=1,
        landmarks_recv_ts_ns=1,
    )


def test_landmarks_pose_array_without_wrist_relative_transform() -> None:
    """Landmarks should map directly to FLU when relative transform is disabled."""
    frame = _frame_with_single_point(
        wrist=WristPose(x=10.0, y=20.0, z=30.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        point=(1.0, 2.0, 3.0),
    )

    message = to_landmarks_pose_array(
        frame,
        stamp=Time(sec=0, nanosec=0),
        frame_id="world",
        landmarks_are_wrist_relative=False,
    )

    assert len(message.poses) == 1
    assert message.poses[0].position.x == 3.0
    assert message.poses[0].position.y == -1.0
    assert message.poses[0].position.z == 2.0


def test_landmarks_pose_array_with_wrist_relative_transform() -> None:
    """Landmarks should transform to world then map into FLU when enabled."""
    sin_45 = 1.0 / sqrt(2.0)
    frame = _frame_with_single_point(
        wrist=WristPose(x=1.0, y=2.0, z=3.0, qx=0.0, qy=0.0, qz=sin_45, qw=sin_45),
        point=(1.0, 0.0, 0.0),
    )

    message = to_landmarks_pose_array(
        frame,
        stamp=Time(sec=0, nanosec=0),
        frame_id="world",
        landmarks_are_wrist_relative=True,
    )

    assert len(message.poses) == 1
    assert isclose(message.poses[0].position.x, 3.0, rel_tol=0.0, abs_tol=1e-6)
    assert isclose(message.poses[0].position.y, -1.0, rel_tol=0.0, abs_tol=1e-6)
    assert isclose(message.poses[0].position.z, 3.0, rel_tol=0.0, abs_tol=1e-6)


def test_landmarks_pose_array_uses_flu_mapping() -> None:
    """Landmarks should map points into FLU basis."""
    frame = _frame_with_single_point(
        wrist=WristPose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        point=(1.0, 2.0, 3.0),
    )

    message = to_landmarks_pose_array(
        frame,
        stamp=Time(sec=0, nanosec=0),
        frame_id="world",
        landmarks_are_wrist_relative=False,
    )

    assert len(message.poses) == 1
    assert message.poses[0].position.x == 3.0
    assert message.poses[0].position.y == -1.0
    assert message.poses[0].position.z == 2.0
