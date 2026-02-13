"""Unit tests for adapter conversion helpers."""

from __future__ import annotations

from math import isclose, sqrt

from builtin_interfaces.msg import Time
from hand_tracking_sdk import HandFrame, HandLandmarks, HandSide, WristPose
from visualization_msgs.msg import Marker

from hand_tracking_sdk_ros2.adapters import (
    SideFrames,
    frame_id_for_side,
    is_valid_landmark_count,
    ros_time_from_unix_ns,
    to_landmarks_pose_array,
    to_marker_array,
    to_wrist_pose_stamped,
    to_wrist_transform,
)


def _frame_with_points(
    *,
    side: HandSide = HandSide.LEFT,
    wrist: WristPose,
    points: tuple[tuple[float, float, float], ...],
    frame_id: str = 'left_wrist',
) -> HandFrame:
    return HandFrame(
        side=side,
        frame_id=frame_id,
        wrist=wrist,
        landmarks=HandLandmarks(points=points),
        sequence_id=1,
        recv_ts_ns=1,
        recv_time_unix_ns=1,
        source_ts_ns=None,
        wrist_recv_ts_ns=1,
        landmarks_recv_ts_ns=1,
    )


def _canonical_points() -> tuple[tuple[float, float, float], ...]:
    return tuple((float(i), float(i + 1), float(i + 2)) for i in range(21))


def test_ros_time_from_unix_ns() -> None:
    """Converter should split unix nanoseconds into sec and nanosec."""
    message = ros_time_from_unix_ns(1_234_567_890)
    assert message.sec == 1
    assert message.nanosec == 234_567_890


def test_frame_id_for_side_uses_source_frame_when_enabled() -> None:
    """Resolver should use source frame id when explicitly enabled."""
    frame = _frame_with_points(
        side=HandSide.RIGHT,
        wrist=WristPose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        points=_canonical_points(),
        frame_id='source_right',
    )
    side_frames = SideFrames(left='left_wrist', right='right_wrist')

    resolved = frame_id_for_side(frame, side_frames=side_frames, use_source_frame_id=True)

    assert resolved == 'source_right'


def test_frame_id_for_side_falls_back_to_side_frame() -> None:
    """Resolver should choose configured per-side frame when source frame is disabled."""
    frame = _frame_with_points(
        side=HandSide.RIGHT,
        wrist=WristPose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        points=_canonical_points(),
        frame_id='source_right',
    )
    side_frames = SideFrames(left='left_wrist', right='right_wrist')

    resolved = frame_id_for_side(frame, side_frames=side_frames, use_source_frame_id=False)

    assert resolved == 'right_wrist'


def test_wrist_pose_stamped_maps_position_and_preserves_identity_orientation() -> None:
    """Converter should map Unity-left wrist pose into FLU output frame."""
    frame = _frame_with_points(
        wrist=WristPose(x=1.0, y=2.0, z=3.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        points=_canonical_points(),
    )

    message = to_wrist_pose_stamped(
        frame,
        stamp=Time(sec=5, nanosec=7),
        frame_id='left_wrist',
    )

    assert message.header.frame_id == 'left_wrist'
    assert message.header.stamp.sec == 5
    assert message.header.stamp.nanosec == 7
    assert message.pose.position.x == 3.0
    assert message.pose.position.y == -1.0
    assert message.pose.position.z == 2.0
    assert isclose(message.pose.orientation.x, 0.0, rel_tol=0.0, abs_tol=1e-8)
    assert isclose(message.pose.orientation.y, 0.0, rel_tol=0.0, abs_tol=1e-8)
    assert isclose(message.pose.orientation.z, 0.0, rel_tol=0.0, abs_tol=1e-8)
    assert isclose(message.pose.orientation.w, 1.0, rel_tol=0.0, abs_tol=1e-8)


def test_wrist_transform_maps_position_and_sets_frames() -> None:
    """Transformer should map wrist pose and set TF frame ids."""
    frame = _frame_with_points(
        wrist=WristPose(x=1.0, y=2.0, z=3.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        points=_canonical_points(),
    )

    transform = to_wrist_transform(
        frame,
        stamp=Time(sec=9, nanosec=11),
        world_frame='world',
        child_frame_id='left_wrist',
    )

    assert transform.header.frame_id == 'world'
    assert transform.child_frame_id == 'left_wrist'
    assert transform.header.stamp.sec == 9
    assert transform.header.stamp.nanosec == 11
    assert transform.transform.translation.x == 3.0
    assert transform.transform.translation.y == -1.0
    assert transform.transform.translation.z == 2.0
    assert isclose(transform.transform.rotation.w, 1.0, rel_tol=0.0, abs_tol=1e-8)


def test_landmarks_pose_array_without_wrist_relative_transform() -> None:
    """Landmarks should map directly to FLU when relative transform is disabled."""
    frame = _frame_with_points(
        wrist=WristPose(x=10.0, y=20.0, z=30.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        points=((1.0, 2.0, 3.0),),
    )

    message = to_landmarks_pose_array(
        frame,
        stamp=Time(sec=0, nanosec=0),
        frame_id='world',
        landmarks_are_wrist_relative=False,
    )

    assert len(message.poses) == 1
    assert message.poses[0].position.x == 3.0
    assert message.poses[0].position.y == -1.0
    assert message.poses[0].position.z == 2.0


def test_landmarks_pose_array_with_wrist_relative_transform() -> None:
    """Landmarks should transform to world then map into FLU when enabled."""
    sin_45 = 1.0 / sqrt(2.0)
    frame = _frame_with_points(
        wrist=WristPose(x=1.0, y=2.0, z=3.0, qx=0.0, qy=0.0, qz=sin_45, qw=sin_45),
        points=((1.0, 0.0, 0.0),),
    )

    message = to_landmarks_pose_array(
        frame,
        stamp=Time(sec=0, nanosec=0),
        frame_id='world',
        landmarks_are_wrist_relative=True,
    )

    assert len(message.poses) == 1
    assert isclose(message.poses[0].position.x, 3.0, rel_tol=0.0, abs_tol=1e-6)
    assert isclose(message.poses[0].position.y, -1.0, rel_tol=0.0, abs_tol=1e-6)
    assert isclose(message.poses[0].position.z, 3.0, rel_tol=0.0, abs_tol=1e-6)


def test_landmarks_pose_array_uses_flu_mapping() -> None:
    """Landmarks should map points into FLU basis."""
    frame = _frame_with_points(
        wrist=WristPose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        points=((1.0, 2.0, 3.0),),
    )

    message = to_landmarks_pose_array(
        frame,
        stamp=Time(sec=0, nanosec=0),
        frame_id='world',
        landmarks_are_wrist_relative=False,
    )

    assert len(message.poses) == 1
    assert message.poses[0].position.x == 3.0
    assert message.poses[0].position.y == -1.0
    assert message.poses[0].position.z == 2.0


def test_marker_array_for_left_contains_expected_markers_and_color() -> None:
    """Marker conversion should emit spheres and bone lines with left-hand color."""
    frame = _frame_with_points(
        side=HandSide.LEFT,
        wrist=WristPose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        points=_canonical_points(),
    )

    marker_array = to_marker_array(
        frame,
        stamp=Time(sec=0, nanosec=0),
        frame_id='world',
        side_ns='left',
        landmarks_are_wrist_relative=False,
    )

    assert len(marker_array.markers) == 22
    first = marker_array.markers[0]
    bones = marker_array.markers[-1]

    assert first.type == Marker.SPHERE
    assert first.ns == 'left_landmark_spheres'
    assert first.color.r == 0.2
    assert first.color.g == 0.4
    assert first.color.b == 1.0
    assert bones.type == Marker.LINE_LIST
    assert bones.ns == 'left_bones'
    assert len(bones.points) == 40


def test_marker_array_for_right_uses_right_color() -> None:
    """Marker conversion should apply right-hand color palette."""
    frame = _frame_with_points(
        side=HandSide.RIGHT,
        wrist=WristPose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        points=_canonical_points(),
        frame_id='right_wrist',
    )

    marker_array = to_marker_array(
        frame,
        stamp=Time(sec=0, nanosec=0),
        frame_id='world',
        side_ns='right',
        landmarks_are_wrist_relative=False,
    )

    first = marker_array.markers[0]
    assert first.color.r == 1.0
    assert first.color.g == 0.2
    assert first.color.b == 0.2


def test_is_valid_landmark_count_checks_expected_joint_count() -> None:
    """Validator should enforce canonical landmark count."""
    valid = _frame_with_points(
        wrist=WristPose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        points=_canonical_points(),
    )
    invalid = _frame_with_points(
        wrist=WristPose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        points=((0.0, 0.0, 0.0),),
    )

    assert is_valid_landmark_count(valid)
    assert not is_valid_landmark_count(invalid)
