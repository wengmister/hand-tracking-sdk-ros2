"""Conversions from SDK frames to ROS message types."""

from __future__ import annotations

from dataclasses import dataclass

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion, TransformStamped
from hand_tracking_sdk import HandFrame, HandSide, JointName
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from .markers import BONE_EDGES


@dataclass(frozen=True, slots=True)
class SideFrames:
    """Configured frame ids per hand side."""

    left: str
    right: str


def ros_time_from_unix_ns(unix_ns: int) -> Time:
    """Convert unix nanoseconds to ROS ``builtin_interfaces/Time``."""
    secs, nsecs = divmod(unix_ns, 1_000_000_000)
    return Time(sec=int(secs), nanosec=int(nsecs))


def frame_id_for_side(
    frame: HandFrame,
    *,
    side_frames: SideFrames,
    use_source_frame_id: bool,
) -> str:
    """Resolve frame id for one SDK frame."""
    if use_source_frame_id and frame.frame_id:
        return frame.frame_id
    if frame.side == HandSide.LEFT:
        return side_frames.left
    return side_frames.right


def make_header(stamp: Time, frame_id: str) -> Header:
    """Build standard ROS header."""
    return Header(stamp=stamp, frame_id=frame_id)


def to_wrist_pose_stamped(frame: HandFrame, *, stamp: Time, frame_id: str) -> PoseStamped:
    """Convert one SDK frame to wrist pose message."""
    return PoseStamped(
        header=make_header(stamp=stamp, frame_id=frame_id),
        pose=Pose(
            position=Point(x=frame.wrist.x, y=frame.wrist.y, z=frame.wrist.z),
            orientation=Quaternion(
                x=frame.wrist.qx,
                y=frame.wrist.qy,
                z=frame.wrist.qz,
                w=frame.wrist.qw,
            ),
        ),
    )


def to_landmarks_pose_array(
    frame: HandFrame,
    *,
    stamp: Time,
    frame_id: str,
    landmarks_are_wrist_relative: bool = False,
) -> PoseArray:
    """Convert one SDK frame to landmarks ``PoseArray`` in publish coordinates."""
    points = frame.landmarks.points
    if landmarks_are_wrist_relative:
        points = _landmarks_wrist_local_to_world(points=points, frame=frame)

    poses = [
        Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        for x, y, z in points
    ]
    return PoseArray(header=make_header(stamp=stamp, frame_id=frame_id), poses=poses)


def to_wrist_transform(
    frame: HandFrame,
    *,
    stamp: Time,
    world_frame: str,
    child_frame_id: str,
) -> TransformStamped:
    """Convert one SDK frame to wrist transform."""
    transform = TransformStamped()
    transform.header = make_header(stamp=stamp, frame_id=world_frame)
    transform.child_frame_id = child_frame_id
    transform.transform.translation.x = frame.wrist.x
    transform.transform.translation.y = frame.wrist.y
    transform.transform.translation.z = frame.wrist.z
    transform.transform.rotation.x = frame.wrist.qx
    transform.transform.rotation.y = frame.wrist.qy
    transform.transform.rotation.z = frame.wrist.qz
    transform.transform.rotation.w = frame.wrist.qw
    return transform


def to_marker_array(
    frame: HandFrame,
    *,
    stamp: Time,
    frame_id: str,
    side_ns: str,
    landmarks_are_wrist_relative: bool = False,
) -> MarkerArray:
    """Build landmark and bone markers for RViz."""
    points = frame.landmarks.points
    if landmarks_are_wrist_relative:
        points = _landmarks_wrist_local_to_world(points=points, frame=frame)

    marker_array = MarkerArray()

    if frame.side == HandSide.LEFT:
        color_r = 0.2
        color_g = 0.4
        color_b = 1.0
    else:
        color_r = 1.0
        color_g = 0.2
        color_b = 0.2

    for idx, (x, y, z) in enumerate(points):
        joint_marker = Marker()
        joint_marker.header = make_header(stamp=stamp, frame_id=frame_id)
        joint_marker.ns = f"{side_ns}_landmark_spheres"
        joint_marker.id = idx
        joint_marker.type = Marker.SPHERE
        joint_marker.action = Marker.ADD
        joint_marker.pose.position.x = x
        joint_marker.pose.position.y = y
        joint_marker.pose.position.z = z
        joint_marker.pose.orientation.w = 1.0
        joint_marker.scale.x = 0.012
        joint_marker.scale.y = 0.012
        joint_marker.scale.z = 0.012
        joint_marker.color.r = color_r
        joint_marker.color.g = color_g
        joint_marker.color.b = color_b
        joint_marker.color.a = 1.0
        marker_array.markers.append(joint_marker)

    line_marker = Marker()
    line_marker.header = make_header(stamp=stamp, frame_id=frame_id)
    line_marker.ns = f"{side_ns}_bones"
    line_marker.id = 1000
    line_marker.type = Marker.LINE_LIST
    line_marker.action = Marker.ADD
    line_marker.scale.x = 0.004
    line_marker.color.r = color_r
    line_marker.color.g = color_g
    line_marker.color.b = color_b
    line_marker.color.a = 1.0

    for idx_a, idx_b in BONE_EDGES:
        pa = points[idx_a]
        pb = points[idx_b]
        line_marker.points.append(Point(x=pa[0], y=pa[1], z=pa[2]))
        line_marker.points.append(Point(x=pb[0], y=pb[1], z=pb[2]))

    marker_array.markers.append(line_marker)
    return marker_array


def is_valid_landmark_count(frame: HandFrame) -> bool:
    """Return whether frame carries expected landmark point count."""
    return len(frame.landmarks.points) == len(JointName)


def _landmarks_wrist_local_to_world(
    *,
    points: tuple[tuple[float, float, float], ...],
    frame: HandFrame,
) -> tuple[tuple[float, float, float], ...]:
    """Transform wrist-local landmark points into world coordinates."""
    world_points: list[tuple[float, float, float]] = []
    for x, y, z in points:
        rx, ry, rz = _rotate_vector_by_quaternion(
            x=x,
            y=y,
            z=z,
            qx=frame.wrist.qx,
            qy=frame.wrist.qy,
            qz=frame.wrist.qz,
            qw=frame.wrist.qw,
        )
        world_points.append((rx + frame.wrist.x, ry + frame.wrist.y, rz + frame.wrist.z))
    return tuple(world_points)


def _rotate_vector_by_quaternion(
    *,
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[float, float, float]:
    """Rotate a vector using quaternion orientation."""
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if norm == 0.0:
        return (x, y, z)

    qx_n = qx / norm
    qy_n = qy / norm
    qz_n = qz / norm
    qw_n = qw / norm

    # Equivalent to q * v * conj(q), expanded for speed.
    tx = 2.0 * (qy_n * z - qz_n * y)
    ty = 2.0 * (qz_n * x - qx_n * z)
    tz = 2.0 * (qx_n * y - qy_n * x)

    rx = x + qw_n * tx + (qy_n * tz - qz_n * ty)
    ry = y + qw_n * ty + (qz_n * tx - qx_n * tz)
    rz = z + qw_n * tz + (qx_n * ty - qy_n * tx)
    return (rx, ry, rz)
