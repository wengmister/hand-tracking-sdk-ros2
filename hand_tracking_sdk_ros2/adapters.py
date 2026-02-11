"""Conversions from SDK frames to ROS message types."""

from __future__ import annotations

from dataclasses import dataclass

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion, TransformStamped
from hand_tracking_sdk import HandFrame, HandSide, JointName
from hand_tracking_sdk import (
    unity_right_to_flu_position,
)
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


def to_wrist_pose_stamped(
    frame: HandFrame,
    *,
    stamp: Time,
    frame_id: str,
    map_to_flu: bool = False,
) -> PoseStamped:
    """Convert one SDK frame to wrist pose message."""
    x, y, z = _map_point_frame(
        x=frame.wrist.x,
        y=frame.wrist.y,
        z=frame.wrist.z,
        map_to_flu=map_to_flu,
    )
    qx, qy, qz, qw = _map_quaternion_frame(
        qx=frame.wrist.qx,
        qy=frame.wrist.qy,
        qz=frame.wrist.qz,
        qw=frame.wrist.qw,
        map_to_flu=map_to_flu,
    )

    return PoseStamped(
        header=make_header(stamp=stamp, frame_id=frame_id),
        pose=Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(
                x=qx,
                y=qy,
                z=qz,
                w=qw,
            ),
        ),
    )


def to_landmarks_pose_array(
    frame: HandFrame,
    *,
    stamp: Time,
    frame_id: str,
    landmarks_are_wrist_relative: bool = False,
    map_to_flu: bool = False,
) -> PoseArray:
    """Convert one SDK frame to landmarks ``PoseArray`` in publish coordinates."""
    points = frame.landmarks.points
    if landmarks_are_wrist_relative:
        points = _landmarks_wrist_local_to_world(points=points, frame=frame)

    mapped_points = [
        _map_point_frame(
            x=x,
            y=y,
            z=z,
            map_to_flu=map_to_flu,
        )
        for x, y, z in points
    ]
    poses = [
        Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        for x, y, z in mapped_points
    ]
    return PoseArray(header=make_header(stamp=stamp, frame_id=frame_id), poses=poses)


def to_wrist_transform(
    frame: HandFrame,
    *,
    stamp: Time,
    world_frame: str,
    child_frame_id: str,
    map_to_flu: bool = False,
) -> TransformStamped:
    """Convert one SDK frame to wrist transform."""
    x, y, z = _map_point_frame(
        x=frame.wrist.x,
        y=frame.wrist.y,
        z=frame.wrist.z,
        map_to_flu=map_to_flu,
    )
    qx, qy, qz, qw = _map_quaternion_frame(
        qx=frame.wrist.qx,
        qy=frame.wrist.qy,
        qz=frame.wrist.qz,
        qw=frame.wrist.qw,
        map_to_flu=map_to_flu,
    )

    transform = TransformStamped()
    transform.header = make_header(stamp=stamp, frame_id=world_frame)
    transform.child_frame_id = child_frame_id
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw
    return transform


def to_marker_array(
    frame: HandFrame,
    *,
    stamp: Time,
    frame_id: str,
    side_ns: str,
    landmarks_are_wrist_relative: bool = False,
    map_to_flu: bool = False,
) -> MarkerArray:
    """Build landmark and bone markers for RViz."""
    points = frame.landmarks.points
    if landmarks_are_wrist_relative:
        points = _landmarks_wrist_local_to_world(points=points, frame=frame)
    points = tuple(
        _map_point_frame(
            x=x,
            y=y,
            z=z,
            map_to_flu=map_to_flu,
        )
        for x, y, z in points
    )

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


def _map_point_frame(
    *,
    x: float,
    y: float,
    z: float,
    map_to_flu: bool,
) -> tuple[float, float, float]:
    """Map SDK coordinates to FLU frame when requested."""
    if not map_to_flu:
        return (x, y, z)
    return unity_right_to_flu_position(x, y, z)


def _map_quaternion_frame(
    *,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    map_to_flu: bool,
) -> tuple[float, float, float, float]:
    """Map SDK quaternion basis to FLU frame when requested."""
    if not map_to_flu:
        return (qx, qy, qz, qw)

    matrix = _quaternion_to_matrix(qx=qx, qy=qy, qz=qz, qw=qw)
    basis = (
        (0.0, 0.0, 1.0),
        (-1.0, 0.0, 0.0),
        (0.0, -1.0, 0.0),
    )
    transformed = _matmul(_matmul(basis, matrix), _transpose(basis))
    return _matrix_to_quaternion(transformed)


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


def _quaternion_to_matrix(
    *,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Convert quaternion components into a rotation matrix."""
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _matrix_to_quaternion(
    matrix: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
) -> tuple[float, float, float, float]:
    """Convert a rotation matrix into normalized quaternion components."""
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]

    trace = m00 + m11 + m22
    if trace > 0.0:
        s = (trace + 1.0) ** 0.5 * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = (1.0 + m00 - m11 - m22) ** 0.5 * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = (1.0 + m11 - m00 - m22) ** 0.5 * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = (1.0 + m22 - m00 - m11) ** 0.5 * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def _transpose(
    matrix: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Return transpose of a 3x3 matrix."""
    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )


def _matmul(
    a: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    b: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Multiply two 3x3 matrices."""
    return (
        (
            a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
        ),
        (
            a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
        ),
        (
            a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
            a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
            a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
        ),
    )
