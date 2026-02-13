"""ROS topic publisher helpers for the hand tracking bridge."""

from __future__ import annotations

from collections.abc import Sequence

from geometry_msgs.msg import PoseArray, PoseStamped
from hand_tracking_sdk import HandSide
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

JOINT_NAMES_TOPIC = "hands/joint_names"


def sensor_qos_profile(reliability_mode: str) -> QoSProfile:
    """
    Build sensor-topic QoS profile from a configuration string.

    :param reliability_mode:
        One of ``best_effort`` or ``reliable``.
    :returns:
        QoS profile used for high-rate stream topics.
    :raises ValueError:
        If the reliability mode is unsupported.
    """
    mode = reliability_mode.strip().lower()

    if mode == "best_effort":
        reliability = ReliabilityPolicy.BEST_EFFORT
    elif mode == "reliable":
        reliability = ReliabilityPolicy.RELIABLE
    else:
        raise ValueError(
            "qos_reliability must be one of {'best_effort', 'reliable'}, "
            f"got {reliability_mode!r}"
        )

    return QoSProfile(
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class BridgePublishers:
    """Publisher collection for bridge output topics."""

    def __init__(
        self,
        node: Node,
        *,
        sensor_qos: QoSProfile,
        enable_pose_array: bool,
        enable_markers: bool,
    ) -> None:
        """Create ROS publishers used by the bridge node."""
        self._enable_pose_array = enable_pose_array
        self._enable_markers = enable_markers

        self._left_wrist_pub = node.create_publisher(PoseStamped, "hands/left/wrist_pose", sensor_qos)
        self._right_wrist_pub = node.create_publisher(
            PoseStamped,
            "hands/right/wrist_pose",
            sensor_qos,
        )

        self._left_landmarks_pub = node.create_publisher(
            PoseArray,
            "hands/left/landmarks",
            sensor_qos,
        )
        self._right_landmarks_pub = node.create_publisher(
            PoseArray,
            "hands/right/landmarks",
            sensor_qos,
        )

        self._left_markers_pub = node.create_publisher(
            MarkerArray,
            "hands/left/markers",
            sensor_qos,
        )
        self._right_markers_pub = node.create_publisher(
            MarkerArray,
            "hands/right/markers",
            sensor_qos,
        )

        self._joint_names_pub = node.create_publisher(
            String,
            JOINT_NAMES_TOPIC,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

    def publish_wrist(self, side: HandSide, message: PoseStamped) -> None:
        """Publish one wrist pose message for the selected side."""
        if side == HandSide.LEFT:
            self._left_wrist_pub.publish(message)
        else:
            self._right_wrist_pub.publish(message)

    def publish_landmarks(self, side: HandSide, message: PoseArray) -> None:
        """Publish one landmarks message when landmark output is enabled."""
        if not self._enable_pose_array:
            return
        if side == HandSide.LEFT:
            self._left_landmarks_pub.publish(message)
        else:
            self._right_landmarks_pub.publish(message)

    def publish_markers(self, side: HandSide, message: MarkerArray) -> None:
        """Publish one marker message when marker output is enabled."""
        if not self._enable_markers:
            return
        if side == HandSide.LEFT:
            self._left_markers_pub.publish(message)
        else:
            self._right_markers_pub.publish(message)

    def publish_joint_names(self, joint_names: Sequence[str]) -> None:
        """Publish canonical joint ordering metadata."""
        self._joint_names_pub.publish(String(data=",".join(joint_names)))
