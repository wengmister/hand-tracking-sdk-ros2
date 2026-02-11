"""Marker definitions for hand landmarks visualization."""

from __future__ import annotations

from hand_tracking_sdk.constants import STREAMED_JOINT_NAMES

# Edges in streamed landmark index order.
BONE_EDGES: tuple[tuple[int, int], ...] = (
    # Thumb
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    # Index
    (0, 5),
    (5, 6),
    (6, 7),
    (7, 8),
    # Middle
    (0, 9),
    (9, 10),
    (10, 11),
    (11, 12),
    # Ring
    (0, 13),
    (13, 14),
    (14, 15),
    (15, 16),
    # Little
    (0, 17),
    (17, 18),
    (18, 19),
    (19, 20),
)

JOINT_NAMES: tuple[str, ...] = STREAMED_JOINT_NAMES
