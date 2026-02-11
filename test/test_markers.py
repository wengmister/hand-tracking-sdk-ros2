from hand_tracking_sdk_ros2.markers import BONE_EDGES


def test_bone_edges_within_landmark_bounds() -> None:
    for idx_a, idx_b in BONE_EDGES:
        assert 0 <= idx_a < 21
        assert 0 <= idx_b < 21


def test_bone_edges_count() -> None:
    assert len(BONE_EDGES) == 20
