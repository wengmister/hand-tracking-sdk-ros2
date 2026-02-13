"""Unit tests for bridge-facing parameter validation surfaces."""

from __future__ import annotations

import pytest
from hand_tracking_sdk.exceptions import ClientConfigurationError

from hand_tracking_sdk_ros2.publishers import sensor_qos_profile
from hand_tracking_sdk_ros2.runtime import FrameRuntime


def test_rejects_invalid_qos_mode() -> None:
    """Unsupported qos mode should raise validation error."""
    with pytest.raises(ValueError, match='qos_reliability'):
        sensor_qos_profile('invalid_mode')


def test_rejects_invalid_transport_mode() -> None:
    """Invalid transport mode should fail runtime configuration."""
    with pytest.raises(ValueError, match='TransportMode'):
        FrameRuntime(
            transport_mode='invalid_mode',
            host='0.0.0.0',
            port=9000,
            timeout_s=1.0,
            reconnect_delay_s=0.25,
            queue_size=8,
        )


def test_rejects_empty_host() -> None:
    """Empty host should be rejected by upstream SDK config validation."""
    with pytest.raises(ClientConfigurationError, match='host must not be empty'):
        FrameRuntime(
            transport_mode='tcp_server',
            host='',
            port=9000,
            timeout_s=1.0,
            reconnect_delay_s=0.25,
            queue_size=8,
        )


def test_rejects_invalid_timeout() -> None:
    """Non-positive timeout should be rejected by SDK config validation."""
    with pytest.raises(ClientConfigurationError, match='timeout_s must be greater than 0'):
        FrameRuntime(
            transport_mode='tcp_server',
            host='0.0.0.0',
            port=9000,
            timeout_s=0.0,
            reconnect_delay_s=0.25,
            queue_size=8,
        )
