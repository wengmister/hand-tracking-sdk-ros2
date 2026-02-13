"""Unit tests for diagnostics publishing helpers."""

from __future__ import annotations

from builtin_interfaces.msg import Time as RosTime
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.time import Time

from hand_tracking_sdk_ros2.diagnostics import DiagnosticsPublisher
from hand_tracking_sdk_ros2.runtime import RuntimeStats


class _FakeDuration:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds


class _FakeTime:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds

    def __sub__(self, other: '_FakeTime') -> _FakeDuration:
        return _FakeDuration(self.nanoseconds - other.nanoseconds)

    def to_msg(self) -> RosTime:
        secs, nsecs = divmod(self.nanoseconds, 1_000_000_000)
        return RosTime(sec=secs, nanosec=nsecs)


class _FakeClock:
    def __init__(self, now_ns: int) -> None:
        self.now_ns = now_ns

    def now(self) -> _FakeTime:
        return _FakeTime(self.now_ns)


class _FakePublisher:
    def __init__(self) -> None:
        self.last_message = None

    def publish(self, message: object) -> None:
        self.last_message = message


class _FakeNode:
    def __init__(self, now_ns: int) -> None:
        self.clock = _FakeClock(now_ns=now_ns)
        self.publisher = _FakePublisher()

    def create_publisher(self, *_: object, **__: object) -> _FakePublisher:
        return self.publisher

    def get_clock(self) -> _FakeClock:
        return self.clock


def _value_map(status: DiagnosticStatus) -> dict[str, str]:
    return {entry.key: entry.value for entry in status.values}


def test_publish_reports_expected_keys_and_ok_state() -> None:
    """Diagnostics include stream counters, errors, and stream connectivity."""
    node = _FakeNode(now_ns=2_000_000_000)
    publisher = DiagnosticsPublisher(node)
    publisher.publish(
        runtime_stats=RuntimeStats(
            frames_in=10,
            frames_out=9,
            frames_dropped_queue=1,
            loop_errors=0,
            parse_errors=2,
            dropped_lines=3,
            callback_errors=0,
        ),
        last_frame_time=Time(nanoseconds=1_900_000_000),
        last_exception=None,
    )

    message = node.publisher.last_message
    assert message is not None
    assert len(message.status) == 1
    status = message.status[0]
    values = _value_map(status)

    assert status.level == DiagnosticStatus.OK
    assert status.message == 'ok'
    assert values['frames_in'] == '10'
    assert values['frames_out'] == '9'
    assert values['frames_dropped_queue'] == '1'
    assert values['dropped_lines'] == '3'
    assert values['parse_errors'] == '2'
    assert values['callback_errors'] == '0'
    assert values['stream_connected'] == 'true'
    assert 'input_fps' in values
    assert 'output_fps' in values
    assert values['last_exception'] == ''


def test_publish_marks_error_and_computes_fps_delta() -> None:
    """Diagnostics report ERROR on exception and derive fps from stat deltas."""
    node = _FakeNode(now_ns=2_000_000_000)
    publisher = DiagnosticsPublisher(node)
    publisher.publish(
        runtime_stats=RuntimeStats(frames_in=10, frames_out=5),
        last_frame_time=Time(nanoseconds=1_950_000_000),
        last_exception=None,
    )

    node.clock.now_ns = 3_000_000_000
    publisher.publish(
        runtime_stats=RuntimeStats(frames_in=30, frames_out=25),
        last_frame_time=Time(nanoseconds=2_950_000_000),
        last_exception=RuntimeError('stream failed'),
    )

    message = node.publisher.last_message
    assert message is not None
    status = message.status[0]
    values = _value_map(status)

    assert status.level == DiagnosticStatus.ERROR
    assert status.message == 'runtime error'
    assert values['input_fps'] == '20.00'
    assert values['output_fps'] == '20.00'
    assert values['last_exception'] != ''
