"""Background runtime for ingesting SDK frames into a bounded queue."""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass

from hand_tracking_sdk import (
    HTSClient,
    HTSClientConfig,
    StreamOutput,
    TransportMode,
    convert_hand_frame_unity_left_to_right,
)
from hand_tracking_sdk.frame import HandFrame


@dataclass(frozen=True, slots=True)
class RuntimeStats:
    """Bridge runtime counters."""

    frames_in: int = 0
    frames_dropped_queue: int = 0
    frames_out: int = 0
    loop_errors: int = 0


class FrameRuntime:
    """Owns an SDK client loop and buffers frames for ROS publication."""

    def __init__(
        self,
        *,
        transport_mode: str,
        host: str,
        port: int,
        timeout_s: float,
        reconnect_delay_s: float,
        convert_to_right_handed: bool,
        queue_size: int,
    ) -> None:
        self._convert_to_right_handed = convert_to_right_handed
        self._queue: deque[HandFrame] = deque(maxlen=queue_size)
        self._queue_lock = threading.Lock()
        self._stats = RuntimeStats()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_exception: Exception | None = None

        client_config = HTSClientConfig(
            transport_mode=TransportMode(transport_mode),
            host=host,
            port=port,
            timeout_s=timeout_s,
            reconnect_delay_s=reconnect_delay_s,
            output=StreamOutput.FRAMES,
            include_wall_time=True,
        )
        self._client = HTSClient(client_config)

    def start(self) -> None:
        """Start background ingest thread once."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True, name="hts-frame-runtime")
        self._thread.start()

    def stop(self) -> None:
        """
        Request background thread stop.

        Note: SDK iterators can block in transport receive. Thread is daemonized
        so process shutdown still completes cleanly.
        """
        self._stop_event.set()

    def pop_frame(self) -> HandFrame | None:
        """Pop oldest buffered frame, if any."""
        with self._queue_lock:
            if not self._queue:
                return None
            frame = self._queue.popleft()
        self._stats = RuntimeStats(
            frames_in=self._stats.frames_in,
            frames_dropped_queue=self._stats.frames_dropped_queue,
            frames_out=self._stats.frames_out + 1,
            loop_errors=self._stats.loop_errors,
        )
        return frame

    def get_stats(self) -> RuntimeStats:
        """Return runtime counters snapshot."""
        return self._stats

    def get_last_exception(self) -> Exception | None:
        """Return last fatal loop exception, if any."""
        return self._last_exception

    def _run(self) -> None:
        try:
            for event in self._client.iter_events():
                if self._stop_event.is_set():
                    return

                frame = event
                if self._convert_to_right_handed:
                    frame = convert_hand_frame_unity_left_to_right(frame)

                dropped = 0
                with self._queue_lock:
                    if len(self._queue) == self._queue.maxlen:
                        self._queue.popleft()
                        dropped = 1
                    self._queue.append(frame)

                self._stats = RuntimeStats(
                    frames_in=self._stats.frames_in + 1,
                    frames_dropped_queue=self._stats.frames_dropped_queue + dropped,
                    frames_out=self._stats.frames_out,
                    loop_errors=self._stats.loop_errors,
                )
        except Exception as exc:  # pragma: no cover - integration path
            self._last_exception = exc
            self._stats = RuntimeStats(
                frames_in=self._stats.frames_in,
                frames_dropped_queue=self._stats.frames_dropped_queue,
                frames_out=self._stats.frames_out,
                loop_errors=self._stats.loop_errors + 1,
            )
