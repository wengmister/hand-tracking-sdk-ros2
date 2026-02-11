# hand_tracking_sdk_ros2

ROS 2 bridge package for `hand-tracking-sdk`.

## Core Dependency

This package treats `hand-tracking-sdk` as a required runtime dependency and
wraps its high-level APIs (`HTSClient`, `HandFrame`, conversion helpers)
instead of reimplementing parser/transport logic.

Supported SDK range:
- `hand-tracking-sdk>=1.0.0,<2.0.0`

## Python / ROS Distro Compatibility

- `Jazzy`: primary tested distro (Python 3.12 baseline aligns with SDK).
- `Humble`: target compatibility, but currently blocked by upstream SDK
  requirement `python>=3.12`.
- `Kilted`: compatibility target via periodic smoke checks.

Until the upstream SDK supports Python 3.10 (or Humble runs with 3.12 in your
environment), Humble support should be considered source-level only.

## Local Install Notes

Install dependency in the same Python interpreter used by ROS:

```bash
python3 -m pip install "hand-tracking-sdk>=1.0.0,<2.0.0"
```
