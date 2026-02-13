# hand_tracking_sdk_ros2

ROS 2 bridge package for [`hand-tracking-sdk`](https://github.com/wengmister/hand-tracking-sdk).

>[!IMPORTANT]
>This repository is under active development. Expect breaking changes to come.

## Core Dependency

This package treats `hand-tracking-sdk` as a required runtime dependency and
wraps its high-level APIs (`HTSClient`, `HandFrame`, conversion helpers)
instead of reimplementing parser/transport logic.

## Python / ROS Distro Compatibility

- `Jazzy`: primary tested distro.
- `Humble`: supported target (SDK now supports Python `>=3.10`).
- `Kilted`: compatibility target via periodic smoke checks.

## Local Install Notes

Install dependency in the same Python interpreter used by ROS:

```bash
python3 -m pip install "hand-tracking-sdk>=1.0.0,<2.0.0"
```

## Build

```bash
cd ros-ws
colcon build --symlink-install --packages-select hand_tracking_sdk_ros2
source install/setup.bash
```

## Run

Run bridge with default config:

```bash
ros2 launch hand_tracking_sdk_ros2 bridge.launch.py
```

Run bridge + RViz:

```bash
ros2 launch hand_tracking_sdk_ros2 view_hands.launch.py
```

`view_hands.launch.py` forces `qos_reliability:=reliable` for RViz compatibility.

## Topics

- `/hands/left/wrist_pose` (`geometry_msgs/PoseStamped`)
- `/hands/right/wrist_pose` (`geometry_msgs/PoseStamped`)
- `/hands/left/landmarks` (`geometry_msgs/PoseArray`)
- `/hands/right/landmarks` (`geometry_msgs/PoseArray`)
- `/hands/left/markers` (`visualization_msgs/MarkerArray`)
- `/hands/right/markers` (`visualization_msgs/MarkerArray`)
- `/hands/joint_names` (`std_msgs/String`, comma-separated canonical order)

## Default Behavior

- Frame normalization:
  - Unity-left input is mapped directly to FLU for published poses/TF/markers
- Landmark semantics:
  - `landmarks_are_wrist_relative: true` (landmarks are transformed into world frame before publish)
- TF:
  - wrist TF is published to `/tf` as `world -> left_wrist|right_wrist`
- Marker visualization:
  - one `SPHERE_LIST` marker plus one `LINE_LIST` marker per hand
  - left hand: blue, right hand: red
- Stream toggles:
  - `enable_pose_array: false` by default (use markers for RViz visualization)
- QoS:
  - bridge default is `best_effort` (`bridge.params.yaml`)
  - RViz launch overrides to `reliable`

## QoS Recommendations

- `best_effort` (recommended default for live tracking):
  - Better for high-rate streams (~70 Hz) on unstable Wi-Fi.
  - Minimizes latency and avoids back-pressure from retransmits.
- `reliable`:
  - Better for wired LAN/localhost and tooling sessions where delivery completeness matters.
  - Recommended when launching RViz with `view_hands.launch.py`.

## Parameters

The default parameter file is `config/bridge.params.yaml`.

| Parameter | Type | Default | Notes |
|---|---|---:|---|
| `transport_mode` | `string` | `tcp_server` | SDK transport mode (`udp`, `tcp_server`, `tcp_client`). |
| `host` | `string` | `0.0.0.0` | Bind/connect host according to transport mode. |
| `port` | `int` | `5555` | Transport port. |
| `timeout_s` | `float` | `1.0` | SDK socket read/connect timeout seconds. |
| `reconnect_delay_s` | `float` | `0.25` | TCP client reconnect delay seconds. |
| `world_frame` | `string` | `world` | Parent frame used for wrist TF and world-space landmarks. |
| `left_wrist_frame` | `string` | `left_wrist` | Child TF frame for left wrist. |
| `right_wrist_frame` | `string` | `right_wrist` | Child TF frame for right wrist. |
| `use_source_frame_id` | `bool` | `false` | Use incoming `frame_id` from SDK frame when present. |
| `landmarks_are_wrist_relative` | `bool` | `true` | Rotate/translate landmarks by wrist pose before publish. |
| `qos_reliability` | `string` | `best_effort` | `best_effort` or `reliable`. |
| `queue_size` | `int` | `256` | Runtime frame queue size before oldest-frame drop. |
| `enable_tf` | `bool` | `true` | Enable wrist TF publishing. |
| `enable_pose_array` | `bool` | `false` | Enable `/hands/*/landmarks` `PoseArray` topics. |
| `enable_markers` | `bool` | `true` | Enable `/hands/*/markers` `MarkerArray` topics. |
| `enable_diagnostics` | `bool` | `true` | Enable `/diagnostics` publishing. |
| `diagnostics_period_s` | `float` | `1.0` | Diagnostics publish period seconds. |

## Verification

Build and test package:

```bash
cd ros-ws
colcon build --symlink-install --packages-select hand_tracking_sdk_ros2
colcon test --packages-select hand_tracking_sdk_ros2 --event-handlers console_direct+
colcon test-result --verbose --all
```

Runtime verification:

```bash
source ros-ws/install/setup.bash
ros2 launch hand_tracking_sdk_ros2 bridge.launch.py
```

In another shell:

```bash
source ros-ws/install/setup.bash
ros2 topic list | grep hands
ros2 topic hz /hands/left/markers
ros2 topic echo /hands/joint_names --once
ros2 run tf2_ros tf2_echo world left_wrist
```

RViz bring-up:

```bash
source ros-ws/install/setup.bash
ros2 launch hand_tracking_sdk_ros2 view_hands.launch.py
```

## Troubleshooting

- No hand topics beyond `/rosout` and `/parameter_events`:
  - Confirm bridge process stays alive in launch output.
  - Verify SDK transport settings (`transport_mode`, `host`, `port`) match sender.
- RViz shows no markers:
  - Use `view_hands.launch.py` (it overrides bridge QoS to `reliable`).
  - Check marker stream exists: `ros2 topic echo /hands/left/markers --once`.
- Visualization feels delayed:
  - Keep `enable_pose_array=false`.
  - Prefer marker-only visualization and `best_effort` for non-RViz runs.
- TF missing:
  - Ensure `enable_tf=true`.
  - Confirm frame names: `world`, `left_wrist`, `right_wrist`.
- Python dependency mismatch:
  - Install `hand-tracking-sdk` into the same interpreter/environment as ROS 2.

## Architecture

- `bridge_node.py`: node orchestration, params, publications, diagnostics.
- `runtime.py`: background SDK ingest with bounded queue.
- `adapters.py`: deterministic SDK frame -> ROS message mapping.
- `markers.py`: landmark graph definitions for marker rendering.
