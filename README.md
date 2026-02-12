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

## Run

Build and source workspace:

```bash
cd ros-ws
colcon build --symlink-install --packages-select hand_tracking_sdk_ros2
source install/setup.bash
```

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
  - `convert_to_right_handed: true`
  - `map_to_flu: true`
- Landmark semantics:
  - `landmarks_are_wrist_relative: true` (landmarks are transformed into world frame before publish)
- TF:
  - wrist TF is published to `/tf` as `world -> left_wrist|right_wrist`
- Marker visualization:
  - per-joint `SPHERE` markers plus `LINE_LIST` bones
  - left hand: blue, right hand: red
- QoS:
  - bridge default is `best_effort` (`bridge.params.yaml`)
  - RViz launch overrides to `reliable`

## Key Parameters

- `transport_mode`: `tcp_server` or `udp_client`
- `host`, `port`, `timeout_s`, `reconnect_delay_s`
- `world_frame`, `left_wrist_frame`, `right_wrist_frame`
- `convert_to_right_handed`, `map_to_flu`, `landmarks_are_wrist_relative`
- `qos_reliability`: `best_effort` or `reliable`
- `enable_tf`, `enable_pose_array`, `enable_markers`, `enable_diagnostics`
- `diagnostics_period_s`, `queue_size`

## Architecture

- `bridge_node.py`: node orchestration, params, publications, diagnostics.
- `runtime.py`: background SDK ingest with bounded queue.
- `adapters.py`: deterministic SDK frame -> ROS message mapping.
- `markers.py`: landmark graph definitions for marker rendering.
