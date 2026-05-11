# Send_HEVC ROS2 Workspace

This repository is now a ROS2 workspace rooted at `Send_HEVC-`.

## Layout

- `src/send_lb_capture_preprocess`
- `src/send_lb_encoder`
- `src/send_lb_transmitter`
- `third_party/`

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Run

```bash
source install/setup.bash
ros2 launch send_lb_capture_preprocess send_lb.launch.py
```

## Topics

- `/send_lb/image_preprocessed`
- `/send_lb/hevc_stream`

## Parameters

- `source_type:=file|camera|hikrobot`
- `transport_mode:=udp|serial`
- `slice_mode:=fixed|mtu|whole`

## VS Code Tasks

在 VS Code 里可以直接运行这些 task：

- `ROS2: Build all`
- `ROS2: Launch capture_preprocess`
- `ROS2: Launch encoder`
- `ROS2: Launch transmitter`

每个 task 都会单独打开一个终端，不会共用同一个窗口。
