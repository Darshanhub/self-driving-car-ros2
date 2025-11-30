# autonomous_perception

A ROS 2 (Humble) package that hosts pretrained BEV / occupancy / detection models and publishes structured perception outputs for the Tesla Webots stack or Isaac ROS bridges.

## Features

- Loads ONNX, TorchScript, or TensorRT engines for models such as BEVFusion, Occupancy Networks, or VoxelNext via a pluggable `BEVFusionEngine` interface.
- Synchronizes multi-camera RGB streams and LiDAR point clouds, performs inference, and publishes:
  - `vision_msgs/Detection3DArray` (3D boxes + classes)
  - `nav_msgs/OccupancyGrid` (bird's-eye occupancy probability)
  - `diagnostic_msgs/DiagnosticArray` (latency, throughput, CPU/GPU/memory)
- Optional path-proposal channel for the `autonomous_controller` package to consume.
- Dataset tooling to convert nuScenes logs into ROS 2 MCAP bags for offline replay plus a profiling logger for latency/resource evaluation.
- Toggle-able debug dumps: set `debug_dump.enable:=true` (via params or YAML) to persist intermediate tensors/metadata so experiments can be replayed and compared later.

## Quick start

```bash
colcon build --symlink-install --packages-select autonomous_perception
source install/setup.bash
ros2 launch autonomous_perception autonomous_perception_launch.py use_sim_time:=true
```

Drop your pretrained weights under `config/weights/` or pass an absolute path via `model_path` parameter. See `config/bevfusion.yaml` for calibration placeholders and tunables.

## Dataset + profiling utilities

- `autonomous_perception.scripts.nuscenes_to_mcap`: adapt this helper to your nuScenes devkit conversion flow to emit ROS 2 MCAP logs ready for `ros2 bag play`.
- `autonomous_perception.scripts.profile_runner`: wrap any launch command and log CPU/RAM stats to CSV; combine with `/perception/diagnostics` for GPU metrics.
