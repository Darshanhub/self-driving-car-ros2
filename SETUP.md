# Environment setup guide

This document captures everything required to bring the ROS 2-based self-driving stack online on Ubuntu 22.04 / ROS 2 Humble. Dataset paths and pretrained weights remain intentionally blank placeholders—you can plug them in later without touching the code.

## 1. System prerequisites

1. **Ubuntu 22.04 + CUDA drivers** (if you plan to accelerate inference on NVIDIA GPUs).
2. **ROS 2 Humble** with desktop variant:
   - Follow the [official install guide](https://docs.ros.org/en/humble/Installation.html).
   - Add ROS environment sourcing to your shell, for example:
     ```bash
     echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
     ```
3. **Webots R2023b or newer** plus `webots_ros2` packages:
   - Install from [Cyberbotics instructions](https://cyberbotics.com/doc/guide/installing-webots).
   - Ensure `webots_ros2` workspace builds and the Tesla example runs before moving on.
4. **Isaac Sim / Isaac ROS bridge (optional)** if you plan to benchmark there. No repo changes are needed; just ensure the ROS bridge publishes the same camera/LiDAR topic names used here.

## 2. Workspace layout

Clone this repository inside your ROS workspace (e.g., `~/ros2_ws/src/self-driving-car-ros2`). After cloning, run:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` covers ROS-specific dependencies such as `rclpy`, `vision_msgs`, `message_filters`, and `cv_bridge`.

## 3. Python virtual environment

All non-ROS Python packages (Torch, Ultralytics, profiling libs, etc.) are captured in `requirements.txt`. Use the provided helper script to create a virtual environment that coexists with ROS:

```bash
chmod +x scripts/setup_python_env.sh
./scripts/setup_python_env.sh  # optionally pass a custom path, e.g., ./scripts/setup_python_env.sh ~/ros_envs/self_driving
source .venv/bin/activate
```

The script installs CPU builds of PyTorch by default. For GPU acceleration, reinstall Torch with the appropriate CUDA wheels once the environment is activated, for example:

```bash
pip install --upgrade torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

## 4. Building the ROS workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Rebuild whenever you change packages under `ros2-packages/`.

## 5. Dataset and weights placeholders

- **nuScenes / Waymo / Argoverse** datasets are not bundled. Decide on a host path (e.g., `/data/nuscenes`) and keep it consistent. The helper script `autonomous_perception.scripts.nuscenes_to_mcap` expects two arguments—`--dataset-root` and `--split`—but currently only writes a stub file until you plug in the official nuScenes devkit conversion.
- **Pretrained BEV/occupancy weights**: drop your ONNX/TensorRT/TorchScript files under `ros2-packages/autonomous_perception/config/weights/` and point `config/bevfusion.yaml`’s `model_path` parameter to the exact filename. Leaving the field blank (default) keeps the node in mock mode so you can verify the plumbing without the heavy model.

## 6. Launch sequence (simulation example)

1. **Launch Webots + Tesla + RViz**
   ```bash
   ros2 launch webots_ros2_tesla robot_launch.py use_sim_time:=true
   ```
2. **Start perception stack** (choose mock vs. actual engine through YAML)
   ```bash
   ros2 launch autonomous_perception autonomous_perception_launch.py \
     params_file:=install/autonomous_perception/share/autonomous_perception/config/bevfusion.yaml \
     use_sim_time:=true
   ```
3. **Run autonomous controller**
   ```bash
   ros2 run autonomous_controller autonomous_controller --ros-args -p use_perception_topics:=true
   ```

## 7. Debugging & profiling

- Toggle intermediate dumps for perception:
  ```bash
  ros2 param set /bev_fusion_node debug_dump.enable true
  ```
  Dumps land under `~/.ros/autonomous_perception/debug/`.
- Log resource usage for latency and throughput comparisons:
  ```bash
  source .venv/bin/activate
  python -m autonomous_perception.scripts.profile_runner \
    --command "ros2 launch autonomous_perception autonomous_perception_launch.py" \
    --duration 120 \
    --output profiling/server_run.csv
  ```

## 8. Next steps once data/weights arrive

1. Update `config/bevfusion.yaml` with your actual sensor intrinsics/extrinsics and `model_path`.
2. Replace the placeholder MCAP converter with a call into your nuScenes devkit so perception can be evaluated offline.
3. Collect accuracy metrics (mAP/NDS/occupancy IoU) and latency/resource CSVs for Jetson, laptop, and server targets.

This guide deliberately leaves dataset locations and weight filenames blank to keep the repo portable—you can feed them in later without modifying the source tree.
