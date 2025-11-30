## Self Driving Car Project

![yz-proje-final](https://github.com/abdulkadrtr/self-driving-car-ros2/assets/87595266/1473a43b-d1fa-426f-aeba-69c99c8a5eae)

This project involves a self-driving car that learns from manual driving data using an artificial neural network model. A route consisting of GPS waypoints is defined for the vehicle, 
and it follows this route using the neural network model. If an obstacle appears while the vehicle is progressing, the neural network model stops the vehicle. Red traffic lights are 
detected using the YOLOv8 model and reported to the neural network, which then stops the vehicle and waits for a green light. The neural network model dynamically makes decisions 
based on environmental conditions, autonomously driving the vehicle to its target destination. For more technical details, please refer to the `report/rapor.pdf` file.

## Demo & Algorithm Description

https://youtu.be/3fzhcxWPNmQ?si=NHlqoIhmli0MQffu

## Installation Instructions


This project has been tested using `Ubuntu 22.04`, `ROS 2 Humble`, and the `Webots` simulation environment. Please ensure that your system meets these requirements.

- For a step-by-step walkthrough (ROS install, Webots, Python virtual environment, debugging tools) read `SETUP.md`. It also explains where to drop nuScenes/Waymo/Argo datasets and pretrained weights once you have them.
- To provision the Python environment quickly, run `scripts/setup_python_env.sh` (after `chmod +x`) which installs the packages listed in `requirements.txt` into `.venv/` by default.

- First, install the `webots_ros2` packages on your system and verify that the `webots_ros2_tesla` package is functioning correctly.

- Next, load the packages from the `ros2-packages` directory into the `ros2_ws/src` folder. 
The `webots_ros2_tesla` package is a modified Tesla package, while the `autonomous_controller` package is the autonomous driving package. 
Transfer these packages into your working environment and compile them using `colcon build`.

- Remember! You need to install the dependencies for the webots_ros2_tesla package from the original webots_ros2 packages.

- You are now ready to run the autonomous driving application. First, open the simulation environment and the RVIZ2 visualization window by running the command:
  
  `ros2 launch webots_ros2_tesla robot_launch.py`

- Next, start the autonomous driving by running the command:
  
  `ros2 run autonomous_controller autonomous_controller`


  From this point on, you can view feedback related to autonomous driving in the RVIZ2 window and the terminal screen. The vehicle will stop when it sees a red light and wait for it to turn green.
  If an obstacle appears, the vehicle will stop without hitting it and continue once the obstacle is removed.
  When the navigation reaches the main target, the autonomous driving speed will be set to zero, stopping the vehicle.

- Three different GPS routes have been added for autonomous driving test.
  You can track the autonomous driving of the vehicle on different routes by replacing the code line below with `path1`, `path2`, and `path3` respectively.

  https://github.com/abdulkadrtr/self-driving-car-ros2/blob/44907a0afea1c2cd980fb15e2d1e210d733d41ae/ros2-packages/autonomous_controller/autonomous_controller/autonomous_controller.py#L16

## More

- The `autonomous-ai-model` directory contains a dataset with manual driving data and a training file for the neural network model. You are free to use these files as needed.
- In the `detection-model` directory, you can find the training codes for the YOLOv8 model, as well as the Roboflow link for the traffic lights dataset prepared for this project.
  Feel free to utilize these resources accordingly.
- External datasets (nuScenes, Waymo, Argoverse, etc.) and pretrained BEV weights are intentionally **not** committed—leave the placeholders in `SETUP.md`/`config/bevfusion.yaml` empty until you provide the paths locally.

## Autonomous perception add-on (BEV Fusion + Occupancy)

- A new ROS 2 package `autonomous_perception` now wraps state-of-the-art pretrained BEV/occupancy models (start with the bundled mock engine, then drop BEVFusion/Occupancy weights under `ros2-packages/autonomous_perception/config/weights`).
- It synchronizes the six Tesla cameras and LiDAR topics, performs inference, and publishes:
  - `/perception/detections_3d` (`vision_msgs/Detection3DArray`)
  - `/perception/occupancy` (`nav_msgs/OccupancyGrid`, ego-centric bird's-eye grid)
  - `/perception/diagnostics` (latency + CPU/GPU/RAM stats)
- The `autonomous_controller` node now subscribes to these topics. Occupancy-positive regions ahead of the ego car are fused with LiDAR for obstacle braking, and the controller logs when perception blocks motion.
- Intermediate tensors/metadata can be dumped by toggling the `debug_dump.enable` parameter or editing `config/bevfusion.yaml`. This honors the user's request for traceable checkpoints during debugging.

### Running the stack (simulation or dataset replay)

```bash
colcon build --symlink-install
source install/setup.bash

# Launch Webots + Tesla sensors
ros2 launch webots_ros2_tesla robot_launch.py use_sim_time:=true

# In another terminal: spin up perception (choose mock/BEV engine via params)
ros2 launch autonomous_perception autonomous_perception_launch.py params_file:=install/autonomous_perception/share/autonomous_perception/config/bevfusion.yaml use_sim_time:=true

# Finally run the controller (consumes perception topics automatically)
ros2 run autonomous_controller autonomous_controller --ros-args -p use_perception_topics:=true
```

### nuScenes / Isaac workflows

- Place your nuScenes dataset path somewhere accessible (e.g., `/data/nuscenes`). The conversion helper `autonomous_perception.scripts.nuscenes_to_mcap` currently writes a placeholder MCAP; point it to the official nuScenes devkit export you prefer and update the script if you need richer topics.
- After producing an MCAP/rosbag, replay with `ros2 bag play your.mcap --clock` while running the perception + controller launch files above to evaluate accuracy offline.
- For Isaac Sim, bridge the camera/LiDAR topics (topic names already match what the perception node expects); no further code changes are necessary.

### Evaluation + profiling

- Use `autonomous_perception.scripts.profile_runner --command "ros2 launch ..."` to log CPU/RAM usage into CSV for Jetson/laptop/server comparison. GPU utilization automatically publishes via `/perception/diagnostics` (requires `pynvml`).
- Accuracy: connect nuScenes ground-truth to the same evaluation scripts shipped with your pretrained model (BEVFusion, Occupancy). Store summary tables/plots under `report/` for presentations.
- Latency: each inference publishes diagnostics plus you can enable the debug dump to persist timestamps. For quick traces, flip `debug_dump.enable:=true` either via YAML or runtime parameter to capture JSON snapshots per N frames.

## Not found: NeuralNetwork issue solution

Due to the incompatibility between `ROS 2` and `torch.load`, you are likely to encounter this error. 
Therefore, after compiling the project with colcon build, paste the following code into the `install/autonomous_controller/lib/autonomous_controller/autonomous_controller.py` file. 
This will resolve the issue.
```
import torch
import torch.nn as nn
class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.layer1 = nn.Linear(6, 32)
        self.layer2 = nn.Linear(32, 16)
        self.output = nn.Linear(16, 2)

    def forward(self, x):
        x = torch.relu(self.layer1(x))
        x = torch.relu(self.layer2(x))
        x = self.output(x)
        return x
```

## Running the ROS 2 stack

1. Install Docker (with the Compose plugin) plus the NVIDIA Container Toolkit/X11 forwarding if you plan to use GPU visualization.
2. After cloning, ensure the helper is executable and launch everything:
   ```bash
   chmod +x scripts/dev.sh
   ./scripts/dev.sh
   ```
3. To mount custom datasets/models, prefix the script with env vars such as  
   `HOST_DATASET_PATH=/mnt/datasets HOST_MODEL_PATH=/mnt/models ./scripts/dev.sh`.
4. Open a shell inside the running container for ROS tools:  
   `docker compose exec ros2-dev bash`.
5. When finished, stop the stack via `docker compose down` (add `-v` if volumes should be removed).
