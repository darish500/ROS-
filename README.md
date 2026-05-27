# Line Follower Robot — ROS2 Jazzy + Gazebo Harmonic

A **differential-drive robot** that autonomously follows a black line on a
white track, simulated entirely in **Gazebo Harmonic (gz-sim 8)** and
controlled by a **ROS2 Jazzy** node.

---

## 📁 Package Structure

```
line_follower_robot/
├── CMakeLists.txt
├── package.xml
│
├── urdf/
│   └── line_follower_robot.urdf.xacro   ← robot description (chassis, wheels, camera)
│
├── worlds/
│   └── line_track.sdf                   ← Gazebo world with white floor + black oval track
│
├── launch/
│   ├── simulation.launch.py             ← MAIN launch file (Gazebo + robot + bridge + controller)
│   └── rviz.launch.py                   ← optional RViz2 visualiser
│
├── config/
│   └── line_follower.rviz               ← RViz2 config (camera feeds, TF, odometry)
│
└── line_follower_robot/
    ├── __init__.py
    └── line_follower_node.py            ← the controller (OpenCV + proportional control)
```

---

## 🔧 Prerequisites

| Tool | Version |
|------|---------|
| Ubuntu | 24.04 LTS (Noble) |
| ROS2 | Jazzy |
| Gazebo | Harmonic (gz-sim 8) |
| Python | 3.12 |

### Install required ROS packages (if not already installed)

```bash
sudo apt update

# Core Gazebo-ROS bridge
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim

# Robot description tools
sudo apt install -y \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro

# OpenCV bridge
sudo apt install -y \
  ros-jazzy-cv-bridge \
  python3-opencv

# RViz2 (optional but recommended)
sudo apt install -y ros-jazzy-rviz2
```

---

## 🏗️ Build

```bash
# 1. Source your ROS2 installation
source /opt/ros/jazzy/setup.bash

# 2. Create / go to workspace
mkdir -p ~/ros2_ws/src
cp -r line_follower_robot ~/ros2_ws/src/

# 3. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --packages-select line_follower_robot --symlink-install

# 5. Source the workspace
source install/setup.bash
```

---

## 🚀 Run

### Terminal 1 — launch everything
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch line_follower_robot simulation.launch.py
```

This single command:
- Starts **Gazebo Harmonic** with the line track world
- Spawns the robot at the **start marker** (bottom straight, x=0, y=-0.8)
- Starts the **ros_gz_bridge** (camera, cmd_vel, odom, tf, joint_states, clock)
- Starts the **line_follower_node** (reads camera → publishes /cmd_vel)

> **Important**: In the Gazebo window, press the **▶ Play** button to start the simulation.

### Terminal 2 — RViz2 (optional)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch line_follower_robot rviz.launch.py
```

---

## 🔍 Useful Monitoring Commands

```bash
# Watch velocity commands being published
ros2 topic echo /cmd_vel

# View available topics
ros2 topic list

# Check camera is publishing
ros2 topic hz /line_camera/image

# View the raw camera image
ros2 run rqt_image_view rqt_image_view /line_camera/image

# View the debug image (shows detected line centroid)
ros2 run rqt_image_view rqt_image_view /line_camera/debug
```

---

## ⚙️ How It Works

### Robot Hardware
| Component | Description |
|-----------|-------------|
| Chassis | 20 × 15 × 5 cm blue box |
| Drive wheels | Two continuous-rotation wheels (left + right), r = 3.3 cm |
| Caster | One passive front sphere |
| Camera | 160 × 120 px, 60° FOV, facing straight down, 15 Hz |

### Line Detection Algorithm
1. The downward camera captures a 160×120 image of the floor below the robot.
2. The image is converted to grayscale.
3. A binary threshold (`black_thresh = 80`) isolates dark (black tape) pixels.
4. A horizontal scan row at 75% of the image height is examined.
5. The **centroid** of black pixels is found in that row.
6. The **pixel error** = `centroid_x − image_centre_x` is computed.

### Proportional Controller
```
angular_z = -Kp × error        (turns toward the line)
linear_x  = base_speed − decay × |error|   (slows on curves)
```

| Parameter | Default | Effect |
|-----------|---------|--------|
| `base_speed` | 0.15 m/s | Forward speed |
| `kp` | 0.005 | Turning sensitivity |
| `black_thresh` | 80 | Line detection sensitivity (0-255) |

If the line is lost for > 30 frames the robot stops and logs a warning.

---

## 🗺️ Track Layout

```
  ┌─────────────────────────────────────┐   y = +0.8
  │                                     │
  │                                     │   (3m × 1.6m oval loop)
  │                                     │
  └──────────┬──────────────────────────┘   y = -0.8
             │
           [🟢 START]   x=0, y=-0.8
```

The robot spawns at the **green circle** (start marker) on the bottom straight,
facing the **+X direction** (east), and should immediately detect the black line
and start following it counter-clockwise.

---

## 🐛 Troubleshooting

| Symptom | Fix |
|---------|-----|
| Gazebo opens but robot doesn't appear | Wait ~5 s for spawn; check `ros2 topic echo /robot_description` |
| Camera topic not publishing | Press ▶ Play in Gazebo; sensors only run when simulation is playing |
| `/cmd_vel` published but robot doesn't move | Check bridge is running: `ros2 topic list \| grep cmd_vel` inside gz: `gz topic -l` |
| Line detection fails (robot spins) | Tune `black_thresh` — try `ros2 param set /line_follower_node black_thresh 60` |
| `cv_bridge` not found | `sudo apt install ros-jazzy-cv-bridge` |
| Robot falls through ground | Increase `z_spawn` argument: `ros2 launch ... z_spawn:=0.1` |

---

## 🔮 Next Steps (toward SLAM)

Once you're comfortable with this simulation:

1. **Add a LiDAR sensor** to the robot URDF (see Gazebo `gpu_lidar` sensor type)
2. **Add a map** — import or create an obstacle-filled world
3. **Install Nav2 + SLAM Toolbox**: `sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup`
4. **Run SLAM**: `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true`
5. **Visualise** the map building in RViz2 in real time
