# TurtleBot3 Patrol with Open-RMF + Free Fleet

Multi-robot simulation with TurtleBot3 for patrolling tasks using Open-RMF and Free Fleet.

## Demo

![Patrol Demo](patrol_demo.gif)

> Video recorded with a phone camera because Ubuntu's built-in screen recorder competes for GPU resources with Gazebo and RViz, causing the simulation to stall or run extremely slowly. Using an external recording avoids this performance issue. Playback is at 3.5x speed.

## General Architecture

The architecture has several layers:

1. **Simulation (Nav2 + Gazebo)**: Spawns the TurtleBots in a simulated world with autonomous navigation (Nav2).
2. **Zenoh Router**: Lightweight pub/sub communication middleware that connects the ROS2 side of the robots with the Open-RMF side.
3. **Zenoh Bridge (ros2dds)**: Translates ROS2 (DDS) messages to Zenoh messages and vice versa. This allows Free Fleet to communicate with the robots without being in the same ROS2 domain.
4. **Free Fleet**: Python implementation of the Open-RMF Fleet Adapter. It receives robot positions via Zenoh and sends navigation commands.
5. **RMF Core**: Core Open-RMF packages (scheduling, traffic negotiation, task dispatch).
6. **API Server + Dashboard**: Web interface to visualize robots and dispatch tasks (patrol, etc.).

```
+---------------+    +------------+    +---------------+    +------------------+
|  TurtleBot    |<-->|  Zenoh     |<-->|  Zenoh Router |<-->|  Free Fleet      |
|  Nav2         |    |  Bridge    |    |               |    |  Adapter + RMF   |
|  (Gazebo)     |    |  ros2dds   |    |               |    |  Core            |
+---------------+    +------------+    +---------------+    +------------------+
                                                                  |
                                                            +-----+------+
                                                            | API Server |
                                                            | Dashboard  |
                                                            | :3000      |
                                                            +------------+
```

In the multi-robot simulation, there is a single zenoh bridge without namespace that sees all robots (robot1, robot2) because they are in the same ROS2 simulation.

---

## Docker Images

```bash
# TurtleBot3 simulation + Nav2 + Gazebo
docker pull ghcr.io/open-rmf/free_fleet/minimal-nav2-bringup:jazzy-latest

# Zenoh <-> ROS2 DDS bridge
docker pull ghcr.io/open-rmf/free_fleet/minimal-zenoh-bridge-ros2dds:jazzy-latest

# Zenoh router (communication middleware)
docker pull eclipse/zenoh

# RMF Demos (contains free_fleet, RMF core, etc.)
docker pull ghcr.io/open-rmf/rmf/rmf_demos:latest

# API Server (REST/WebSocket backend for the dashboard)
docker pull ghcr.io/open-rmf/rmf-web/api-server:latest

# Web dashboard (React frontend to visualize and dispatch tasks)
docker pull ghcr.io/open-rmf/rmf-web/dashboard:latest
```

---

## Step 1: Launch Nav2 Multi-TurtleBot3 Simulation

> **What it does**: Starts Gazebo (headless) with a world containing 2 TurtleBots (robot1 and robot2), each with its own Nav2 stack (AMCL localization, planner, controller). It also opens 2 RViz windows, one per robot, to visualize the map, laser scan, and send navigation goals.

### Terminal 1: Launch the container

```bash
# Allow X11 connections (so the container can open graphical windows)
xhost +

# Launch container with NVIDIA GPU support and display access
docker run --rm -it --runtime=nvidia --gpus all \
  --net=host --name nav2-bringup \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --entrypoint "bash" \
  ghcr.io/open-rmf/free_fleet/minimal-nav2-bringup:jazzy-latest
```

**Flag explanation:**
- `--runtime=nvidia --gpus all`: Use NVIDIA GPU to render Gazebo/RViz.
- `--net=host`: Share the host network (needed so containers can reach each other via localhost).
- `--env="DISPLAY"`: Pass the DISPLAY variable to open graphical windows.
- `--env="QT_X11_NO_MITSHM=1"`: Avoid shared memory errors with Qt/RViz.
- `--volume="/tmp/.X11-unix:..."`: Mount the X11 socket for window rendering.
- `--entrypoint "bash"`: Enter the container in bash instead of running the default command.

### Inside the container:

```bash
# Install RViz2 (not pre-installed in the minimal image)
apt update && apt -y install ros-jazzy-rviz2

# Load the ROS2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Use CycloneDDS as ROS2 middleware (recommended for Zenoh compatibility)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Limit ROS2 node discovery to localhost (avoids interference with other PCs on the network)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Force rendering on NVIDIA GPU (required on laptops with hybrid Intel+NVIDIA GPU)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Add TurtleBot3 models to Gazebo's model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_simulations/turtlebot3_gazebo/models

# Launch the multi-robot simulation with RViz
ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py use_rviz:=True
```

**What to expect**: 2 RViz windows will open (one per robot). Gazebo runs in headless/server mode (no GUI window), but the simulation runs normally in the background.

### Localize the robots in RViz (IMPORTANT):

The robots spawn at these positions:
- **robot1**: x=0.0, y=0.5 (slightly above center)
- **robot2**: x=0.0, y=-0.5 (slightly below center)

In **each RViz window**:
1. Click the **"Start"** button in the Navigation 2 panel (the map should appear).
2. Click **"2D Pose Estimate"** in the toolbar and click on the map where the robot is, dragging in the direction it's facing.
3. Verify that the **laser scan (green dots) aligns with the walls** of the map. If it doesn't, repeat step 2.

> This is necessary because AMCL needs an initial estimate of where each robot is in order to localize correctly.

---

## Step 2: Launch Zenoh Router

> **What it does**: Zenoh is a lightweight pub/sub communication protocol. The router acts as a central point through which all messages pass between the zenoh-bridge (robot side) and the free_fleet_adapter (RMF side). Without the router, both endpoints cannot discover each other.

### Terminal 2:

```bash
docker run --init --net=host --name zenoh-router eclipse/zenoh
```

**Explanation:**
- `--init`: Use an init process to handle signals correctly (cleans up zombies).
- `--net=host`: Same network as host to communicate with other containers.

**What to expect**: Logs indicating the router is listening on port 7447.

---

## Step 3: Launch Zenoh Bridge (ros2dds)

> **What it does**: The bridge translates between the ROS2 world (DDS protocol) and the Zenoh world. It listens to ROS2 topics from the robots (like `/robot1/tf`, `/robot2/navigate_to_pose`) and republishes them as Zenoh messages with namespaces. This way the free_fleet_adapter can know where the robots are and send them navigation goals without being directly in the ROS2 domain.

> In multi-robot mode, a **single bridge without namespace** is used because both robots are in the same simulation and their topics already come namespaced (`/robot1/...`, `/robot2/...`).

### Terminal 3:

First, clone the free_fleet repo to get the zenoh config files:

```bash
git clone https://github.com/open-rmf/free_fleet
```

Then launch the bridge:

```bash
docker run --rm -it --net=host --entrypoint bash \
  --volume $(pwd)/free_fleet/free_fleet_examples/config/zenoh:/zenoh_config \
  ghcr.io/open-rmf/free_fleet/minimal-zenoh-bridge-ros2dds:jazzy-latest
```

Inside the container:

```bash
./zenoh-bridge/zenoh-bridge-ros2dds \
  -c zenoh_config/nav2_unique_multi_tb3_zenoh_bridge_ros2dds_client_config.json5
```

**What to expect**: Logs showing the bridge connects to the zenoh router and starts discovering ROS2 topics.

---

## Step 4: Set up Free Fleet + RMF Core

> **What it does**: Launches a container with the rmf_demos image, inside which free_fleet (the fleet adapter) is compiled and the RMF core packages are launched (traffic schedule, traffic negotiation, task dispatch, etc.). These packages are the "brain" that coordinates tasks between robots.

### Terminal 4: Launch the container

```bash
docker run --rm -it --net=host --name ff ghcr.io/open-rmf/rmf/rmf_demos:latest bash
```

### Inside the container - Install and compile Free Fleet:

```bash
# Install dependencies
apt update && apt install -y python3-pip ros-jazzy-rmw-cyclonedds-cpp

# Install required Python libraries:
# - eclipse-zenoh: Zenoh client for Python (communication with the bridge)
# - pycdr2: ROS2 message serialization/deserialization (CDR format)
# - rosbags: read/write rosbags
# - nudged: 2D transformations (to convert coordinates between RMF and Nav2)
pip3 install eclipse-zenoh==1.1.0 pycdr2 rosbags nudged --break-system-packages

# Create workspace and clone free_fleet
mkdir -p /root/ff_ws/src && cd /root/ff_ws/src
git clone https://github.com/open-rmf/free_fleet

# Install ROS dependencies and compile
cd .. && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -yr
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Launch RMF Core (same container):

```bash
source /root/ff_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Launches: rmf_traffic_schedule, rmf_traffic_negotiation, task_dispatcher, etc.
ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml
```

**What to expect**: Logs showing RMF nodes starting up (schedule, negotiation, dispatcher).

---

## Step 5: Launch Free Fleet Adapter

> **What it does**: The fleet adapter connects the robots with RMF. It reads each robot's position via Zenoh, reports it to the RMF scheduler, and when RMF assigns a task (patrol, go-to, etc.), it translates that task into navigation goals sent to the robots via Zenoh.

### Terminal 5: Open another shell in the `ff` container

```bash
docker exec -it ff bash
```

### Inside the container:

```bash
source /root/ff_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Workaround: copy the fleet_adapter launch file to where ROS2 can find it
cd /root/ff_ws/install/free_fleet_adapter/share/free_fleet_adapter && \
mkdir -p launch && cp fleet_adapter.launch.xml launch/

# Launch the fleet adapter for multi-TB3 simulation
ros2 launch free_fleet_examples \
  nav2_unique_multi_tb3_simulation_fleet_adapter.launch.xml \
  server_uri:="ws://localhost:8000/_internal"
```

**`server_uri` explanation**: This is the URL of the RMF API Server (step 6). The adapter connects via WebSocket to report robot status and receive tasks.

**What to expect**: Logs showing it detects robot1 and robot2 via Zenoh and registers them in RMF.

---

## Step 6: Launch API Server and Dashboard

> **What it does**: The API Server is a FastAPI backend that exposes a REST + WebSocket API. It receives RMF state (robots, tasks, map) and serves it to the Dashboard. The Dashboard is a React app that shows the map, robot positions, and allows creating tasks (patrol, loop, go-to).

### Terminal 6: API Server

```bash
docker run --network host -it \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ghcr.io/open-rmf/rmf-web/api-server:latest
```

### Terminal 7: Dashboard

```bash
docker run --network host -it \
  -e RMF_SERVER_URL=http://localhost:8000 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e TRAJECTORY_SERVER_URL=ws://localhost:8006 \
  ghcr.io/open-rmf/rmf-web/dashboard:latest
```

**Access the dashboard**: Open browser at `http://localhost:3000`

**What to expect**: The TurtleBot3 world map with robots shown as dots, and the ability to create tasks from the "NEW TASK" button.

---

## Step 7: Dispatch Patrol Tasks

From the Dashboard (`localhost:3000`):

1. Go to the **TASKS** tab.
2. Click **NEW TASK**.
3. Select task type **Patrol** (or **Loop**).
4. Choose start and destination waypoints (the waypoints defined in the nav graph of the map).
5. Submit the task. RMF will assign a robot and it will start navigating.

---

## Step 8: Record rosbag

> **What it does**: Records the navigation topics to a rosbag file for later replay. This serves as evidence that the robots performed the patrol.

### Open another shell in the nav2-bringup container:

```bash
docker exec -it nav2-bringup bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Record navigation topics from both robots
ros2 bag record \
  /robot1/cmd_vel /robot1/odom /robot1/scan /robot1/map /robot1/amcl_pose \
  /robot2/cmd_vel /robot2/odom /robot2/scan /robot2/map /robot2/amcl_pose \
  /tf /tf_static \
  -o /tmp/patrol_rosbag
```

To stop recording: `Ctrl+C`

Then copy the rosbag out of the container:

```bash
# From the host (another terminal)
docker cp nav2-bringup:/tmp/patrol_rosbag ./patrol_rosbag
```

---

## Step 9: Record Screen Video

> **Note**: Ubuntu's built-in screen recorder and tools like OBS use the GPU for encoding, which competes with Gazebo and RViz for GPU resources. This causes the simulation to stall or become extremely slow. **Recording with a phone camera** avoids this issue entirely and is the recommended approach for this setup.

---

## Terminal Summary

| Terminal | Container | What it runs |
|----------|-----------|--------------|
| 1 | nav2-bringup | Gazebo (headless) + Nav2 + RViz (2 TurtleBots) |
| 2 | eclipse/zenoh | Zenoh Router |
| 3 | zenoh-bridge-ros2dds | Zenoh <-> ROS2 Bridge |
| 4 | ff | Free Fleet compile + RMF Core |
| 5 | ff (exec) | Free Fleet Adapter |
| 6 | api-server | RMF API Server (port 8000) |
| 7 | dashboard | Web Dashboard (port 3000) |
| 8 | nav2-bringup (exec) | ros2 bag record |

---

## Rosbag Contents

The recorded rosbag (`patrol_rosbag/`) contains ~16k messages across these topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/robot1/cmd_vel`, `/robot2/cmd_vel` | `geometry_msgs/Twist` | Velocity commands sent to each robot |
| `/robot1/odom`, `/robot2/odom` | `nav_msgs/Odometry` | Odometry (position + velocity) from each robot |
| `/robot1/scan`, `/robot2/scan` | `sensor_msgs/LaserScan` | Laser scan data from each robot |
| `/robot1/map`, `/robot2/map` | `nav_msgs/OccupancyGrid` | Navigation map for each robot |
| `/robot1/amcl_pose`, `/robot2/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Localized pose from AMCL |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Transform tree (coordinate frames) |

---

## Troubleshooting
- **Screen recording causes simulation to stall**: The GPU is shared between rendering and encoding. Record with a phone camera instead.
