# First Project with Open-RMF

An Open-RMF project with two simulation worlds (**test1** and **ICC Kyoto**), featuring a TinyRobot fleet managed through a custom fleet adapter, a Gazebo simulation, and the RMF web dashboard.

## Project Structure

```
src/project/
  project_assets/        # Robot and environment 3D models (TinyRobot, etc.)
  project_config/        # Launch and config files per world (fleet params, nav graphs)
  project_fleet_adapter/  # Python fleet manager (FastAPI) + fleet adapter (rmf_easy)
  project_maps/          # Building YAMLs, Gazebo worlds, meshes and nav graphs
  project_simulation/    # Top-level simulation launch files
fix_lift.py              # Patch script for a known LiftPlugin segfault (see Troubleshooting)
```

## Prerequisites

Pull the required Docker images:

```bash
docker pull ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest
docker pull ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly
docker pull ghcr.io/open-rmf/rmf-web/demo-dashboard:jazzy-nightly
```

You also need [rocker](https://github.com/osrf/rocker) for GPU and X11 forwarding:

```bash
pip install rocker
```

## Setup

### 1. Start the RMF container

From your **home directory** (`~`), run:

```bash
rocker --nvidia --x11 --network host --name rmf_demos --user --home \
  --volume "$(pwd)/rmf-ws:/home/$USER/rmf_ws" \
  -- ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest bash
```

> **Note:** If you cloned this repo to a different path, adjust the `--volume` mount so that the `First Project/` folder is accessible inside the container as a colcon workspace. For example:
> ```bash
> --volume "/path/to/First Project:/home/$USER/rmf_ws"
> ```

### 2. Fix GPU permissions (if needed)

Inside the container, if Gazebo fails with `/dev/dri/renderD128: Permission denied`:

```bash
sudo chmod 666 /dev/dri/*
```

### 3. Build the workspace

Inside the container:

```bash
source /rmf_demos_ws/install/setup.bash
cd ~/rmf_ws
colcon build
source install/setup.bash
```

## Launching the Simulations

### Option A: Simple World (test1)

This is a basic two-floor building with a single TinyRobot.

**Terminal 1** (inside the container) -- Start the API server first (from host):

```bash
docker run --rm --network host \
  -e RMF_API_SERVER_CONFIG='{"host":"0.0.0.0","port":8000,"log_level":"warning"}' \
  ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly
```

**Terminal 2** (inside the container) -- Launch the simulation:

```bash
source ~/rmf_ws/install/setup.bash
ros2 launch project_simulation test1.launch.xml server_uri:="ws://localhost:8000/_internal"
```

**Terminal 3** (from host) -- Start the dashboard:

```bash
docker run --rm --network host \
  -e REACT_APP_TRAJECTORY_SERVER_URL="ws://localhost:8006" \
  -e RMF_SERVER_URL="http://localhost:8000" \
  ghcr.io/open-rmf/rmf-web/demo-dashboard:jazzy-nightly
```

Open the dashboard at **http://localhost:3000**.

### Option B: ICC Kyoto World

This is a more complex world based on the ICC Kyoto conference center, with doors, a lift, and multiple waypoints.

**Terminal 1** (from host) -- Start the API server:

```bash
docker run --rm --network host \
  -e RMF_API_SERVER_CONFIG='{"host":"0.0.0.0","port":8000,"log_level":"warning"}' \
  ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly
```

**Terminal 2** (inside the container) -- Launch the simulation:

```bash
source ~/rmf_ws/install/setup.bash
ros2 launch project_simulation icc_kyoto.launch.xml server_uri:="ws://localhost:8000/_internal"
```

**Terminal 3** (from host) -- Start the dashboard:

```bash
docker run --rm --network host \
  -e REACT_APP_TRAJECTORY_SERVER_URL="ws://localhost:8006" \
  -e RMF_SERVER_URL="http://localhost:8000" \
  ghcr.io/open-rmf/rmf-web/demo-dashboard:jazzy-nightly
```

Open the dashboard at **http://localhost:3000**.

### Sending Tasks via the Dashboard

Once the robot appears in the dashboard, you can dispatch tasks such as:

- **Patrol (Loop):** Send the robot to patrol between two waypoints (e.g., `podium` -> `snack_bar` -> `podium`).
- **Delivery:** Request a pickup at `snack_bar_pickup` and dropoff at another waypoint.

You can also dispatch tasks via CLI. Open a **new terminal inside the container** (Terminal 4):

```bash
# Get a shell inside the already-running container
docker exec -it rmf_demos bash

# Source the workspaces
source /rmf_demos_ws/install/setup.bash
source ~/rmf_ws/install/setup.bash

# Wait until the robot appears in the dashboard, then dispatch a patrol task
ros2 run rmf_demos_tasks dispatch_patrol \
  -p podium -p snack_bar -p stage_right -p podium \
  --use_sim_time
```

> **Important:** The patrol command **must** be run inside the container with both workspaces sourced. The robot must already be visible in the dashboard (i.e. registered with RMF) before dispatching the task, otherwise the bid will time out before any robot responds.

## Screenshots

### Dashboard overview

Gazebo simulation (left) running the ICC Kyoto world side-by-side with the RMF Dashboard (right) showing the building map, the robot position, and the fleet status.

![Dashboard](screenshots/dashboard.png)

### Patrol Task 1

A patrol task is dispatched. The planned route is drawn on the dashboard map, and the robot begins navigating along the assigned waypoints.

![Task 1 - Patrol route](screenshots/Task1.png)

### Patrol Task 2

A second patrol task with a different set of waypoints. The robot follows the new route through the building corridors.

![Task 2 - Patrol route](screenshots/Task2.png)

### Task Completed

The dashboard confirms that the dispatched task has been completed successfully.

![Task Completed](screenshots/Task_Completed.png)

## Troubleshooting

### 1. Gazebo crashes with Segmentation Fault in LiftPlugin

**Problem:** Both the `test1` and `icc_kyoto` worlds include lifts, and the `LiftPlugin` from `rmf_building_sim_gz_plugins` crashes with a segfault. The root cause is a bug where `enableComponent<AxisAlignedBox>()` is called inside an `ecm.Each()` iteration loop, which modifies the Entity Component Manager while iterating over it, corrupting the internal iterator.

**Solution:** Patch the lift plugin source code. The `fix_lift.py` script automates the patch. Inside the container, **in the same terminal you will use to launch the simulation**:

```bash
cd ~/
git clone https://github.com/open-rmf/rmf_simulation.git -b main
python3 ~/rmf_ws/fix_lift.py \
  ~/rmf_simulation/rmf_building_sim_gz_plugins/src/lift.cpp
cd ~/rmf_simulation
source /rmf_demos_ws/install/setup.bash
colcon build --packages-select rmf_building_sim_gz_plugins
source install/setup.bash
```

> **Critical:** The `source install/setup.bash` above loads the patched library **only in this terminal session**. Do not close or reuse this terminal for anything else — launch the simulation directly from here:

```bash
# Still in ~/rmf_simulation, now launch:
source ~/rmf_ws/install/setup.bash
ros2 launch project_simulation icc_kyoto.launch.xml server_uri:="ws://localhost:8000/_internal"
```

### 2. Fleet adapter fails to connect / Robot does not appear in dashboard

**Problem:** The fleet adapter crashes at startup or fails to report robot state. The API server shows the fleet registered but with `"robots": {}`.

**Root cause:** Two bugs in `RobotClientAPI.py`:
1. The fleet manager (FastAPI) defines its routes with trailing slashes (e.g., `/status/`), but the fleet adapter client called them without trailing slashes (e.g., `/status?robot_name=...`). FastAPI returns a 307 redirect which can lose query parameters.
2. The `get_data()` method did not check the `success` field in the response. When the fleet manager has not yet received robot state from Gazebo (normal at startup), it returns `{"data": {}, "success": false}`. The adapter then tried to parse the empty dict, causing a `KeyError: 'robot_name'`.

**Solution:** Fixed in `RobotClientAPI.py` by adding trailing slashes to all URL endpoints and checking `response.json()['success']` before parsing the data.

### 3. Missing `is_charger` flag on the charger waypoint

**Problem:** The fleet config (`tinyRobot_config.yaml`) sets `charger: "podium"` but the navigation graph did not have the `is_charger: true` property on the podium vertex.

**Solution:** Added `is_charger: true` to the podium vertex in `nav_graphs/0.yaml`.

### 4. GPU permission denied in container

**Problem:** Gazebo fails with `Permission denied` on `/dev/dri/renderD128` or `/dev/dri/renderD129`.

**Solution:**
```bash
sudo chmod 666 /dev/dri/*
```

### 5. Stale CMake cache after moving the workspace

**Problem:** `colcon build` fails with errors about paths not matching if the workspace was previously built at a different path (e.g., moving from `rmf_ws` to `rmf-ws`).

**Solution:** Delete the build directory for the affected package and rebuild:
```bash
rm -rf build/<package_name>
colcon build
```

### 6. Patrol task dispatched but robot does not move

**Problem:** `dispatch_patrol` exits without errors but the robot stays still. The task may appear as "queued" or "failed" in the dashboard.

**Root causes (in order of likelihood):**

1. **Command run outside the container or without the correct source.** `dispatch_patrol` must be run *inside* the container with both `/rmf_demos_ws/install/setup.bash` and `~/rmf_ws/install/setup.bash` sourced. If run from the host or from a fresh terminal, ROS2 topics are not connected to the simulation.

2. **Robot not yet registered when the task is dispatched.** The bidding window is 2 seconds. If the robot has not appeared in the dashboard yet (i.e. the fleet adapter has not polled the fleet manager yet), no robot bids on the task and the bid expires. Wait until the robot icon appears in the RMF dashboard before sending any task.

3. **Simulation launched from a terminal without the patched lift library.** If fix_lift.py was applied but the simulation was launched from a different terminal (without `source ~/rmf_simulation/install/setup.bash`), Gazebo loads the old library and crashes. The fleet adapter starts but never receives robot state from Gazebo, so the robot never registers. See issue #1 above for the correct launch procedure.

**Quick check:** After launching, run the following in a container terminal to confirm the robot is registered:
```bash
source /rmf_demos_ws/install/setup.bash
source ~/rmf_ws/install/setup.bash
ros2 topic echo /fleet_states --once
```
You should see `tinyRobot` fleet with `tinyRobot1` listed. If the output is empty or the fleet has no robots, the fleet adapter is not receiving robot state from Gazebo.
