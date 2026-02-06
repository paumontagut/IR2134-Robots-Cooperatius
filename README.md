# IR2134 — Cooperative Robots (2025–2026)

Repository for labs, experiments, and notes for **IR2134 – Cooperative Robots** (academic year 2025–2026).

## Open-RMF

### Requirements

> **IMPORTANT: pull the Docker images first!!!**

```bash
docker pull ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest
docker pull ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly
docker pull ghcr.io/open-rmf/rmf-web/demo-dashboard:jazzy-nightly
```

## Lab: Building Maps & Simulation (Open-RMF + Gazebo Sim)

In this lab we created and edited a building (using `traffic-editor`), generated the corresponding Gazebo Sim world from a `.building.yaml`, and launched the simulation with the correct resource paths so models are found.

**Note:** Gazebo Sim crashes on my setup when I add a lift/elevator to the building, so I uploaded both YAML variants: one **with** the lift and one **without** it.

### Commands used

1) Launch traffic-editor (GPU + X11)

```bash
rocker --nvidia=gpus --x11 --name traffic-editor --user   --env __NV_PRIME_RENDER_OFFLOAD=1   --env __GLX_VENDOR_LIBRARY_NAME=nvidia   --env __VK_LAYER_NV_optimus=NVIDIA_only   --volume "$(pwd)/buildings:/rmf_demos_ws/buildings"   -- ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest   traffic-editor
```

2) Enter the workspace and start a container for generation

```bash
cd ~/RobCoperatius

rocker --nvidia=gpus --x11 --name rmf-gen   --user --home   --volume "$(pwd)/buildings:/rmf_demos_ws/buildings"   -- ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest bash
```

3) Generate the Gazebo world from the building YAML

```bash
cd /rmf_demos_ws/buildings

ros2 run rmf_building_map_tools building_map_generator gazebo   icc_kyoto.building.yaml   icc_kyoto.world   ./icc_kyoto_world
```

4) Set Gazebo resource paths (models / assets)

```bash
export GZ_SIM_RESOURCE_PATH="/rmf_demos_ws/buildings/icc_kyoto_world:/rmf_demos_ws/install/rmf_demos_assets/share/rmf_demos_assets/models:$HOME/.gz/fuel/fuel.gazebosim.org"
```

5) Run the simulation

```bash
gz sim -v 4 -r icc_kyoto.world
```
