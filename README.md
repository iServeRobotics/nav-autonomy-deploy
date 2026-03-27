# nav-autonomy-deploy

Docker-based deployment system for autonomous navigation on robotic platforms. Orchestrates containerized ROS 2 services for real-time navigation, LiDAR-based SLAM, and remote monitoring.

## Overview

This repository provides deployment configurations and scripts to run the iServe Robotics navigation stack via Docker Compose.

- **nav_autonomy** -- Core navigation stack including SLAM, path planning, localization, and robot control
- **iserve_map** -- Mapping, monitoring, cloud telemetry, and log analysis

## Prerequisites

- Docker & Docker Compose (v1.29+)
- Linux host with network namespace support
- Mid-360 LiDAR (or compatible)
- Optional: Joystick controller (serial)
- Optional: NVIDIA GPU with CUDA runtime
- Optional: Unitree Go2/G1 robot
- Optional: X11 server (for RViz visualization)

## Quick Start

### 1. Configure Hardware

Edit the `.env` file with your hardware details:

| Variable | Description | Default |
|----------|-------------|---------|
| `ROS_DOMAIN_ID` | ROS 2 domain for multi-robot isolation | `42` |
| `ROBOT_CONFIG_PATH` | Robot model config path | `unitree/unitree_g1` |
| `ROBOT_IP` | Robot IP address (check your network) | `192.168.12.1` |
| `LIDAR_INTERFACE` | Network interface for LiDAR (check with `ip addr`) | `enP8p1s0` |
| `LIDAR_IP` | LiDAR device IP | `192.168.1.1xx` |
| `LIDAR_COMPUTER_IP` | Processing computer IP | `192.168.1.xxx` |
| `USE_UNITREE` | Enable Unitree robot control | `true` |
| `UNITREE_IP` | Unitree robot IP | `192.168.12.1` |
| `DOCKER_RUNTIME` | Container runtime (`runc` or `nvidia`) | `runc` |
| `MAP_PATH` | Pre-built map for localization (empty = mapping mode) | `` |

> **Important:** Double-check `ROBOT_IP` and `LIDAR_INTERFACE` match your actual hardware setup before starting.

### 2. Start Services

Both `nav_autonomy` and `iserve_map` run from a single compose file:

```bash
docker compose up -d          # Start both services (detached)
docker compose logs -f        # View logs
docker compose down           # Stop both services
```

To update both images and restart:

```bash
docker compose pull && docker compose down && docker compose up -d
```

<details>
<summary>Run only one service</summary>

```bash
docker compose up -d nav_autonomy    # Nav stack only
docker compose up -d iserve_map      # iServe Map only
```

</details>

**Fallback (without Docker Compose):**

If Docker Compose is not available, use the standalone script:

```bash
./run.sh              # Start
./run.sh -d           # Detached mode
```

> **Note (Jetson G1):** To install the Docker Compose plugin without updating JetPack:
> ```bash
> curl -SL https://github.com/docker/compose/releases/latest/download/docker-compose-linux-aarch64 -o ~/.docker/cli-plugins/docker-compose && chmod +x ~/.docker/cli-plugins/docker-compose
> ```

### 3. Connect Visualization

Connect [Foxglove](https://foxglove.dev/) to the robot for remote monitoring:

- **Navigation**: `ws://<robot-ip>:8765`
- **iServe Map**: `ws://<robot-ip>:8766`

Import `Overwwatch.json` into Foxglove for a pre-configured layout with 3D terrain view, autonomy status, and path visualization.

To send a goal pose, click the **Publish** button on the right toolbar of the 3D panel (highlighted in red below), then click on the map: the first click sets the position (x, y), and the second click sets the heading.

![Pick a goal pose in Foxglove](assets/pick_goal.png)

## Updating Docker Images

Both services use pre-built images from Docker Hub. To pull the latest versions and restart:

```bash
docker compose pull && docker compose down && docker compose up -d
```

| Image | Tag | Source |
|-------|-----|--------|
| `iserverobotics/nav_autonomy` | `jazzy` | [nav_autonomy](https://github.com/iServeRobotics/nav_autonomy) |
| `iserverobotics/iserve_map` | `jazzy` | [iserve_map](https://github.com/iServeRobotics/iserve_map) |

## Project Structure

```
.
├── docker-compose.yml          # Container orchestration (nav_autonomy + iserve_map)
├── run.sh                      # Launch script for nav_autonomy
├── .env                        # Hardware configuration
├── Overwwatch.json             # Foxglove visualization layout
├── maps/                       # Stored map PCD files
├── logs/                       # Runtime logs
└── LICENSE                     # BSD-3-Clause
```

## License

[BSD-3-Clause](LICENSE)

The nav_autonomy image is derived from the [CMU autonomy stack](https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform) via [Vector Navigation Stack](https://github.com/VectorRobotics/vector_navigation_stack).
