#!/bin/bash
set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Default values
DISTRO="jazzy"
DETACH=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --humble)   DISTRO="humble"; shift ;;
        --jazzy)    DISTRO="jazzy"; shift ;;
        -d|--detach) DETACH=true; shift ;;
        --help|-h)
            echo "Usage: $0 [--humble|--jazzy] [-d|--detach]"
            echo ""
            echo "Runs nav_autonomy with FASTLIO2 and route planner."
            echo ""
            echo "Options:"
            echo "  --humble    Use ROS 2 Humble image"
            echo "  --jazzy     Use ROS 2 Jazzy image (default)"
            echo "  -d,--detach Run in background"
            echo ""
            echo "Configure hardware in .env file"
            exit 0 ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1 ;;
    esac
done

# Source .env file
set -a
[ -f .env ] && source .env
set +a

# Create directories
mkdir -p maps logs

# Container name
CONTAINER_NAME="nav_autonomy"

# Stop existing container if running
if docker ps -q -f name="^${CONTAINER_NAME}$" | grep -q .; then
    echo -e "${YELLOW}Stopping existing ${CONTAINER_NAME} container...${NC}"
    docker stop ${CONTAINER_NAME} 2>/dev/null || true
fi
docker rm ${CONTAINER_NAME} 2>/dev/null || true

# Image
IMAGE="iserverobotics/nav_autonomy:${DISTRO}"

# Pull if not available
if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE}$"; then
    echo -e "${YELLOW}Pulling ${IMAGE}...${NC}"
    docker pull "$IMAGE"
fi

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Starting Nav Autonomy${NC}"
echo -e "${GREEN}ROS Distribution: ${DISTRO}${NC}"
echo -e "${GREEN}ROS Domain ID:    ${ROS_DOMAIN_ID:-42}${NC}"
echo -e "${GREEN}SLAM Method:      FASTLIO2${NC}"
echo -e "${GREEN}Planner:          Route Planner${NC}"
echo -e "${GREEN}Lidar IP:         ${LIDAR_IP:-192.168.1.116}${NC}"
echo -e "${GREEN}Use Unitree:      ${USE_UNITREE:-false}${NC}"
echo -e "${GREEN}================================================${NC}"

# Build docker run command
DOCKER_OPTS=(
    --name ${CONTAINER_NAME}
    --shm-size=8gb
    --network host
    --privileged
    --restart unless-stopped
    -it
)

# Add GPU runtime if configured
if [ "${DOCKER_RUNTIME}" = "nvidia" ]; then
    DOCKER_OPTS+=(--runtime nvidia)
fi

# Detach mode
if [ "$DETACH" = true ]; then
    DOCKER_OPTS+=(-d)
fi

# Environment variables
DOCKER_OPTS+=(
    -e DISPLAY=${DISPLAY:-:0}
    -e QT_X11_NO_MITSHM=1
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=all
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    -e LIDAR_INTERFACE=${LIDAR_INTERFACE:-}
    -e LIDAR_COMPUTER_IP=${LIDAR_COMPUTER_IP:-192.168.1.5}
    -e LIDAR_GATEWAY=${LIDAR_GATEWAY:-192.168.1.1}
    -e LIDAR_IP=${LIDAR_IP:-192.168.1.116}
    -e MOTOR_SERIAL_DEVICE=${MOTOR_SERIAL_DEVICE:-/dev/ttyACM0}
    -e ENABLE_WIFI_BUFFER=${ENABLE_WIFI_BUFFER:-false}
    -e USE_RVIZ=${USE_RVIZ:-false}
    -e MAP_PATH=${MAP_PATH:-}
    -e USE_UNITREE=${USE_UNITREE:-false}
    -e UNITREE_IP=${UNITREE_IP:-192.168.12.1}
    -e UNITREE_CONN=${UNITREE_CONN:-LocalAP}
)

# Volumes
DOCKER_OPTS+=(
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v ${HOME}/.Xauthority:/root/.Xauthority:rw
    -v /etc/localtime:/etc/localtime:ro
    -v /etc/timezone:/etc/timezone:ro
    -v ${SCRIPT_DIR}/maps:/ros2_ws/maps:rw
    -v ${SCRIPT_DIR}/logs:/ros2_ws/logs:rw
)

# Devices
DOCKER_OPTS+=(
    --device /dev/input:/dev/input
    --device /dev/dri:/dev/dri
)

# Motor serial device (only if exists)
if [ -e "${MOTOR_SERIAL_DEVICE:-/dev/ttyACM0}" ]; then
    DOCKER_OPTS+=(--device ${MOTOR_SERIAL_DEVICE:-/dev/ttyACM0}:${MOTOR_SERIAL_DEVICE:-/dev/ttyACM0})
fi

# Groups
DOCKER_OPTS+=(
    --group-add ${INPUT_GID:-995}
    --group-add ${DIALOUT_GID:-20}
)

# Capabilities
DOCKER_OPTS+=(
    --cap-add NET_ADMIN
    --cap-add SYS_ADMIN
    --cap-add SYS_TIME
)

# Command to run inside container
CONTAINER_CMD='
echo "Starting real robot system with route planner..."
ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch.py use_fastlio2:=true &
NAV_PID=$!
sleep 2

# Launch Unitree WebRTC control if enabled
if [ "$USE_UNITREE" = "true" ]; then
    echo "Starting Unitree WebRTC control (IP: $UNITREE_IP, Method: $UNITREE_CONN)..."
    ros2 launch unitree_webrtc_ros unitree_control.launch.py robot_ip:=$UNITREE_IP connection_method:=$UNITREE_CONN &
fi

# Start twist relay for Foxglove Teleop
python3 /usr/local/bin/twist_relay.py &

# Start goal autonomy relay (enables autonomy when goal_pose received)
python3 /usr/local/bin/goal_autonomy_relay.py &

echo "Starting Foxglove Bridge on port 8765..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &

if [ "$USE_RVIZ" = "true" ]; then
    echo "Starting RViz2..."
    ros2 run rviz2 rviz2 -d /ros2_ws/src/route_planner/far_planner/rviz/default.rviz &
fi

wait $NAV_PID
'

# Run container
echo -e "${YELLOW}Starting container...${NC}"
docker run "${DOCKER_OPTS[@]}" ${IMAGE} bash -c "$CONTAINER_CMD"
