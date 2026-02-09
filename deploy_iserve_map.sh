#!/bin/bash
set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Source .env for display
set -a
[ -f .env ] && source .env
set +a

DISTRO="${1:-humble}"

case "$DISTRO" in
    humble|jazzy) ;;
    --help|-h)
        echo "Usage: $0 [humble|jazzy]"
        echo ""
        echo "Deploys iserve_map using pre-built image."
        echo "Configure ROS_DOMAIN_ID and VOXEL_RESOLUTION in .env"
        echo ""
        echo "Examples:"
        echo "  $0 humble"
        echo "  $0 jazzy"
        exit 0 ;;
    *)
        echo -e "${RED}Unknown distro: $DISTRO (use humble or jazzy)${NC}"
        exit 1 ;;
esac

# Pull image if not available locally
IMAGE="iserverobotics/iserve_map:${DISTRO}"

if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE}$"; then
    echo -e "${YELLOW}Pulling ${IMAGE}...${NC}"
    docker pull "$IMAGE"
fi

# nav-monitor and log-analyzer are now built into the iserve_map image
# No separate monitor image needed

mkdir -p maps logs/monitor logs/analyzer

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Deploying iServe Map + Monitor + Log Analyzer${NC}"
echo -e "${GREEN}ROS Distribution: ${DISTRO}${NC}"
echo -e "${GREEN}ROS Domain ID:    ${ROS_DOMAIN_ID:-42}${NC}"
echo -e "${GREEN}Monitor Target:   ${TARGET_CONTAINER:-nav_autonomy}${NC}"
echo -e "${GREEN}Monitor Interval: ${MONITOR_INTERVAL:-1.0}s${NC}"
echo -e "${GREEN}Log Analyzer:     ON (summary every ${SUMMARY_INTERVAL:-60}s)${NC}"
echo -e "${GREEN}Publish map:      OFF${NC}"
echo -e "${GREEN}Auto-save:        every 10s${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo -e "Save map:     ${YELLOW}ros2 service call /map_builder/save_map iserve_map/srv/SaveMap \"{path: '/iserve_ws/maps'}\"${NC}"
echo -e "Evaluate:     ${YELLOW}ros2 service call /map_optimizer/evaluate_maps iserve_map/srv/EvaluateMaps \"{maps_directory: '/iserve_ws/maps'}\"${NC}"
echo -e "Switch map:   ${YELLOW}ros2 service call /map_switcher/switch_map iserve_map/srv/SwitchMap \"{pcd_path: '/iserve_ws/maps/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}\"${NC}"
echo -e "Repeat poses: ${YELLOW}ros2 service call /pose_repeater/repeat_poses std_srvs/srv/Trigger{}${NC}"
echo -e "Stop repeat:  ${YELLOW}ros2 service call /pose_repeater/stop_repeat std_srvs/srv/Trigger{}${NC}"
echo -e "Clear poses:  ${YELLOW}ros2 service call /pose_repeater/clear_poses std_srvs/srv/Trigger{}${NC}"
echo -e "Set loops:    ${YELLOW}ros2 param set /pose_repeater repeat_count 0${NC}  (0=infinite, N=N loops)"
echo ""

# Start iserve_map with the selected profile
docker compose -f iserve_map.deploy.yml --profile "$DISTRO" up
