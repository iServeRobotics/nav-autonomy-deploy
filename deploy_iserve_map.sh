#!/bin/bash
# =============================================================================
# iServe Map - Deployment Script
# Copyright (c) 2024-2026 iServe Robotics. All rights reserved.
#
# Licensed under CC BY-NC 4.0 (Non-Commercial Use Only).
# Commercial use requires a separate license from iServe Robotics.
# See LICENSE-ISERVE-MAP or https://creativecommons.org/licenses/by-nc/4.0/
#
# Contact: licensing@iserverobotics.com
# =============================================================================
set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# --- License acceptance gate ---
LICENSE_ACCEPTED_FILE="$SCRIPT_DIR/.iserve_license_accepted"
if [ ! -f "$LICENSE_ACCEPTED_FILE" ]; then
    echo ""
    echo -e "${YELLOW}============================================================${NC}"
    echo -e "${YELLOW}  iServe Map - License Agreement${NC}"
    echo -e "${YELLOW}============================================================${NC}"
    echo ""
    echo "  iServe Map is licensed under CC BY-NC 4.0."
    echo "  Free for evaluation, research, and non-commercial use."
    echo ""
    echo -e "  ${RED}Commercial use requires a separate license.${NC}"
    echo "  Contact: info@iserve.ai"
    echo ""
    echo "  Full terms: https://creativecommons.org/licenses/by-nc/4.0/"
    echo "  See also:   LICENSE-ISERVE-MAP"
    echo ""
    echo -e "${YELLOW}============================================================${NC}"
    echo ""
    read -rp "Do you accept the license terms? [y/N] " ACCEPT
    if [[ "$ACCEPT" != "y" && "$ACCEPT" != "Y" ]]; then
        echo -e "${RED}License not accepted. Exiting.${NC}"
        exit 1
    fi
    date -Iseconds > "$LICENSE_ACCEPTED_FILE"
    echo -e "${GREEN}License accepted.${NC}"
    echo ""
fi

# Source .env for display
set -a
[ -f .env ] && source .env
set +a

case "${1:-}" in
    --help|-h)
        echo "Usage: $0"
        echo ""
        echo "Deploys iserve_map (Jazzy) using pre-built image."
        echo "Configure ROS_DOMAIN_ID and VOXEL_RESOLUTION in .env"
        exit 0 ;;
esac

# Pull image if not available locally
IMAGE="iserverobotics/iserve_map:jazzy"

if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE}$"; then
    echo -e "${YELLOW}Pulling ${IMAGE}...${NC}"
    docker pull "$IMAGE"
fi

mkdir -p maps logs/monitor logs/analyzer

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Deploying iServe Map + Monitor + Log Analyzer${NC}"
echo -e "${GREEN}ROS Distribution: jazzy${NC}"
echo -e "${GREEN}ROS Domain ID:    ${ROS_DOMAIN_ID:-42}${NC}"
echo -e "${GREEN}Monitor Target:   ${TARGET_CONTAINER:-nav_autonomy}${NC}"
echo -e "${GREEN}Monitor Interval: ${MONITOR_INTERVAL:-1.0}s${NC}"
echo -e "${GREEN}Log Analyzer:     ON (summary every ${SUMMARY_INTERVAL:-60}s)${NC}"
echo -e "${GREEN}OTel:             ${OTEL_ENABLED:-true}${NC}"
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

# Start iserve_map
docker compose -f iserve_map.deploy.yml up
