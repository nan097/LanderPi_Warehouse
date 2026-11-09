#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
source tools/simulation_env.sh
export ROS_DOMAIN_ID=75
export ROS_LOG_DIR=/tmp/landerpi_inspect_logs
python3 tools/inspect_sort.py
