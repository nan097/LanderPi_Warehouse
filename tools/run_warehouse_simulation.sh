#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
source tools/simulation_env.sh
export ROS_LOG_DIR=/tmp/landerpi_warehouse_logs
exec ros2 launch landerpi_simulation warehouse_simulation.launch.py "$@"
