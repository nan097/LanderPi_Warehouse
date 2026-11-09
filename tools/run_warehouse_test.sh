#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
source tools/simulation_env.sh
export ROS_LOG_DIR=/tmp/landerpi_warehouse_test_logs
exec python3 tools/test_warehouse_simulation.py "$@"
