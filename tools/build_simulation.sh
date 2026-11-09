#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then source install/setup.bash; fi
source tools/simulation_env.sh
colcon build --symlink-install --packages-select \
  landerpi_description landerpi_task_interfaces landerpi_experiment_tools \
  landerpi_task_manager landerpi_moveit_config landerpi_simulation landerpi_bringup \
  --parallel-workers 2
