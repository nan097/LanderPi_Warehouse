#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
source tools/simulation_env.sh
exec python3 tools/test_sort_simulation.py "$@"
