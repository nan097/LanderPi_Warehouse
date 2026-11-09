#!/bin/bash
set -eo pipefail
exec bash "$(dirname "$0")/run_warehouse_simulation.sh" autorun:=true "$@"
