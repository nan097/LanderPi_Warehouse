#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")/.."
exec bash tools/run_sort_simulation.sh autorun:=true "$@"
