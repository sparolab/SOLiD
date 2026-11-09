#!/usr/bin/env bash
# KISS-ICP + SOLiD
# Offline (macOS / Linux, no ROS): SOLiD loop detection + loop closing on a bag where
# KISS-ICP was recorded, e.g.  ros2 bag record /kiss/frame /kiss/odometry
#   ./kiss_icp.sh <bag> <out_dir>
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
python3 "$HERE/../../common/mac/solid_loops.py" --bag "$1" --out "$2" --close-loops \
  --cloud /kiss/frame --odom /kiss/odometry --config "$HERE/../../config/vlp16.yaml"
