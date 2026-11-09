#!/usr/bin/env bash
# Leg-KILO + SOLiD (legged, LiDAR)
# Offline (macOS / Linux, no ROS): SOLiD loop detection + loop closing on a bag where
# Leg-KILO was recorded, e.g.  rosbag record /cloud_registered /Odometry
#   ./legkilo.sh <bag> <out_dir>
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
python3 "$HERE/../../common/mac/solid_loops.py" --bag "$1" --out "$2" --close-loops \
  --cloud /cloud_registered --odom /Odometry --config "$HERE/../config/legkilo.yaml"
