#!/usr/bin/env bash
# FAST-LIO + SOLiD (Kimera-Multi sparkal1)
# Offline (macOS / Linux, no ROS): SOLiD loop detection + loop closing on a bag where
# FAST-LIO was recorded, e.g.  rosbag record /sparkal1/lidar_points /Odometry
#   ./fast_lio.sh <bag> <out_dir>
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
python3 "$HERE/../../common/mac/solid_loops.py" --bag "$1" --out "$2" --close-loops \
  --cloud /sparkal1/lidar_points --odom /Odometry --config "$HERE/../config/kimera_sparkal1.yaml"
