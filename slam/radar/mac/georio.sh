#!/usr/bin/env bash
# GeoRIO + SOLiD (4D radar)
# Offline (macOS / Linux, no ROS): SOLiD loop detection + loop closing on a bag where
# GeoRIO was recorded, e.g.  ros2 bag record /current_scan /odometry_georio
#   ./georio.sh <bag> <out_dir>
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
python3 "$HERE/../../common/mac/solid_loops.py" --bag "$1" --out "$2" --close-loops \
  --cloud /current_scan --odom /odometry_georio --config "$HERE/../config/georio.yaml"
