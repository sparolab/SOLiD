#!/usr/bin/env bash
# GaRLILEO + SOLiD (legged, 4D radar)
# Offline (macOS / Linux, no ROS): SOLiD loop detection + loop closing on a bag where
# GaRLILEO was recorded, e.g.  ros2 bag record /ti_mmwave/radar_scan_pcl_0 /Odometry  (pass the first /imu stamp as $3)
#   ./garlileo.sh <bag> <out_dir>
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
python3 "$HERE/../../common/mac/solid_loops.py" --bag "$1" --out "$2" --close-loops \
  --cloud /ti_mmwave/radar_scan_pcl_0 --odom /Odometry --odom-time-offset "$3" --config "$HERE/../config/garlileo.yaml"
