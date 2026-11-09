#!/usr/bin/env bash
# FAST-LIO on a Livox Mid-360 + SOLiD (humanoid)
# Offline (macOS / Linux, no ROS): SOLiD loop detection + loop closing on a bag where
# FAST-LIO was recorded, e.g.  rosbag record /cloud_registered_body /Odometry
#   ./fast_lio_mid360.sh <bag> <out_dir>
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
python3 "$HERE/../../common/mac/solid_loops.py" --bag "$1" --out "$2" --close-loops \
  --cloud /cloud_registered_body --odom /Odometry --config "$HERE/../../config/livox_mid360.yaml"
