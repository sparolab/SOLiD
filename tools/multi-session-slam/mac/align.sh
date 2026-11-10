#!/usr/bin/env bash
# Multi-session alignment without ROS (macOS / Linux): runs the C++ tool solid_align_sessions.
#   ./align.sh <out_dir> <central_session> <session> [session ...]
#   CONFIG=../config/radar_4d.yaml ./align.sh ...      (default: ../config/lidar.yaml)
# solid_align_sessions is built with the library: cmake -S cpp -B build -DSOLID_BUILD_BACKEND=ON
# (put it on PATH or set SOLID_TOOLS to its folder).
set -euo pipefail
[ $# -ge 3 ] || { sed -n 3,4p "$0"; exit 1; }
HERE=$(cd "$(dirname "$0")" && pwd)
CONFIG=${CONFIG:-$HERE/../config/lidar.yaml}
TOOL=${SOLID_TOOLS:+$SOLID_TOOLS/}solid_align_sessions
exec "$TOOL" "$CONFIG" "$@"
