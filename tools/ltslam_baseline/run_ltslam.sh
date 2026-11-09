#!/usr/bin/env bash
# Align two SOLiD sessions with LT-SLAM (ROS 1, Docker).
#   ./run_ltslam.sh <sessions_dir> <central_session> <query_session> [out_dir]
# <sessions_dir>/<session>/ must hold Scans/, SCDs/ and singlesession_posegraph.g2o,
# i.e. the save_dir output of the SOLiD ROS 1 / ROS 2 node or mac/solid_loops.py.
set -euo pipefail
SESSIONS=$(realpath "$1"); CENTRAL=$2; QUERY=$3
OUT=$(realpath -m "${4:-$SESSIONS/../aligned_${CENTRAL}_${QUERY}}")
IMAGE=${LTSLAM_IMAGE:-dongjae0107/lt-mapper:latest}
mkdir -p "$OUT"
for s in "$CENTRAL" "$QUERY"; do
  for f in Scans SCDs singlesession_posegraph.g2o; do
    [ -e "$SESSIONS/$s/$f" ] || { echo "missing $SESSIONS/$s/$f"; exit 1; }
  done
done
PARAMS=$(mktemp)
sed -e "s#SESSION_A#$CENTRAL#" -e "s#SESSION_B#$QUERY#" "$(dirname "$0")/params.yaml" > "$PARAMS"
docker run --rm --entrypoint bash \
  -v "$SESSIONS":/data/sessions:ro -v "$OUT":/data/aligned \
  -v "$PARAMS":/root/catkin_ws/src/lt-mapper/ltslam/config/params.yaml:ro \
  "$IMAGE" -c '
    source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
    roscore >/dev/null 2>&1 & sleep 3
    roslaunch ltslam run.launch
    chmod -R a+rw /data/aligned'
rm -f "$PARAMS"
echo "aligned poses: $OUT/${QUERY}_central_aft_intersession_loops.txt (KITTI format, in ${CENTRAL}'s frame)"
