# Multi-session SLAM guide

Align runs recorded on different days into one map. Every SOLiD front-end saves its run as a session (`save_dir` on the ROS nodes, `--out` on the macOS tool, see the [loop detection guide](../slam/GUIDE.md#output-a-session)). The alignment node loads two or more sessions and expresses all of them in the frame of the first one (the central session):

1. **Retrieval**: every keyframe of a session searches the central session's SOLiD descriptors.
2. **Check**: the most similar candidates are registered with GICP (keyframe submaps, SOLiD yaw as the initial guess).
3. **Consensus**: each checked loop implies one central ← session transform; only the largest group that agrees is kept.
4. **Graph**: all sessions go into one GTSAM pose graph (odometry + agreeing loops, Cauchy kernel) and are optimized together.

The code is `solid::multi_session` in [`cpp/`](../cpp); `ros1/` and `ros2/` are thin nodes around it.

## Run

```bash
# ROS 1
roslaunch solid_multi_session align.launch sessions:="$HOME/day1 $HOME/day2" save_dir:=$HOME/aligned
# ROS 2
ros2 launch solid_multi_session align.launch.py sessions:="$HOME/day1 $HOME/day2" save_dir:=$HOME/aligned
```

| Argument | | |
| --- | --- | --- |
| `sessions` | required | session folders separated by spaces; the first is the central session |
| `config` | `config/default.yaml` | see below |
| `save_dir` | empty | where to save the result |
| `rviz` | `true` | show the aligned map, trajectories and loops |

Outputs in `save_dir`: `<session>_aligned.txt` (KITTI) and `<session>_aligned_tum.txt` (TUM) for every session in the central frame, and `inter_session_loops.csv` (every candidate with its GICP fitness and whether it was used). RViz topics: `paths`, `loops` and `map` (one color per session).

### Example: KITTI 00 as two sessions

```bash
cd slam/mac
python solid_loops.py --bag kitti00.bag --config ../config/kitti.yaml --out sessions/00a --range 0:2000 --rebase
python solid_loops.py --bag kitti00.bag --config ../config/kitti.yaml --out sessions/00b --range 2000:4541 --rebase
roslaunch solid_multi_session align.launch sessions:="$PWD/sessions/00a $PWD/sessions/00b" save_dir:=$PWD/aligned
```

`--rebase` puts each session in its own frame, so the second one starts 300 m away from where it really is.

## Configuration

[`config/default.yaml`](config/default.yaml):

| Section | Key | |
| --- | --- | --- |
| `retrieval` | `min_score` | minimum R-SOLiD cosine similarity of a candidate |
| | `max_verifications` | GICP-check at most this many candidates, the most similar first |
| `verification` | `submap_keyframes` | ± keyframes merged around each loop keyframe |
| | `icp_voxel`, `icp_max_dist`, `icp_max_fitness` | GICP input voxel (m), max correspondence distance (m), max mean squared residual (m²) to accept a loop |
| `consensus` | `trans`, `rot_deg` | loops agree when their session transforms differ by less than this |
| | `min_inliers` | a session with fewer agreeing loops is left unaligned |
| `graph` | `odom_sigma_*`, `loop_sigma_*` | noise of odometry and loop edges |

## Compared with LT-SLAM (Scan Context)

The sessions also work with the original [LT-mapper](https://github.com/gisbi-kim/lt-mapper) LT-SLAM, which searches with Scan Context. [`ltslam_baseline/run_ltslam.sh`](ltslam_baseline/run_ltslam.sh) runs it in Docker:

```bash
./ltslam_baseline/run_ltslam.sh <sessions_dir> <central_session> <query_session> [out_dir]
```

On the KITTI 00 example (00a: scans 0-1999, 00b: 2000-4540):

| Method | Inter-session candidates | Candidates within 5 m | 00b ATE (max) | Time |
| --- | --- | --- | --- | --- |
| LT-SLAM (Scan Context) | 542 | 74.5 % | 30.14 m (72.2 m) | about 2 min |
| **SOLiD multi-session** | 350 (100 GICP-checked, 100 agree) | 100 % | **0.42 m (0.81 m)** | 12 s |

LT-SLAM accepts the wrong Scan Context pairs through its ICP check, so 00b is pulled out of place.
