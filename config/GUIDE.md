# Configuration guide

One YAML file per sensor, read at runtime by the ROS 1 node, the ROS 2 node and the Mac tool.

| File | Sensor |
| --- | --- |
| `kitti.yaml` | Velodyne HDL-64E (KITTI) |
| `ouster64.yaml` | Ouster OS1-64 |
| `vlp16.yaml` | Velodyne VLP-16 |
| `livox_mid360.yaml` | Livox Mid-360 (vertical FOV -7 ~ +52 deg) |
| `radar_4d.yaml` | 4D imaging radar (sparse: submaps of 20 scans) |
| `multi_session/lidar.yaml`, `multi_session/radar_4d.yaml` | multi-session alignment |

Platform-specific copies (odometry frames, extrinsics) are next to the launch files in [`integrations/*/config`](../integrations).

Copy the closest one for a new sensor and pass its path as `config`.

## Keys

| Section | Key | |
| --- | --- | --- |
| `solid` | `fov_up`, `fov_down` | the sensor's vertical field of view (deg). Set these first |
| | `num_height` | elevation bins, about the number of rings |
| | `num_range`, `num_angle` | range bins (R-SOLiD) and azimuth bins (A-SOLiD, yaw resolution 360 / `num_angle`) |
| | `min_range`, `max_range` | m, points outside are dropped; `max_range` also sets the range bin size |
| | `voxel_size` | m, downsampling before binning; `<= 0` disables it (needed with `use_weight`) |
| | `use_weight` | `true`: each point counts with its `input.weight_field` value (radar power) instead of 1 |
| | `knn` | KD-tree neighbours re-scored per query |
| `keyframe` | `dist`, `angle_deg` | a new keyframe after this much translation (m) or rotation (deg) |
| `submap` | `scans` | scans merged (with the odometry poses) into each keyframe's descriptor; 1 = single scan |
| | `max_dist` | m, only merge scans within this travel distance (0 = no limit) |
| `loop` | `exclude_recent` | the latest N keyframes are never matched (they are trivially similar) |
| | `min_score` | minimum R-SOLiD cosine similarity of a candidate |
| | `max_candidates` | candidates reported per keyframe |
| `input` | `cloud_in_world` | `true` if the cloud is in the odometry frame; it is moved back to the sensor frame |
| | `weight_field` | point field used as the weight (`power`, `intensity`, `rcs`, ...), `""` = none |
| | `extrinsic` | `{t: [x, y, z], R: [9 values, row-major]}`: the sensor in the odometry body frame (e.g. LiDAR in IMU for FAST-LIO) |
| `loop_closing` | `enable` | verify candidates with GICP and optimize the pose graph online (iSAM2) |
| | `submap_keyframes`, `scan_voxel`, `icp_voxel` | ± keyframes per GICP submap, voxel of the stored scans and of the GICP input (m) |
| | `icp_max_dist`, `icp_max_fitness` | GICP correspondence distance (m) and the mean squared residual (m²) to accept a loop |
| | `odom_sigma_*`, `loop_sigma_*` | noise of the odometry and loop edges (loops use a Cauchy kernel) |

## Loop closing: drift before and after

ATE after a best rigid fit to the ground truth, so only the drift counts:

| Data | Odometry | Odometry only | With SOLiD loop closing |
| --- | --- | --- | --- |
| KITTI 00 (HDL-64) | KISS-ICP | 3.93 m (max 8.24) | **1.13 m** (max 2.52) |
| KITTI 00, ROS 1 / ROS 2 node online (2x) | KISS-ICP | 4.02 m | **1.15 m** |
| SNAIL-Radar 20240113/data1 (ARS548) | GeoRIO | 0.47 m (max 1.40) | **0.19 m** (max 0.43) |
| GaRLILEO BridgeLoop (TI IWR1843, Spot) | GaRLILEO | 0.42 m (max 0.80) | **0.30 m** (max 0.75) |

On the KILVO humanoid (Unitree G1, Livox Mid-360, FAST-LIO; no ground truth), the GICP check kept all 19 candidates that FAST-LIO also puts within 3 m and rejected all 9 that it does not.

## Tuning `min_score`

`min_score` trades recall for precision and depends on the sensor and the place. Measured precision (pairs within 5 m of each other in ground truth):

| Data | Odometry | `min_score` | Loops | Precision |
| --- | --- | --- | --- | --- |
| KITTI 00, HDL-64 | ground truth | 0.99 | 416 | 1.00 (recall 0.80) |
| GaRLILEO BridgeLoop, TI IWR1843, submap 50 | GaRLILEO | 0.98 | 116 | 0.95 (recall 0.70) |
| KILVO outdoor_h01, Livox Mid-360, submap 10 | FAST-LIO | 0.98 | 28 | 0.68 before GICP, 1.00 after |
| SNAIL-Radar 20240113/data1, ARS548 4D radar, submap 20 | GeoRIO | 0.97 | 143 | 1.00 (recall 0.67) |
| Kimera-Multi sparkal1, VLP-16 | FAST-LIO (ROS 1) | 0.98 | 25 | 0.76 (0.83 at 0.985) |
| Kimera-Multi sparkal1, VLP-16 | KISS-ICP (ROS 1) | 0.98 | 21 | 0.76 (0.89 at 0.985) |
| Kimera-Multi sparkal1, VLP-16 | FAST-LIO (ROS 2) | 0.98 | 24 | 0.67 (0.82 at 0.985) |
| Kimera-Multi sparkal1, VLP-16 | KISS-ICP (ROS 2) | 0.98 | 23 | 0.74 (0.70 at 0.985) |

With loop closing on, the GICP check removes most wrong candidates, so `min_score` can be a little lower than for candidates alone. To tune it for your data, run the Mac tool with `--gt-radius` on a sequence with ground truth poses, or run the node with `min_score: 0` and filter `loops.csv` by score afterwards.

## Sparse sensors: submaps

A single 4D radar scan (a few hundred points) is too sparse for place recognition. With `submap.scans > 1`, the last scans are merged into the keyframe's frame with the odometry poses before the descriptor is computed; the merged cloud is also what the session saves in `Scans/`. On KITTI 00 cut down to 400 random points per scan (no voxel filter):

| `submap.scans` | `min_score` | Loops | Precision | Recall |
| --- | --- | --- | --- | --- |
| 1 | 0.97 | 26 | 0.27 | 0.01 |
| 10 | 0.98 | 538 | 0.86 | 0.89 |
| 10 | 0.99 | 202 | 1.00 | 0.39 |
| 20 | 0.99 | 352 | 1.00 | 0.68 |

On real 4D radar, [SNAIL-Radar](https://snail-radar.github.io/) `20240113/data1` (Continental ARS548, about 1,100 points per scan) with [GeoRIO](https://github.com/wooseongY/GeoRIO) odometry, against ground truth:

| `submap.scans` | `min_score` 0.95: precision / recall | `min_score` 0.97: precision / recall |
| --- | --- | --- |
| 1 | 0.95 / 0.25 | 1.00 / 0.02 |
| 10 | 0.90 / 0.87 | 1.00 / 0.60 |
| 20 | 0.91 / 0.90 | 1.00 / 0.67 |
| 40 | 0.93 / 0.88 | 1.00 / 0.70 |

Recall here is the share of keyframes that revisit a place (within 5 m, more than 50 keyframes earlier) and get a correct candidate. `radar_4d.yaml` uses 20 scans within 15 m and `min_score` 0.97.

## Pitfalls

- **Wrong frame.** SOLiD bins by elevation, so the cloud must be in a frame whose z axis points up. A camera or optical IMU frame (z forward) collapses the descriptor.
- **Wrong FOV.** Points outside `fov_down`..`fov_up` are dropped. Check the datasheet.
- **Weights with voxels.** `use_weight: true` requires `voxel_size <= 0`.
