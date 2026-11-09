# Configuration guide

One YAML file per sensor, read at runtime by the ROS 1 node, the ROS 2 node and the Mac tool.

| File | Sensor |
| --- | --- |
| `kitti.yaml` | Velodyne HDL-64E (KITTI) |
| `ouster64.yaml` | Ouster OS1-64 |
| `vlp16.yaml` | Velodyne VLP-16 |
| `radar_4d.yaml` | 4D imaging radar, points weighted by power |

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

## Tuning `min_score`

`min_score` trades recall for precision and depends on the sensor and the place. Measured precision (pairs within 5 m of each other in ground truth):

| Data | Odometry | `min_score` | Loops | Precision |
| --- | --- | --- | --- | --- |
| KITTI 00, HDL-64 | ground truth | 0.99 | 416 | 1.00 (recall 0.80) |
| Kimera-Multi sparkal1, VLP-16 | FAST-LIO (ROS 1) | 0.98 | 25 | 0.76 (0.83 at 0.985) |
| Kimera-Multi sparkal1, VLP-16 | KISS-ICP (ROS 1) | 0.98 | 21 | 0.76 (0.89 at 0.985) |
| Kimera-Multi sparkal1, VLP-16 | FAST-LIO (ROS 2) | 0.98 | 24 | 0.67 (0.82 at 0.985) |
| Kimera-Multi sparkal1, VLP-16 | KISS-ICP (ROS 2) | 0.98 | 23 | 0.74 (0.70 at 0.985) |

To tune it for your data, run the Mac tool with `--gt-radius` on a sequence with ground truth poses, or run the node with `min_score: 0` and filter `loops.csv` by score afterwards.

## Sparse sensors: submaps

A single 4D radar scan (a few hundred points) is too sparse for place recognition. With `submap.scans > 1`, the last scans are merged into the keyframe's frame with the odometry poses before the descriptor is computed; the merged cloud is also what the session saves in `Scans/`. On KITTI 00 cut down to 400 random points per scan (no voxel filter):

| `submap.scans` | `min_score` | Loops | Precision | Recall |
| --- | --- | --- | --- | --- |
| 1 | 0.97 | 26 | 0.27 | 0.01 |
| 10 | 0.98 | 538 | 0.86 | 0.89 |
| 10 | 0.99 | 202 | 1.00 | 0.39 |
| 20 | 0.99 | 352 | 1.00 | 0.68 |

Recall here is the share of keyframes that revisit a place (within 5 m, more than 50 keyframes earlier) and get a correct candidate. `radar_4d.yaml` uses 20 scans within 15 m.

## Pitfalls

- **Wrong frame.** SOLiD bins by elevation, so the cloud must be in a frame whose z axis points up. A camera or optical IMU frame (z forward) collapses the descriptor.
- **Wrong FOV.** Points outside `fov_down`..`fov_up` are dropped. Check the datasheet.
- **Weights with voxels.** `use_weight: true` requires `voxel_size <= 0`.
