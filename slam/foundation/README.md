<div align="center">
  <h1>SOLiD x VGGT-SLAM</h1>
  <a href="https://arxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/arXiv-2408.07330-b31b1b?logo=arxiv&logoColor=white" alt="arXiv" /></a>
  <a href="https://sparolab.github.io/research/solid/"><img src="https://img.shields.io/badge/Project-Commerge-6f42c1" alt="SOLiDxVGGT-SLAM" /></a>
  <a href=""><img src="https://img.shields.io/badge/YouTube-Video-FF0000?logo=youtube&logoColor=white" alt="YouTube" /></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-BSD--3--Clause-green" alt="BSD 3-Clause License" /></a>
  <br />
  <br />
  <p align="center">
  <img src="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=14&duration=1500&pause=1000&center=true&vCenter=true&width=200&height=24&lines=SOLiD+meets+VGGT+SLAM%21" alt="tagline"/>
  </p>
</div>

<div align="center">
  <img src="assets/vggt_solid_slam.gif" width="720"/>
</div>

## Result

On the bundled `office_loop` sequence (473 frames), the validated configuration
produces exactly one loop at the intended final revisit:

| Result | Value |
|---|---:|
| accepted loop | submap `204 -> 0` |
| SOLiD descriptor distance | `0.457` |
| odometry radius | `0.4` |
| VGGT image match ratio | `0.998` |
| false loops before final revisit | `0` |
| descriptor input | XYZ geometry only |

The radius search is an odometry-space candidate gate, not an RGB/intensity
feature. `SOLID_MIN_SUBMAP_GAP=50` prevents adjacent-submap matches.

## Files

| File | Purpose |
|---|---|
| `vggt_slam/pluggable_retrieval.py` | SALAD/NetVLAD/SOLiD retrieval backends and radius gate |
| `vggt_slam/o3d_loop_vis.py` | capture loop-before/loop-after maps and camera wireframes |
| `vggt_slam_integration.patch` | changes required in VGGT-SLAM `main.py` and `solver.py` |
| `HANDOFF_solid_loop_closure.md` | experiments, reasoning, and detailed handoff |

## Install into VGGT-SLAM

From a clean VGGT-SLAM checkout:

```bash
git apply /path/to/SOLiD/slam/foundation/vggt_slam_integration.patch
cp /path/to/SOLiD/slam/foundation/vggt_slam/*.py vggt_slam/
pip install -e /path/to/SOLiD
```

The patch is based on VGGT-SLAM commit `35327ac`.

## Reproduce the validated office loop

```bash
VGGT_SEED=0 \
SOLID_NO_FLOOR=1 SOLID_CAMERA_Z_UP=1 SOLID_INTENSITY=0 SOLID_AZIMUTH=0 \
SOLID_MAX_RANGE=2.0 SOLID_NUM_RANGE=10 SOLID_NUM_HEIGHT=12 \
SOLID_FOV=30 SOLID_MIN_RANGE=0.05 SOLID_VOXEL=0.04 SOLID_SUB=6 \
SOLID_MEAN_REMOVE=0 SOLID_MIN_SUBMAP_GAP=50 SOLID_ODOM_RADIUS=0.4 \
SOLID_ICP_MIN=0 VGGT_IMG_THRES=0 \
python -u main.py --image_folder office_loop --submap_size 16 \
  --max_loops 1 --retrieval solid --lc_thres 0.55
```

To capture loop-before/after snapshots, append:

```bash
--o3d_lc_vis --o3d_lc_no_window --o3d_lc_dir o3d_loop_vis --o3d_lc_voxel 0.03
```

The visualization above uses VGGT-SLAM's common dense reconstruction, not an
externally measured ground-truth map.
