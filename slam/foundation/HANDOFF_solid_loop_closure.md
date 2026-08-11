# Handoff — SOLiD loop closure inside VGGT-SLAM (office + KITTI)

**Last updated:** 2026-08-11
**Goal:** Use SOLiD (LiDAR range descriptor) as the loop-closure place recognizer inside VGGT-SLAM (feed-forward monocular SLAM), replacing its default SALAD/NetVLAD. Get true loops to close without diverging.

**TL;DR of the outcome:** On self-similar scenes (indoor office rooms, KITTI straight roads) SOLiD-on-VGGT-clouds **cannot** isolate the true loop from perceptual aliases using geometry alone — proven with ground truth. The one thing that helped substantially is **adding image colour to the descriptor** (R-SOLiD + G-SOLiD + B-SOLiD). On real LiDAR SOLiD works fine (user confirmed); the limiter here is VGGT's generic monocular depth.

---

## 1. Environments & how to run

- **VGGT-SLAM runs:** `conda activate vggt-slam` (py3.11, torch2.3.1+cu121). Run dir: `reference/VGGT-SLAM/`.
- **Offline analysis:** `conda activate loop_splat` — has the `solid` python module (`import solid`) + open3d + numpy. Used for all the parameter sweeps on dumps.
- **Launch pattern (background, avoids self-kill):** write a `.sh` with the env exports, then
  `setsid bash run.sh > out.log 2>&1 < /dev/null & disown`. Do NOT `pgrep -f "image_folder ..."` to pre-kill — it matches the launcher and kills its own shell. Kill GPU procs via `nvidia-smi --query-compute-apps=pid`.
- A VGGT-SLAM run on full KITTI 05 (~2761 frames) → ~12 submaps, ~8-10 min. viser serves on **port 8080** with `--vis_map`.

### Canonical run command
```bash
VGGT_SEED=0 SOLID_NO_FLOOR=1 SOLID_INTENSITY=1 \
SOLID_MAX_RANGE=1.2 SOLID_NUM_RANGE=20 SOLID_NUM_HEIGHT=15 SOLID_FOV=12 \
SOLID_MIN_RANGE=0.02 SOLID_VOXEL=0.003 SOLID_MEAN_REMOVE=0 \
SOLID_CHANNEL_NORM=1 SOLID_CAMERA_Z_UP=1 SOLID_AZIMUTH=0 \
SOLID_MIN_SUBMAP_GAP=60 VGGT_IMG_THRES=0.5 \
python -u main.py --image_folder kitti05_full --submap_size 32 \
    --max_loops 1 --retrieval solid --lc_thres 0.3 --vis_map
```

---

## 2. Code changes made (all in `reference/VGGT-SLAM/`)

### `main.py`
- **Seed fixing** (after imports): `VGGT_SEED` (default 0) → `random/np/torch.manual_seed` + `cudnn.deterministic=True, benchmark=False`. VGGT-SLAM's `solver.sample_pixel_coordinates` uses `torch.randint` with no seed, so every run differed (submap boundaries, loop matches). Now runs are near-identical. **Do NOT** enable `torch.use_deterministic_algorithms(True)` — it makes VGGT ~unusably slow (submap/2min). Residual cuDNN drift only shifts keyframe boundaries ±1-2 frames.

### `vggt_slam/solver.py`
- In the `retrieval_needs_points` (SOLiD) block (~line 350): attach per-point **colour** `new_submap.colors = images.permute(0,2,3,1)` (S,H,W,3) aligned with the point map, guarded by a shape check.
- Loop-apply gate (~line 398): `[LC-RATIO]` debug print of `desc_d` + `image_match_ratio` for every candidate; gate threshold from env `VGGT_IMG_THRES` (default 0.95). Set 0.99 to log-only (block all applies → pure odometry, no divergence, still dumps).

### `vggt_slam/pluggable_retrieval.py` — `SolidRetrieval` (the main file)
All env-overridable; defaults chosen for VGGT clouds:
- **Profile overrides:** `SOLID_MAX_RANGE`(8), `SOLID_NUM_RANGE`(60), `SOLID_NUM_HEIGHT`(20), `SOLID_FOV`(10), `SOLID_MIN_RANGE`(0.05), `SOLID_VOXEL`(0.03). (Defaults are generic; pass the KITTI/office values below.)
- `SOLID_SCALE_NORM` (default off): per-cloud rescale. HARMFUL — leave off. VGGT depth is scale-consistent across submaps within a run.
- `SOLID_MEAN_REMOVE` (default off): mean-removal helps LoopSplat real depth, HURTS here.
- `SOLID_NO_FLOOR=1`: skip floor-plane gravity align. KITTI camera is already level (road normal ≈ camera +y) so it's near-identity; also avoids the o3d `segment_plane` bug (picks a WALL not floor in indoor, and is RANSAC-random per call). **Known bug:** `_floor_gravity` uses `segment_plane` = largest plane = often a wall; inconsistent frame-to-frame. Fix (not committed): pick most-vertical-normal plane + seed RANSAC.
- `SOLID_SUB`: per-frame point subsample stride (default 6; 1 = none).
- **`SOLID_INTENSITY=1`: THE COLOUR DESCRIPTOR.** `_descriptor_from_points` returns `R-SOLiD ++ G-SOLiD ++ B-SOLiD` (3×num_range), weighting rsolid bins by each image colour channel via `ext.extract(pc, weights)` (SOLiD natively supports per-point weights — originally LiDAR intensity / radar RCS; sets `cfg.use_weight=True`). Needs `submap.colors`.
- `SOLID_MIN_SUBMAP_GAP`: temporal adjacency exclusion (frame-id units). This is the "gap" — user dislikes large values (200) as a hack; a small value (~50-60) is standard conveyor exclusion.
- **ICP gate:** `_icp_fitness` / `_icp_fitness_by_ids`, gated by `SOLID_ICP_MIN` (default 0 = log-only). Multi-yaw ICP with centroid init on the two matched frame clouds. **Verdict: does NOT work** (see dead ends). `[SOLID-ICP]` prints per candidate.
- **Dump format changed:** `SOLID_DUMP=path.pkl` now saves `{submap_id: {"clouds":[per-frame arrays], "colors":[per-frame arrays or None]}}`. (Older dumps were just `{id: [clouds]}` — offline scripts must branch on format.)

---

## 3. Configs that matter

| scene | max_range | num_range | num_height | fov | voxel | min_range | floor | mean_rm |
|---|---|---|---|---|---|---|---|---|
| **KITTI 05 (corrected z-up)** | 1.2 | 20 | 15 | 12 | 0.003 | 0.02 | fixed camera→z-up | 0 |
| office_loop | 2.0 | 10 | 12 | 30 | 0.04 | 0.05 | ON | 0 |

Rationale: VGGT depth is **non-metric** (normalized), scale differs per scene — KITTI xy-range ~0.05-0.34, office ~0.5. So `max_range` must be tuned to the actual point distribution each scene (check with a scale dump). num_height barely matters on KITTI (flat driving).

---

## 4. Key findings

### office_loop
- Single go-around loop; the ONE true loop is **last submap → first submap (submap 0)**. See memory `vggt-slam-solid-loop-requirement.md`.
- Pathological self-similar: a MID submap's keyframe cloud genuinely looks like the START (visually confirmed). No local signal isolates last→0.

### KITTI 05 (GT-validated)
- **GT loops** (from `/nas/Ground/KITTI_Dataset/Odometry/data_odometry_poses/dataset/poses/05.txt`): frame **1286↔535** (mid revisit) and **2322↔15** (end→start, main loop). It's a LONG revisit (559 frames) spanning several submaps.
- VGGT odometry (loops off) roughly reproduces the KITTI 05 trajectory shape (with drift) — the backend works; divergence in earlier runs was from applying wrong alias loops.
- **Separation metric** used throughout: label each submap true/false via GT (<15m to a ≥2-back submap), then `sep = min(false best-match desc dist) − max(true best-match desc dist)`. Positive = a threshold separates true from false. **Best achieved: −0.047** (still negative).

```
count (geometry only)             sep = -0.085   (false alias @ 0.005 < any true loop)
R-SOLiD+G-SOLiD+B-SOLiD (120-d)   sep = -0.053   ← COLOUR: ~40% better, false-min 0.005→0.022
  + chromaticity colour-norm      sep = -0.047   ← marginal extra (illumination invariance)
```

**The blocker:** on KITTI 05 one true loop (idx5 in the 12-submap run) is weak — VGGT produced an inconsistent submap that matches the wrong target — and a few road-alias submaps are descriptor-stronger. That one weak loop ruins any global threshold.

---

## 5. Dead ends (DON'T redo these — all measured, most GT-validated)

- **Large temporal gap (e.g. 200)** to force last→0 on office: works but user rejects as a structural hack.
- **submap_size 4 / 8**: worse (fewer points → noisier descriptor, more aliases).
- **VGGT `image_match_ratio` gate**: does not separate — aliases score ≥ true loop (0.79 vs 0.58).
- **ICP / RANSAC geometric fitness**: FALSE aliases register BETTER than true loops (KITTI idx7→4 = 0.998 vs true 0.43; office 146→0 tie). Fixed the earlier fitness=0 bug (sparse last submap + no translation in init) — even fixed, still inverted. Geometry itself aliases.
- **Descriptor / similarity threshold**: can't separate — aliases descriptor-closer than true loops.
- **num_range inflation (40→60→80)**: improves the number but is dimension inflation → overfits 12 submaps. Not principled.
- **bin aggregation mean/max** (vs sum): sum is best; mean noisier, max saturates/degenerate.
- **channel combination** (per-pair min/max/mean of R,G,B distances): ≈ concat, no gain.
- **conv / full-2D range-height matrix** (instead of `rsolid = range_matrix @ number_vector`): no gain — colour marginal carries the signal.
- **HSV/hue**: worse than RGB (hue noisy on low-saturation road/sky).
- **matching aggregation median/mean** across frame pairs (vs min): worse — true revisits have viewpoint change (partial match), false roads are consistently identical.

---

## 6. Data & artifacts

- **KITTI 05 colour images (NAS):** `/nas/Ground/[Camera] KITTI_ODOMETRY_COLOR/dataset/sequences/05/image_2/` (2761 png). Symlinked as `reference/VGGT-SLAM/kitti05_full`. Other loop sequences (00/02/06/08) are in the same folder — needed for multi-sequence generalization.
- **KITTI GT poses:** `/nas/Ground/KITTI_Dataset/Odometry/data_odometry_poses/dataset/poses/05.txt` (Nx12, reshape to (N,3,4), t = [:, :,3]).
- **Dumps** (session scratchpad — TEMPORARY, regenerate if gone): `kitti05.pkl` (12 submaps, clouds only, old format), `kitti05_col.pkl` (clouds+colours, new dict format), `vggt_raw*.pkl` (office). To regenerate: run with `SOLID_DUMP=path.pkl VGGT_IMG_THRES=0.99` (log-only).
- The per-submap→frame mapping (needed to align dumps with GT) is parsed from the run log's `['kitti05_full/000xxx.png', ...]` lists.

## 7. Related memory files
`~/.claude/projects/-home-hogyun2-Test-Etc-research-SKiD-SLAM2/memory/`:
- `solid-color-intensity-finding.md` — the colour result + all dead ends (most detailed).
- `vggt-slam-solid-loop-requirement.md` — office last↔first requirement, scale-norm/floor findings.

---

## 8. Next steps (open levers, in priority order)

1. **Azimuth-preserving colour descriptor** (implemented 2026-08-11; KITTI A/B still needed). Set
   `SOLID_AZIMUTH=1 SOLID_NUM_ANGLE=60 SOLID_AZ_WEIGHT=1`. It appends a root-normalized
   RGB range×azimuth histogram and matches it over all circular yaw shifts. Standard rsolid
   sums over azimuth and loses this place-distinctive context.
2. **Multi-sequence validation.** Everything above is on ONE KITTI sequence (12 submaps) — small, overfit-prone. Get clouds+colours for KITTI 00/02/06/08 (on NAS), train/test split. Needed before any "it works" claim, and required for the MLP idea.
3. **Small MLP / metric-learning head** on the R,G,B-SOLiD features — likely closes the last −0.05 gap, but ONLY meaningful multi-sequence (single-sequence = memorizing ~10 labels).
4. **Better input clouds:** VGGT depth is the real bottleneck (generic roads). Consider confidence filtering, or validating the whole pipeline on real KITTI velodyne LiDAR to prove SOLiD+config are correct and VGGT is the limiter.

### Coordinate-system correction (2026-08-11)

VGGT point maps use OpenCV camera coordinates (`x=right, y=down, z=forward`) whereas SOLiD
expects a z-up scan. Previously `SOLID_NO_FLOOR=1` skipped every rotation, so KITTI binned
camera depth as height and image vertical as azimuth. It now skips only RANSAC and applies
the deterministic mapping `(x,y,z) -> (x,z,-y)` by default. Set `SOLID_CAMERA_Z_UP=0` only
to reproduce the old, incorrect behavior. This correction must be A/B-tested separately
from `SOLID_AZIMUTH` so its gain is measurable.

### Post-correction KITTI 05 A/B (2026-08-11)

Evaluated on one frozen VGGT dump (12 submaps / 376 keyframes) against KITTI GT, with
the same raw clouds and colours for every variant. The old profile retrieved the correct
GT target for 0/4 loop-bearing query submaps (`sep=-0.0526`). After z-up conversion and
retuning, an initial coarse `max_range=1.0, num_range=10, fov=25` profile retrieved 1/4.
A subsequent 16–40 bin sweep found `max_range=1.2, num_range=20, fov=12`, which retrieves
**2/4** true-loop targets (`sep=-0.0243`): query submap 6→2 (GT 0.07m) and 10→0 (GT 0.40m).
This 60-d RGB descriptor is preferred over the over-coarse 30-d/10-bin result. Normalizing
each R/G/B rsolid channel independently is enabled with `SOLID_CHANNEL_NORM=1`.
`SOLID_AZIMUTH` improved the query-level separation relative to the old profile but
still retrieved 0/4, so keep it OFF. The improvement is real but not yet safe for automatic
loop application because separation remains negative.

## 9. Framing / conclusion for a report
Geometric descriptors (SOLiD) on foundation-model *depth* struggle with perceptual aliasing because the depth is generic. Foundation models' strength is *learned/appearance* features — hence adding image colour to SOLiD is the productive direction, and for self-similar scenes learned visual descriptors (SALAD/NetVLAD) or semantic maps may beat geometric SOLiD. SOLiD remains strong on its native LiDAR/radar modality.

## 10. Validated office single-loop configuration (2026-08-11)

Pure descriptor sweeps never ranked final→0 first: repeated rooms remained stronger aliases.
The pure VGGT odometry, however, cleanly separated the true return: final→start spatial
distance was 0.305–0.389, while every eligible intermediate alias was at least 0.552 away.
`SOLID_ODOM_RADIUS=0.4` therefore adds a standard pose-proximity gate before descriptor
ranking; it does not hardcode the final submap or target ID. The final validated descriptor
is **basic geometry-only SOLiD rsolid**: no RGB, intensity weights, azimuth colour context,
or learned appearance gate.

```bash
VGGT_SEED=0 SOLID_NO_FLOOR=1 SOLID_CAMERA_Z_UP=1 \
SOLID_INTENSITY=0 SOLID_AZIMUTH=0 \
SOLID_MAX_RANGE=2.0 SOLID_NUM_RANGE=10 SOLID_NUM_HEIGHT=12 SOLID_FOV=30 \
SOLID_MIN_RANGE=0.05 SOLID_VOXEL=0.04 SOLID_SUB=6 SOLID_MEAN_REMOVE=0 \
SOLID_MIN_SUBMAP_GAP=50 SOLID_ODOM_RADIUS=0.4 SOLID_ICP_MIN=0 \
VGGT_IMG_THRES=0 python -u main.py --image_folder office_loop \
  --submap_size 16 --max_loops 1 --retrieval solid --lc_thres 0.55 --vis_map
```

Validated online outcome: no candidates for the first 12 normal submaps; at the final
submap, exactly one `204→0` closure was applied (`desc_d=0.457`, `spatial_d=0.340`,
`image_match_ratio=0.998`, logged but not used as a gate). Final graph loop count: **1**.
Log: `/tmp/office_basic_solid_radius_final.log` (temporary). The tentative current-submap world centers used by
the gate are computed from the shared boundary frame plus VGGT local relative poses; stored
submaps use their optimized graph homographies.

### Open3D before/after visualization

Add `--o3d_lc_vis` to the office command. At each applied loop it captures the odometry
initial values immediately before graph optimization and the corrected values immediately
after optimization. The interactive window shows BEFORE on the left with red camera
wireframes and AFTER on the right with green wireframes. `--o3d_lc_no_window` saves only;
`--o3d_lc_dir DIR` and `--o3d_lc_voxel 0.03` control output and visualization density.
Each snapshot is saved as RGB point-cloud PLY + compressed NPZ, with camera LineSet PLYs.
