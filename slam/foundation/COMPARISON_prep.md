<!-- PREP notes for the slam/foundation README (finalize together). Not the README. -->
# VGGT-SLAM × place-recognition detector comparison (office_loop)

SOLiD / NetVLAD / SALAD plugged into the **same VGGT-SLAM loop-closure pipeline**
(VGGT feed-forward point/pose prediction → submap → SL(4) graph). Only the submap
retrieval descriptor differs, via `--retrieval {salad,netvlad,solid}`:
`python main.py --image_folder office_loop --max_loops 1 --retrieval <det>`.

Integration: `vggt_slam/pluggable_retrieval.py` (all three subclass `ImageRetrieval`,
override `get_all_submap_embeddings`; matching = L2 distance of L2-normalized
vectors in `map.retrieve_best_score_frame`, so any normalized descriptor drops in).
SALAD/NetVLAD embed submap frames; **SOLiD embeds VGGT's predicted per-frame point
maps** (rsolid, gravity-aligned) — solver computes `world_points` right after VGGT
inference so SOLiD has geometry before loop detection.

## Result — office_loop (473 frames, ~30 submaps; NO ground truth → qualitative)
| Backend | detected loop (best) | applied LC | retrieval time | device |
|---|---|---|---|---|
| **SOLiD** | mid-revisit → submap 51 (L2 dist 0.0018, very confident) | 1 | **0.0020 s/frame** | **CPU** |
| SALAD | true global 204 → 0 (end→start, dist 0.62) | 1 | 0.0101 s/frame | GPU |
| NetVLAD | true global 204 → 0 (dist 0.83) | 1 | 0.0099 s/frame | GPU |

**Observations**
- All three run end-to-end and each applies **1 loop closure**.
- **Visual (SALAD/NetVLAD)** both lock onto the appearance-correct **end-to-start
  global loop** (submap 204 → 0).
- **SOLiD** proposes very-confident *geometric* matches (repeated office structure →
  submap 51 mid-sequence); its top candidates were often rejected by VGGT's **own**
  geometric gate (`image_match_ratio < 0.95`, "Loop closure image match ratio too
  low, skipping") which is independent of the detector.
- **SOLiD is ~5× faster and runs on CPU** with zero learned parameters.

**Caveat:** office_loop ships without ground-truth poses, so this is a qualitative
loop-detection + timing comparison, not ATE. For a quantitative foundation-model
number, run the VGGT-SLAM TUM eval (`evals/eval_tum.sh`) per detector.

Logs: `vggt_{solid,salad,netvlad}_poses.txt` (VGGT-SLAM `--log_results`).
Env: conda `vggt-slam` (py3.11, torch 2.3.1+cu121, salad + VGGT_SPARK + solid + hloc).
