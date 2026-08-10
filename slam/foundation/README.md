# slam/foundation — SOLiD × VGGT-SLAM (foundation-model SLAM)

**Target:** [VGGT-SLAM 2.0](https://github.com/MIT-SPARK/VGGT-SLAM) (MIT-SPARK) —
real-time dense feed-forward RGB SLAM built on **VGGT** (Visual Geometry Grounded
Transformer), aligning submaps on the SL(4) manifold. **Python / PyTorch, non-ROS.**

**Why it's a strong SOLiD target (unlike GS-ICP):** VGGT-SLAM already has the full
backend — submaps (`vggt_slam/submap.py`), a factor graph (`graph.py`), loop
closure (`loop_closure.py`) and global optimization — so it can *consume* loop
constraints and actually close loops. GS-ICP has no such backend.

**Current place recognition:** `vggt_slam/loop_closure.py` uses **DINO-SALAD**
image-retrieval embeddings (`ImageRetrieval` class: `get_single_embeding`,
`get_all_submap_embeddings`, `find_loop_closures` via cosine similarity across
submaps). This is the visual analog of Kimera's DBoW2/ORB.

**SOLiD integration point:** VGGT predicts per-frame **point maps / depth**, so we
can run SOLiD on that geometry as a **geometric place-recognition alternative or
complement** to DINO-SALAD. Mirror the `ImageRetrieval` interface with a
`SolidRetrieval` that returns SOLiD `rsolid` as the per-frame retrieval vector and
implements `find_loop_closures` via SOLiD similarity + yaw. Drops into the existing
graph/loop-closure machinery — real loops get closed.
- Cloud source: VGGT point map (world points) → gravity-align → `solid.Extractor`.
- Profile: start from `gs_rgbd` (indoor, narrow FOV); tune.
- Uses the `solid` pip package (`pip install .` from the SOLiD repo root).

**Data:** the repo bundles `office_loop.zip` (a loop-containing sequence) — handy
for testing loop detection without extra downloads.

**Env / caveat:** VGGT is a large transformer + DINO-SALAD checkpoint; needs a GPU
(RTX 3070 Ti 8GB is tight but VGGT-SLAM 2.0 targets real-time). Model weights +
`salad` package + torch env required.

Status: **planned** — repo cloned to `SKiD-SLAM2/reference/VGGT-SLAM`.
