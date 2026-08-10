"""Autonomous SOLiD gs_rgbd tuner — maximize true/false loop SEPARATION on fr3.

Caches fr3 keyframe clouds (GT-pose-rotated -> world z-up = LiDAR convention:
sensor at origin, range in x-y horizontal plane, z=up) ONCE, then sweeps a wide
Config grid, re-running only extract+query. Ranks by a score that rewards
SEPARATION (true-revisit similarity minus false) AND recall — because the SLAM's
false-loop rate is driven by separation, not recall alone.

Also reports precision@sep-threshold: of the query frames whose best match scores
above the midpoint, how many are true revisits. Appends ranked results to
tune_results.txt so progress survives across sessions.

  python tune_auto.py --dataset <fr3 dir> [--round N]
"""
import argparse
import itertools
import os

import numpy as np
from PIL import Image

import solid

FX, FY, CX, CY = 535.4, 539.2, 320.1, 247.6
DEPTH_SCALE = 5000.0
RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tune_results.txt")


def parse_list(path):
    return [l.split() for l in open(path).read().splitlines()
            if l and not l.startswith("#")]


def pose_R(row):
    tx, ty, tz, qx, qy, qz, qw = map(float, row[1:8])
    x, y, z, w = qx, qy, qz, qw
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])


def backproject(depth_m, stride):
    h, w = depth_m.shape
    vs, us = np.mgrid[0:h:stride, 0:w:stride]
    d = depth_m[vs, us]
    m = (d > 0) & (d < 8.0)
    d = d[m]
    x = (us[m] - CX) * d / FX
    y = (vs[m] - CY) * d / FY
    return np.stack([x, y, d], -1).astype(np.float32)


def make_config(**kw):
    cfg = solid.Config()
    defaults = dict(fov_up=15.0, fov_down=-15.0, num_angle=60, num_range=20,
                    num_height=30, min_range=0.1, max_range=10.0, voxel_size=0.05,
                    use_weight=False, knn=30, min_similarity=0.0)
    defaults.update(kw)
    for k, v in defaults.items():
        setattr(cfg, k, v)
    return cfg


def evaluate(cfg, clouds, positions, has_gt_loop, min_gap, match_ok):
    ext = solid.Extractor(cfg)
    descs = []
    for pc in clouds:
        r = np.asarray(ext.extract(pc).rsolid, np.float32)
        n = np.linalg.norm(r)
        descs.append(r / n if n > 0 else r)
    D = np.array(descs)
    n = len(clouds)
    best_score = np.full(n, np.nan)
    best_is_true = np.zeros(n, bool)
    for i in range(min_gap + 1, n):
        sims = D[:i - min_gap] @ D[i]
        if not len(sims):
            continue
        j = int(np.argmax(sims))
        best_score[i] = sims[j]
        best_is_true[i] = np.linalg.norm(positions[j] - positions[i]) < match_ok
    revisit = has_gt_loop.copy()
    recall = (best_is_true & revisit).sum() / max(1, revisit.sum())
    s_rev = best_score[revisit & ~np.isnan(best_score)]
    s_non = best_score[(~has_gt_loop) & ~np.isnan(best_score)]
    mrev = float(np.mean(s_rev)) if s_rev.size else float("nan")
    mnon = float(np.mean(s_non)) if s_non.size else float("nan")
    sep = mrev - mnon
    # precision@midpoint: among queries scoring above (mrev+mnon)/2, frac true
    thr = (mrev + mnon) / 2
    hi = ~np.isnan(best_score) & (best_score > thr)
    prec = (best_is_true & hi).sum() / max(1, hi.sum())
    return recall, sep, mrev, mnon, prec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--kf_stride", type=int, default=10)
    ap.add_argument("--px_stride", type=int, default=3)
    ap.add_argument("--min_gap", type=int, default=25)
    ap.add_argument("--gt_loop_dist", type=float, default=0.5)
    ap.add_argument("--match_ok", type=float, default=0.75)
    ap.add_argument("--round", type=int, default=1)
    ap.add_argument("--grid", default="wide", help="wide | fine | user")
    args = ap.parse_args()

    depth = parse_list(os.path.join(args.dataset, "depth.txt"))
    gt = parse_list(os.path.join(args.dataset, "groundtruth.txt"))
    gt_ts = np.array([float(r[0]) for r in gt])
    frames = []
    for r in depth[::args.kf_stride]:
        t = float(r[0]); j = int(np.argmin(np.abs(gt_ts - t)))
        if abs(gt_ts[j] - t) < 0.02:
            frames.append((r[1], gt[j]))
    print(f"caching {len(frames)} clouds (backend={solid.backend}) ...")
    clouds, positions = [], []
    for drel, gtrow in frames:
        R = pose_R(gtrow)
        dimg = np.asarray(Image.open(os.path.join(args.dataset, drel)), np.float32) / DEPTH_SCALE
        clouds.append((R @ backproject(dimg, args.px_stride).T).T.astype(np.float32))
        positions.append(np.array(list(map(float, gtrow[1:4]))))
    positions = np.array(positions)
    has_gt_loop = np.zeros(len(frames), bool)
    for i in range(len(frames)):
        if i > args.min_gap:
            d = np.linalg.norm(positions[:i - args.min_gap] - positions[i], axis=1)
            has_gt_loop[i] = d.min() < args.gt_loop_dist
    print(f"GT revisits: {has_gt_loop.sum()}/{len(frames)}\n")

    if args.grid == "wide":
        grid = dict(max_range=[8., 10., 12.], num_range=[20, 30, 40],
                    num_height=[16, 24, 30, 40], fov=[15., 30., 45., 60.],
                    min_range=[0.1, 0.5], voxel_size=[0.05], num_angle=[60])
    elif args.grid == "user":
        grid = dict(max_range=[10.], num_range=[20], num_height=[30], fov=[15.],
                    min_range=[0.1], voxel_size=[0.05], num_angle=[60])
    else:  # fine (filled in by driver via env-less edit)
        grid = dict(max_range=[10.], num_range=[20, 24], num_height=[28, 30, 32],
                    fov=[12., 15., 20.], min_range=[0.1, 0.3], voxel_size=[0.03, 0.05],
                    num_angle=[60])
    keys = list(grid)
    combos = list(itertools.product(*[grid[k] for k in keys]))
    print(f"round {args.round}: sweeping {len(combos)} configs (grid={args.grid}) ...")
    results = []
    for combo in combos:
        d = dict(zip(keys, combo))
        cfg = make_config(max_range=d["max_range"], num_range=d["num_range"],
                          num_height=d["num_height"], fov_up=d["fov"], fov_down=-d["fov"],
                          min_range=d["min_range"], voxel_size=d["voxel_size"],
                          num_angle=d["num_angle"])
        recall, sep, mrev, mnon, prec = evaluate(cfg, clouds, positions, has_gt_loop,
                                                 args.min_gap, args.match_ok)
        # rank score: separation-dominant, recall & precision as tie-breakers
        score = sep * 3 + recall * 0.3 + prec * 0.5
        results.append((score, recall, sep, prec, mrev, mnon, d))
    results.sort(key=lambda r: r[0], reverse=True)

    lines = [f"\n===== ROUND {args.round} (grid={args.grid}, {len(combos)} configs) ====="]
    lines.append("score | recall | sep | prec | rev/non | params")
    for score, recall, sep, prec, mrev, mnon, d in results[:15]:
        lines.append(f"  {score:.3f} | r={recall:.2f} | sep={sep:+.3f} | p={prec:.2f} | "
                     f"({mrev:.3f}/{mnon:.3f}) | mr={d['max_range']:g} nr={d['num_range']} "
                     f"nh={d['num_height']} fov=±{d['fov']:g} minr={d['min_range']:g} vox={d['voxel_size']:g}")
    out = "\n".join(lines)
    print(out)
    with open(RESULTS, "a") as f:
        f.write(out + "\n")
    print(f"\n-> appended to {RESULTS}")


if __name__ == "__main__":
    main()
