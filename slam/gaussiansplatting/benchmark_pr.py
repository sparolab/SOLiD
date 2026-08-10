"""Timing comparison of place-recognition descriptors: NetVLAD vs SALAD vs SOLiD.

Per-keyframe descriptor-extraction time on a TUM fr3 frame. NetVLAD/SALAD are deep
nets (GPU); SOLiD is a lightweight geometric descriptor (CPU). Reports min/median/
mean over N iterations (min ~ uncontended). Run in background (model downloads).
"""
import os
import sys
import time

import cv2
import numpy as np
import torch

LOOPSPLAT = "/home/hogyun2/Test/Etc/research/SKiD-SLAM2/reference/LoopSplat"
sys.path.insert(0, LOOPSPLAT)
os.chdir(LOOPSPLAT)

import solid  # noqa: E402

DATA = "/home/hogyun2/Test/Etc/research/SKiD-SLAM2/reference/GS_ICP_SLAM/dataset/TUM/rgbd_dataset_freiburg3_long_office_household"
N = 50


def stats(ts):
    ts = np.array(ts) * 1000.0
    return f"min {ts.min():.2f} | median {np.median(ts):.2f} | mean {ts.mean():.2f} ms"


def sample_image_depth():
    rgb = sorted(os.listdir(os.path.join(DATA, "rgb")))[100]
    dep = sorted(os.listdir(os.path.join(DATA, "depth")))[100]
    img = cv2.cvtColor(cv2.imread(os.path.join(DATA, "rgb", rgb)), cv2.COLOR_BGR2RGB)
    depth = cv2.imread(os.path.join(DATA, "depth", dep), cv2.IMREAD_UNCHANGED).astype(np.float32) / 5000.0
    return img, depth


def bench_solid(depth):
    fx, fy, cx, cy = 535.4, 539.2, 320.1, 247.6
    h, w = depth.shape
    vs, us = np.mgrid[0:h:3, 0:w:3]
    d = depth[vs, us]; m = (d > 0) & (d < 8)
    d = d[m]; x = (us[m]-cx)*d/fx; y = (vs[m]-cy)*d/fy
    pc = np.stack([x, y, d], -1).astype(np.float32)
    ext = solid.Extractor(solid.load_config("gs_rgbd"))
    for _ in range(5): ext.extract(pc)
    ts = []
    for _ in range(N):
        t = time.perf_counter(); ext.extract(pc); ts.append(time.perf_counter()-t)
    d0 = ext.extract(pc)
    return stats(ts), f"dim rsolid={d0.rsolid.shape[0]}+asolid={d0.asolid.shape[0]}", len(pc)


def bench_netvlad(img):
    from src.gsr.descriptor import GlobalDesc
    gd = GlobalDesc()
    x = torch.from_numpy(cv2.resize(img, (224, 224))).permute(2, 0, 1)[None].float().cuda() / 255.0
    with torch.no_grad():
        for _ in range(5): gd(x)
        torch.cuda.synchronize(); ts = []
        for _ in range(N):
            t = time.perf_counter(); gd(x); torch.cuda.synchronize(); ts.append(time.perf_counter()-t)
        dim = gd(x).shape[-1]
    return stats(ts), f"dim={dim}"


def bench_salad(img):
    m = torch.hub.load("serizba/salad", "dinov2_salad", trust_repo=True).eval().cuda()
    x = torch.from_numpy(cv2.resize(img, (322, 322))).permute(2, 0, 1)[None].float().cuda() / 255.0
    with torch.no_grad():
        for _ in range(5): m(x)
        torch.cuda.synchronize(); ts = []
        for _ in range(N):
            t = time.perf_counter(); m(x); torch.cuda.synchronize(); ts.append(time.perf_counter()-t)
        dim = m(x).shape[-1]
    params = sum(p.numel() for p in m.parameters())/1e6
    return stats(ts), f"dim={dim}, {params:.0f}M params"


def main():
    img, depth = sample_image_depth()
    print(f"=== Place-recognition descriptor timing (TUM fr3, N={N}) ===\n")
    print("[SOLiD]  (CPU, geometric on depth cloud)")
    try:
        s, info, npts = bench_solid(depth); print(f"  {s}\n  {info}, cloud={npts} pts\n")
    except Exception as e:
        print("  FAIL:", e, "\n")
    print("[NetVLAD] (GPU, hloc — LoopSplat's detector)")
    try:
        s, info = bench_netvlad(img); print(f"  {s}\n  {info}\n")
    except Exception as e:
        print("  FAIL:", repr(e)[:200], "\n")
    print("[SALAD] (GPU, DINOv2-SALAD — VGGT-SLAM's detector)")
    try:
        s, info = bench_salad(img); print(f"  {s}\n  {info}\n")
    except Exception as e:
        print("  FAIL:", repr(e)[:200], "\n")
    print("### BENCH DONE ###")


if __name__ == "__main__":
    main()
