"""Run LoopSplat on fr3 with a SELECTABLE loop detector.

One runner for all four place-recognition detectors, plugged into the SAME
LoopSplat GS frame-to-model tracking (odometry) + 3DGS-submap-registration + PGO
(loop closure) pipeline, so only the detector differs:

    netvlad  -> base Loop_closure                 (LoopSplat default)
    solid    -> SolidLoopClosure                   (SOLiD rsolid + yaw-ICP)
    dinov2   -> VisualLoopClosure(kind="dinov2")
    salad    -> VisualLoopClosure(kind="salad")

LoopSplat natively writes, per PGO correction, before/after trajectories vs GT to
    output/fr3_<det>/pgo/<n>/{before,after_gs}/plot/{trj_final.json,stats_final.json}
which feed both the ATE table and the camera-trajectory GIF (GT/before/after).

  python run_slam_lc.py --detector solid
  python run_slam_lc.py --detector dinov2 --out fr3_dinov2
Env overrides: LC_MIN_SIM, LC_MIN_INTERVAL, LC_FRAME_LIMIT, SOLID_USE_YAW.
"""
import argparse
import glob
import os
import sys

import numpy as np
import torch

LOOPSPLAT = "/home/hogyun2/Test/Etc/research/SKiD-SLAM2/reference/LoopSplat"
SOLID_DIR = "/home/hogyun2/Test/Etc/research/SKiD-SLAM2/reference/SOLiD/slam/gaussiansplatting"
sys.path.insert(0, LOOPSPLAT)
sys.path.insert(0, SOLID_DIR)
os.chdir(LOOPSPLAT)
os.environ["WANDB_MODE"] = "disabled"

from src.entities.gaussian_slam import GaussianSLAM       # noqa: E402
from src.entities.lc import Loop_closure                  # noqa: E402
from src.utils.io_utils import load_config                # noqa: E402
from src.utils.utils import setup_seed                    # noqa: E402

SH_C0 = 0.28209479177387814
SNAP_DIR = None
MAX_PTS = 200000


def make_loop_closer(detector, cfg, dataset, logger):
    if detector == "netvlad":
        return Loop_closure(cfg, dataset, logger)
    if detector == "solid":
        from solid_loopsplat import SolidLoopClosure
        return SolidLoopClosure(cfg, dataset, logger)
    if detector in ("dinov2", "salad"):
        from visual_loopsplat import VisualLoopClosure
        return VisualLoopClosure(cfg, dataset, logger, kind=detector)
    raise ValueError(detector)


def dump(gslam, tag):
    pts, cols = [], []
    for ck in sorted(glob.glob(str(gslam.output_path / "submaps" / "*.ckpt"))):
        try:
            gp = torch.load(ck, map_location="cpu")["gaussian_params"]
        except Exception:
            continue
        pts.append(np.asarray(gp["xyz"], np.float32))
        cols.append(np.clip(SH_C0 * np.asarray(gp["features_dc"], np.float32)[:, 0, :] + 0.5, 0, 1))
    if pts:
        P = np.concatenate(pts); C = np.concatenate(cols)
        if len(P) > MAX_PTS:
            idx = np.random.default_rng(0).choice(len(P), MAX_PTS, replace=False)
            P, C = P[idx], C[idx]
    else:
        P = np.zeros((0, 3), np.float32); C = np.zeros((0, 3), np.float32)
    c2w = gslam.estimated_c2ws.detach().cpu().numpy()
    os.makedirs(SNAP_DIR, exist_ok=True)
    np.savez(os.path.join(SNAP_DIR, tag + ".npz"), points=P, colors=C,
             traj=c2w[:, :3, 3], poses=c2w)
    print(f"[gifsnap] {tag}: {len(P)} pts, {len(c2w)} poses", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detector", required=True,
                    choices=["netvlad", "solid", "dinov2", "salad"])
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    cfg = load_config("configs/TUM_RGBD/fr3_fast.yaml")
    cfg["use_wandb"] = False
    out = args.out or f"fr3_{args.detector}"
    cfg["data"]["output_path"] = f"output/{out}/"
    fl = os.getenv("LC_FRAME_LIMIT")
    if fl:
        cfg["frame_limit"] = int(fl)
        cfg["data"]["frame_limit"] = int(fl)     # dataset reads it from data cfg
        cfg["data"]["output_path"] = f"output/{out}_smoke/"
    # detector-agnostic detection tuning
    cfg["lc"]["solid_profile"] = "gs_rgbd"
    cfg["lc"]["use_solid_yaw"] = os.getenv("SOLID_USE_YAW", "1") != "0"
    if os.getenv("LC_MIN_SIM"):
        cfg["lc"]["min_similarity"] = float(os.getenv("LC_MIN_SIM"))
    if os.getenv("LC_MIN_INTERVAL"):
        cfg["lc"]["min_interval"] = int(os.getenv("LC_MIN_INTERVAL"))
    # ICP-fitness gate: reject geometrically weak (false) loop edges. The trailing
    # over-detections on short end segments have low registration overlap, so a
    # higher threshold keeps the good mid-sequence loops and drops the bad tail.
    if os.getenv("LC_MIN_OVERLAP"):
        cfg["lc"].setdefault("registration", {})["min_overlap_ratio"] = float(os.getenv("LC_MIN_OVERLAP"))
    setup_seed(cfg["seed"])

    gslam = GaussianSLAM(cfg)

    lc = make_loop_closer(args.detector, cfg, gslam.dataset, gslam.logger)
    lc.submap_path = gslam.output_path / "submaps"
    lc.est_c2ws = gslam.estimated_c2ws          # live pose tensor (SOLiD needs it)
    gslam.loop_closer = lc

    global SNAP_DIR
    SNAP_DIR = str(gslam.output_path / "gif_snaps")
    cnt = {"n": 0}
    orig_apply = GaussianSLAM.apply_correction_to_submaps
    orig_update = GaussianSLAM.update_keyframe_poses

    def patched_apply(self, correction_list):
        dump(self, f"{cnt['n']:02d}_before")
        return orig_apply(self, correction_list)

    def patched_update(self, lc_output, submaps_kf_ids, cur_frame_id):
        r = orig_update(self, lc_output, submaps_kf_ids, cur_frame_id)
        dump(self, f"{cnt['n']:02d}_after")
        cnt["n"] += 1
        return r

    GaussianSLAM.apply_correction_to_submaps = patched_apply
    GaussianSLAM.update_keyframe_poses = patched_update

    print(f"=== LoopSplat run: detector={args.detector} output={gslam.output_path} ===", flush=True)
    gslam.run()
    print(f"=== done: {args.detector} ===", flush=True)


if __name__ == "__main__":
    main()
