"""Map + 3-color-camera loop-closure GIF (Open3D offscreen), tuned view.

Dense 3DGS map + keyframe cameras as 5-point wireframes, orbiting at a fixed
oblique elevation. Reuses the approved make_gif_o3d.py look:
  - floor-plane gravity alignment (upright)
  - surroundings removal: statistical-outlier + central percentile crop
  - small dense points, black GT cameras
Cameras (from pgo, GT-aligned frame):
    GT           -> black   (after_gs trj_gt)
    before (odom)-> red     (before trj_est)
    after  (LC)  -> green   (after_gs trj_est)
The map (gif_snaps/<last>_after.npz, SLAM frame) is brought into the pgo
GT-aligned frame by a similarity transform (Umeyama) on shared keyframes, then
both map and cameras are gravity-aligned together.

  python make_map_cam_gif_o3d.py --run fr3_dinov2 --out dinov2_map_lc.gif
"""
import argparse
import glob
import json
import os

import imageio.v2 as imageio
import numpy as np
import open3d as o3d

LS = "/home/hogyun2/Test/Etc/research/SKiD-SLAM2/reference/LoopSplat"


def last_dir(pattern, key):
    ds = sorted(glob.glob(pattern), key=key)
    if not ds:
        raise SystemExit(f"nothing matches {pattern}")
    return ds[-1]


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    cov = D.T @ S / len(src)
    U, sig, Vt = np.linalg.svd(cov)
    W = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        W[2, 2] = -1
    R = U @ W @ Vt
    s = np.trace(np.diag(sig) @ W) / ((S ** 2).sum() / len(src))
    t = mu_d - s * R @ mu_s
    return s, R, t


def gravity_R(pts):
    """Rotation that maps the dominant floor plane normal to +Z (upright)."""
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    model, inl = pc.segment_plane(0.05, 3, 1000)
    n = np.array(model[:3]); n /= np.linalg.norm(n)
    if np.dot(pts.mean(0) - pts[inl].mean(0), n) > 0:
        n = -n
    z = np.array([0, 0, 1.0]); v = np.cross(n, z); s = np.linalg.norm(v); c = np.dot(n, z)
    if s < 1e-8:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))


def cam_wireframe(poses, color, scale, sub):
    w, h, f = 0.75 * scale, 0.55 * scale, scale
    corn = np.array([[w, h, f], [w, -h, f], [-w, -h, f], [-w, h, f]])
    P, L = [], []
    for k in range(0, len(poses), sub):
        Rc, t = poses[k][:3, :3], poses[k][:3, 3]
        cw = (Rc @ corn.T + t[:, None]).T
        base = len(P); P.append(t)
        P.extend(cw.tolist())
        for i in range(4):
            L.append([base, base + 1 + i])
            L.append([base + 1 + i, base + 1 + (i + 1) % 4])
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(np.array(P))
    ls.lines = o3d.utility.Vector2iVector(np.array(L))
    ls.colors = o3d.utility.Vector3dVector(np.tile(color, (len(L), 1)))
    return ls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--out", default="map_cam_lc.gif")
    ap.add_argument("--sub", type=int, default=6, help="draw every Nth camera")
    ap.add_argument("--pgo", type=int, default=-1, help="pgo step to render (-1 = last; pick the one with the biggest before/after shift for a dramatic red->green)")
    ap.add_argument("--point_size", type=float, default=2.5)
    ap.add_argument("--clean_std", type=float, default=1.2)
    ap.add_argument("--keep_pct", type=float, default=90.0, help="central crop: keep inner %% by radius")
    ap.add_argument("--elev", type=float, default=22.0)
    ap.add_argument("--azim0", type=float, default=-60.0, help="starting azimuth (deg)")
    ap.add_argument("--radius_scale", type=float, default=1.6)
    ap.add_argument("--frames", type=int, default=72)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=900)
    args = ap.parse_args()

    snapdir = f"{LS}/output/{args.run}/gif_snaps"
    snaps = glob.glob(f"{snapdir}/*_after.npz") or glob.glob(f"{snapdir}/*_before.npz")
    snapf = sorted(snaps, key=lambda p: int(p.split('/')[-1][:2]))[-1]
    after = np.load(snapf)
    print(f"map snap: {os.path.basename(snapf)}")
    # native keyframe positions for alignment: full poses if present, else traj
    native_pos = after["poses"][:, :3, 3] if "poses" in after.files else after["traj"]
    mpts, mcol = after["points"].astype(np.float64), after["colors"].astype(np.float64)

    if args.pgo >= 0:
        pgo = f"{LS}/output/{args.run}/pgo/{args.pgo}/"
    else:
        pgo = last_dir(f"{LS}/output/{args.run}/pgo/*/", lambda p: int(p.rstrip('/').split('/')[-1]))
    aj = json.load(open(f"{pgo}after_gs/plot/trj_final.json"))
    bj = json.load(open(f"{pgo}before/plot/trj_final.json"))
    ids = np.array(aj["trj_id"])
    est_gt = np.array(aj["trj_est"]); gt = np.array(aj["trj_gt"])
    before_gt = np.array(bj["trj_est"])
    print(f"{args.run}: map {len(mpts)} pts, {len(ids)} keyframes, pgo {pgo.split('output/')[1]}")

    # map -> pgo GT-aligned frame (Umeyama on shared keyframes)
    s, R, t = umeyama(native_pos[ids], est_gt[:, :3, 3])
    mpts = (s * (R @ mpts.T)).T + t

    # surroundings removal (as approved): statistical outlier + central crop
    pc = o3d.geometry.PointCloud(); pc.points = o3d.utility.Vector3dVector(mpts)
    _, keep = pc.remove_statistical_outlier(nb_neighbors=16, std_ratio=args.clean_std)
    mpts, mcol = mpts[keep], mcol[keep]
    ctr0 = np.median(mpts, 0); rr = np.linalg.norm(mpts - ctr0, axis=1)
    m = rr < np.percentile(rr, args.keep_pct); mpts, mcol = mpts[m], mcol[m]
    print(f"after clean/crop: {len(mpts)} pts")

    # gravity align (floor plane) -> upright; apply to map + all cameras
    Rg = gravity_R(mpts)
    A = np.eye(4); A[:3, :3] = Rg
    mpts = (Rg @ mpts.T).T
    gt = np.array([A @ T for T in gt])
    before_gt = np.array([A @ T for T in before_gt])
    est_gt = np.array([A @ T for T in est_gt])

    extent = np.linalg.norm(mpts.max(0) - mpts.min(0))
    scale = extent * 0.010
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(mpts)
    pcd.colors = o3d.utility.Vector3dVector(np.clip(mcol, 0, 1))

    ren = o3d.visualization.rendering.OffscreenRenderer(args.width, args.height)
    ren.scene.set_background([1, 1, 1, 1])
    pm = o3d.visualization.rendering.MaterialRecord()
    pm.shader = "defaultUnlit"; pm.point_size = args.point_size
    ren.scene.add_geometry("map", pcd, pm)
    lm = o3d.visualization.rendering.MaterialRecord()
    lm.shader = "unlitLine"; lm.line_width = 2.5
    ren.scene.add_geometry("before", cam_wireframe(before_gt, [0.9, 0.1, 0.1], scale, args.sub), lm)
    ren.scene.add_geometry("gt", cam_wireframe(gt, [0, 0, 0], scale, args.sub), lm)
    ren.scene.add_geometry("after", cam_wireframe(est_gt, [0.05, 0.75, 0.05], scale, args.sub), lm)

    # frame on map + ALL cameras (trajectory is wider than the map) + margin,
    # so outer camera glyphs don't clip at the image edges.
    allpts = np.vstack([mpts, gt[:, :3, 3], before_gt[:, :3, 3], est_gt[:, :3, 3]])
    center = allpts.mean(0)
    span = np.linalg.norm(allpts.max(0) - allpts.min(0))
    radius = span * args.radius_scale * 0.62
    el = np.deg2rad(args.elev)
    tmp = os.path.dirname(os.path.abspath(args.out)) + "/_mapcamframes"; os.makedirs(tmp, exist_ok=True)
    frames = []
    for k in range(args.frames):
        az = np.deg2rad(args.azim0) + 2 * np.pi * k / args.frames
        eye = center + radius * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        ren.setup_camera(60.0, center, eye, [0, 0, 1])
        img = np.asarray(ren.render_to_image())
        imageio.imwrite(f"{tmp}/f{k:03d}.png", img); frames.append(img)
        if (k + 1) % 16 == 0:
            print(f"  {k+1}/{args.frames}")
    imageio.mimsave(args.out, frames, fps=args.fps, loop=0)
    print("saved", args.out)


if __name__ == "__main__":
    main()
