"""viser viewer for LoopSplat output — global Gaussian map + camera trajectory.

Matches VGGT-SLAM's visualization stack (viser 0.2.23). Loads the submap ckpts
(Gaussian centers + SH DC color) and the estimated trajectory, and serves an
interactive 3D view in the browser. Works on partial output while a run is still
going (re-launch to refresh).

  python viser_view.py --output <LoopSplat output dir> [--port 8080]

Then open the printed URL (http://<host>:8080) in a browser.

Map-correction before/after (post loop closure) is added once a full run with the
loop finishes; see README.
"""
import argparse
import glob
import os

import numpy as np
import torch
import viser

SH_C0 = 0.28209479177387814  # SH degree-0 factor: rgb = SH_C0 * f_dc + 0.5


def load_submap_points(output_dir, clean=False):
    pts, cols = [], []
    for ck in sorted(glob.glob(os.path.join(output_dir, "submaps", "*.ckpt"))):
        try:
            gp = torch.load(ck, map_location="cpu")["gaussian_params"]
        except Exception as e:
            print(f"  skip {os.path.basename(ck)}: {e}")
            continue
        xyz = np.asarray(gp["xyz"], dtype=np.float32)
        fdc = np.asarray(gp["features_dc"], dtype=np.float32)[:, 0, :]  # (N,3)
        rgb = np.clip(SH_C0 * fdc + 0.5, 0.0, 1.0)
        pts.append(xyz)
        cols.append((rgb * 255).astype(np.uint8))
    if not pts:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)
    P = np.concatenate(pts); C = np.concatenate(cols)
    if clean and len(P) > 100:
        # aggressively drop sparse surrounding noise + keep central region
        import open3d as o3d
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(P.astype(np.float64))
        _, keep = pc.remove_statistical_outlier(nb_neighbors=16, std_ratio=1.2)
        P, C = P[keep], C[keep]
        ctr = np.median(P, axis=0)
        r = np.linalg.norm(P - ctr, axis=1)
        m = r < np.percentile(r, 90)
        P, C = P[m], C[m]
    return P, C


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True, help="LoopSplat output dir (has submaps/, estimated_c2w.ckpt)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--max_points", type=int, default=400000)
    ap.add_argument("--point_size", type=float, default=0.01)
    ap.add_argument("--frustum_every", type=int, default=25)
    ap.add_argument("--refresh", type=float, default=8.0,
                    help="seconds between auto-reloads (0 = load once)")
    ap.add_argument("--share", action="store_true",
                    help="also create a public viser share URL (remote streaming; needs internet)")
    ap.add_argument("--clean", action="store_true",
                    help="drop sparse surrounding noise (statistical outlier removal)")
    args = ap.parse_args()

    server = viser.ViserServer(host=args.host, port=args.port)
    print(f"\n=== viser running: open  http://localhost:{args.port}  ===")
    if args.share:
        try:
            url = server.request_share_url()
            print(f"=== public share URL (remote streaming): {url} ===")
        except Exception as e:
            print(f"  share URL failed ({e}); local streaming still works")
    print(f"(auto-refresh every {args.refresh}s; Ctrl-C to stop)")

    import time

    def refresh():
        pts, cols = load_submap_points(args.output, clean=args.clean)
        if len(pts) > args.max_points:
            idx = np.random.default_rng(0).choice(len(pts), args.max_points, replace=False)
            pts, cols = pts[idx], cols[idx]
        if len(pts):
            server.scene.add_point_cloud("/map", points=pts, colors=cols,
                                         point_size=args.point_size)
        ec_path = os.path.join(args.output, "estimated_c2w.ckpt")
        n_pose = 0
        if os.path.isfile(ec_path):
            try:
                c2w = np.asarray(torch.load(ec_path, map_location="cpu"))
                traj = c2w[:, :3, 3].astype(np.float32)
                n_pose = len(traj)
                if len(traj) > 1:
                    server.scene.add_spline_catmull_rom(
                        "/trajectory", positions=traj, color=(255, 40, 40),
                        line_width=3.0)
                for i in range(0, len(c2w), max(1, args.frustum_every)):
                    server.scene.add_camera_frustum(
                        f"/cam/{i:04d}", fov=1.0, aspect=1.33, scale=0.05,
                        wxyz=_R_to_quat(c2w[i, :3, :3]), position=c2w[i, :3, 3])
            except Exception as e:
                print(f"  traj reload skipped: {e}")
        return len(pts), n_pose

    while True:
        try:
            np_, nc = refresh()
            print(f"  [refresh] {np_} points, {nc} poses")
        except Exception as e:
            print(f"  refresh error: {e}")
        if args.refresh <= 0:
            while True:
                time.sleep(1.0)
        time.sleep(args.refresh)


def _R_to_quat(R):
    # rotation matrix -> (w, x, y, z)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = np.argmax([R[0, 0], R[1, 1], R[2, 2]])
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return np.array([w, x, y, z], dtype=np.float32)


if __name__ == "__main__":
    main()
