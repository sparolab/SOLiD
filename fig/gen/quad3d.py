"""SOLiD README thumbnail (fig/solid_demo_v2.gif): 4 quadrants (LiDAR, 4D radar, legged, humanoid) in the 'rising trajectory'
style. The map builds up, the trajectory rises with time, each closed loop links back to the place it
recognized, and the trajectory and map are SOLiD's online pose graph at that moment (iSAM2 replayed
frame by frame) next to the odometry (gray).

    python fig/gen/quad3d.py solid_demo.gif --data <root of the sessions> [--panel lidar=<session> ...]
Needs opencv-python, gtsam, scipy and ffmpeg."""
import csv
import os
import sys

import cv2
import gtsam
import numpy as np
from scipy.spatial.transform import Rotation as Rot

import argparse
import tempfile

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("out", help="output .gif")
ap.add_argument("--data", default="data", help="root of the sessions below (default: ./data)")
ap.add_argument("--panel", action="append", default=[], metavar="KEY=SESSION",
                help="session of a quadrant (lidar, radar, legged, humanoid); a SOLiD session with closed/ "
                     "(solid_close_loops output)")
ap.add_argument("--frames", help="keep the PNG frames here")
ap.add_argument("--only", default="", help="render only these frames (preview)")
ARGS = ap.parse_args()
OUT = ARGS.frames or tempfile.mkdtemp(prefix="solid_gif_")
ICONS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icons")
os.makedirs(OUT, exist_ok=True)
W, H = 800, 450
QW, QH = W // 2, H // 2
SS = 2
FR, HOLD = 170, 40
ONLY = [int(x) for x in ARGS.only.split(",") if x]
rng = np.random.default_rng(0)
D = ARGS.data
ELEV = 38.0

COL = {"lidar": (63, 81, 181), "radar": (216, 27, 96), "legged": (0, 137, 123), "humanoid": (245, 124, 0)}


def kitti(p):
    a = np.loadtxt(p).reshape(-1, 3, 4)
    T = np.tile(np.eye(4), (len(a), 1, 1))
    T[:, :3, :4] = a
    return T


def read_pcd(p, n):
    with open(p, "rb") as f:
        while not f.readline().startswith(b"DATA"):
            pass
        a = np.frombuffer(f.read(), np.float32).reshape(-1, 4)[:, :3]
    return a[rng.choice(len(a), n, replace=False)] if len(a) > n else a


QUADS = [
    dict(key="lidar", title="LiDAR  ·  KISS-ICP", s=f"{D}/lidar/sessions/kitti00_kiss", npts=900, stride=2, rmax=40, sig=(0.1, 1, 0.5, 3)),
    dict(key="radar", title="4D radar  ·  GeoRIO", s=f"{D}/radar/sessions/st_20240113", npts=1500, stride=2, rmax=60, sig=(0.1, 1, 1, 5)),
    dict(key="legged", title="Legged  ·  GaRLILEO", s=f"{D}/legged/sessions/garlileo_bridgeloop", npts=1500, stride=1, rmax=25, lift=0.45, sig=(0.1, 1, 1, 5)),
    dict(key="humanoid", title="Humanoid  ·  FAST-LIO", s=f"{D}/humanoid/sessions/kilvo_outdoor_h01", npts=900, stride=1, rmax=15, sig=(0.1, 1, 0.5, 3)),
]
for spec in ARGS.panel:
    key, path = spec.split("=", 1)
    next(q for q in QUADS if q["key"] == key)["s"] = path


def raw(q, xyz, az):
    a, e = np.radians(az), np.radians(ELEV)
    x, y, z = (xyz - q["center"]).T
    xr = np.cos(a) * x - np.sin(a) * y
    yr = np.sin(a) * x + np.cos(a) * y
    return np.stack([xr, -(yr * np.sin(e) + z * np.cos(e))], 1)


def ks(f, N):
    return min(N - 1, int((min(f, FR - 1) + 1) / FR * (N - 1)))


def online_snapshots(q, odoT, loops):
    """Replay the pose graph with iSAM2 as solid_node does and keep, for every frame of the video,
    the whole trajectory as the online system knew it at that moment."""
    to, ro, tl, rl = q["sig"]
    dg = lambda t, r: gtsam.noiseModel.Diagonal.Sigmas(np.r_[[np.radians(r)] * 3, [t] * 3])
    on, ln = dg(to, ro), gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Cauchy.Create(1.0), dg(tl, rl))
    isam = gtsam.ISAM2()
    N = len(odoT)
    want = {ks(f, N) for f in range(FR)}
    snaps, by_q = {}, {}
    for a, b, T in loops:
        by_q.setdefault(a, []).append((b, T))
    for i in range(N):
        g, v = gtsam.NonlinearFactorGraph(), gtsam.Values()
        if i == 0:
            g.add(gtsam.PriorFactorPose3(0, gtsam.Pose3(odoT[0]), on))
            v.insert(0, gtsam.Pose3(odoT[0]))
        else:
            rel = np.linalg.inv(odoT[i - 1]) @ odoT[i]
            g.add(gtsam.BetweenFactorPose3(i - 1, i, gtsam.Pose3(rel), on))
            prev = isam.calculateEstimate().atPose3(i - 1).matrix()
            v.insert(i, gtsam.Pose3(prev @ rel))
        for b, T in by_q.get(i, []):
            g.add(gtsam.BetweenFactorPose3(b, i, gtsam.Pose3(T), ln))
        isam.update(g, v)
        if by_q.get(i):
            for _ in range(3):
                isam.update()
        if i in want:
            e = isam.calculateEstimate()
            snaps[i] = np.stack([e.atPose3(j).matrix() for j in range(i + 1)])
    return snaps


for q in QUADS:
    s = q["s"]
    odoT = kitti(f"{s}/odom_poses.txt")
    q["odo"] = odoT[:, :3, 3]
    q["onl"] = kitti(f"{s}/closed/online_poses.txt")[:, :3, 3]
    opt = kitti(f"{s}/closed/optimized_poses.txt")
    rows = [r for r in csv.DictReader(open(f"{s}/closed/loops_checked.csv")) if r["accepted"] == "1"]
    q["loops"] = [(int(r["query_id"]), int(r["match_id"])) for r in rows]
    lT = []
    for r in rows:
        T = np.eye(4)
        T[:3, :3] = Rot.from_quat([float(r[k]) for k in ("qx", "qy", "qz", "qw")]).as_matrix()
        T[:3, 3] = [float(r[k]) for k in "xyz"]
        lT.append((int(r["query_id"]), int(r["match_id"]), T))
    q["snaps"] = online_snapshots(q, odoT, lT)
    last = q["snaps"][max(q["snaps"])]
    print(q["key"], "final online vs optimized:", np.abs(last[:, :3, 3] - opt[: len(last), :3, 3]).max().round(3), "m", flush=True)
    pts, kfs, loc = [], [], []
    for i in range(0, len(opt), q["stride"]):
        p = read_pcd(f"{s}/Scans/{i:06d}.pcd", q["npts"])
        p = p[np.linalg.norm(p[:, :2], axis=1) < q["rmax"]]
        loc.append(p)
        pts.append((opt[i][:3, :3] @ p.T).T + opt[i][:3, 3])
        kfs.append(np.full(len(p), i))
    m = np.concatenate(pts)
    q["loc"] = np.concatenate(loc)
    q["map_kf"] = np.concatenate(kfs)
    ground = np.percentile(m[:, 2], 20)
    hz = np.clip((m[:, 2] - ground) / max(np.percentile(m[:, 2] - ground, 95), 1e-3), 0, 1)
    c = np.array(COL[q["key"]]) / 255.0
    dark = c * 0.55
    q["map"], q["map_col"] = m, (1 - hz)[:, None] * dark + hz[:, None] * np.minimum(c * 1.25 + 0.1, 1.0)
    N = len(q["odo"])
    span = np.ptp(q["onl"][:, :2], axis=0).max()
    q["lift"] = q.get("lift", 0.22) * span  # taller for repeated laps, so each lap gets its own level
    q["zt"] = ground + np.linspace(0, q["lift"], N)
    q["center"] = np.r_[(q["onl"][:, :2].min(0) + q["onl"][:, :2].max(0)) / 2, ground]
    # pick the azimuth that fills the panel best, then fit map + lifted trajectories
    probe = np.concatenate([m[:: max(1, len(m) // 15000)],
                            np.c_[q["onl"][:, :2], q["zt"]], np.c_[q["odo"][:, :2], q["zt"]]])
    best = None
    for az in range(0, 180, 10):
        r = raw(q, probe, az)
        lo, hi = np.percentile(r, 1, axis=0), np.percentile(r, 99, axis=0)
        sc = min((QW - 12) * SS / (hi[0] - lo[0]), (QH - 40) * SS / (hi[1] - lo[1]))
        if best is None or sc > best[0]:
            best = (sc, az, (lo + hi) / 2)
    q["scale"], q["az"], q["mid"] = best


def uv(q, xyz):
    r = raw(q, xyz, q["az"])
    return np.stack([QW * SS / 2 + q["scale"] * (r[:, 0] - q["mid"][0]),
                     (QH * SS) / 2 + 14 * SS + q["scale"] * (r[:, 1] - q["mid"][1])], 1)


def splat(p, col, k):
    w, h = QW * SS, QH * SS
    u, v = p[:, 0].astype(int), p[:, 1].astype(int)
    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    lin = v[ok] * w + u[ok]
    cnt = np.bincount(lin, minlength=w * h).astype(np.float32)
    acc = np.stack([np.bincount(lin, weights=col[ok, i], minlength=w * h) for i in range(3)], 1)
    rgb = acc / np.maximum(cnt, 1)[:, None]
    a = 1 - np.exp(-k * cnt)
    img = np.ones((w * h, 3)) * (1 - a[:, None]) + rgb * a[:, None]
    return (img.reshape(h, w, 3)[:, :, ::-1] * 255).astype(np.uint8).copy()


icons = {k: cv2.imread(f"{ICONS}/{k}.png", cv2.IMREAD_UNCHANGED) for k in COL}
INK = (40, 35, 31)

for f in (ONLY or range(FR + HOLD)):
    tiles = []
    for q in QUADS:
        N = len(q["odo"])
        k = ks(f, N)
        est = q["snaps"][k]  # the whole trajectory as known online at keyframe k
        sel = q["map_kf"] <= k
        kf, lp = q["map_kf"][sel], q["loc"][sel]
        Tk = est[kf]
        wp = np.einsum("nij,nj->ni", Tk[:, :3, :3], lp) + Tk[:, :3, 3]
        img = splat(uv(q, wp), q["map_col"][sel], 0.9)
        onl = est[:, :3, 3]
        bgr = COL[q["key"]][::-1]
        light = tuple(int(0.75 * c + 0.25 * 255) for c in bgr)
        on = np.c_[onl[:, :2], q["zt"][: k + 1]]
        od = np.c_[q["odo"][: k + 1, :2], q["zt"][: k + 1]]
        for a, b in q["loops"]:
            if a > k:
                continue
            pa, pb = uv(q, np.array([[*onl[a, :2], q["zt"][a]], [*onl[b, :2], q["zt"][b]]])).astype(int)
            cv2.line(img, tuple(pa), tuple(pb), bgr if k - a < 15 else light, int(1.3 * SS), cv2.LINE_AA)
        cv2.polylines(img, [uv(q, od).astype(np.int32)], False, (170, 170, 170), SS, cv2.LINE_AA)
        cv2.polylines(img, [uv(q, on).astype(np.int32)], False, bgr, int(1.6 * SS), cv2.LINE_AA)
        c = uv(q, on[-1:])[0].astype(int)
        cv2.circle(img, tuple(c), 5 * SS, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(img, tuple(c), 5 * SS, INK, 2 * SS, cv2.LINE_AA)
        if any(0 <= k - a < 8 for a, _ in q["loops"]):
            cv2.circle(img, tuple(c), 12 * SS, bgr, SS, cv2.LINE_AA)
        tile = cv2.resize(img, (QW, QH), interpolation=cv2.INTER_AREA)
        ic = icons[q["key"]]
        h, w = ic.shape[:2]
        al = ic[:, :, 3:4] / 255.0
        tile[6:6 + h, 6:6 + w] = (tile[6:6 + h, 6:6 + w] * (1 - al) + ic[:, :, :3] * al).astype(np.uint8)
        cv2.putText(tile, q["title"], (6 + w + 8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1, cv2.LINE_AA)
        tiles.append(tile)
    frame = np.concatenate([np.concatenate(tiles[:2], 1), np.concatenate(tiles[2:], 1)], 0)
    cv2.line(frame, (QW, 8), (QW, H - 8), (225, 225, 225), 1)
    cv2.line(frame, (8, QH), (W - 8, QH), (225, 225, 225), 1)
    cv2.imwrite(f"{OUT}/{f:04d}.png", frame)
print("frames", FR + HOLD)

if not ONLY:
    import shutil
    import subprocess
    if shutil.which("ffmpeg") is None:
        sys.exit(f"ffmpeg not found; the frames are in {OUT}")
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", "20", "-i", f"{OUT}/%04d.png", "-vf",
                    "split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=4",
                    ARGS.out], check=True)
    print(f"-> {ARGS.out}")
