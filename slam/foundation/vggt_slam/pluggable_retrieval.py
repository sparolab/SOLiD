"""Pluggable place-recognition retrieval for VGGT-SLAM loop closure.

VGGT-SLAM detects loops by nearest-neighbour of per-frame submap descriptors
(`ImageRetrieval.get_all_submap_embeddings` -> `map.retrieve_best_score_frame`,
which compares by L2 distance of L2-normalized vectors). `find_loop_closures` is
descriptor-agnostic, so a new detector only needs to override
`get_all_submap_embeddings(submap) -> (S, D) L2-normalized tensor`.

Backends (select with build_retrieval / --retrieval):
    salad   : DINOv2-SALAD on submap frames        (VGGT-SLAM default)
    netvlad : NetVLAD (hloc GlobalDesc) on frames
    solid   : SOLiD rsolid on the submap's VGGT-predicted per-frame point clouds
              (geometric; needs submap.pointclouds set BEFORE this is called ->
               solver computes world_points first for the solid backend)

All vectors are L2-normalized so map.py's `norm(embedding - query)` ranking is a
monotone function of cosine similarity, identical machinery for every backend.
"""
import os
import numpy as np
import torch

from vggt_slam.loop_closure import ImageRetrieval

device = "cuda" if torch.cuda.is_available() else "cpu"


def _l2n(x, eps=1e-12):
    return x / (x.norm(dim=-1, keepdim=True) + eps)


class SaladRetrieval(ImageRetrieval):
    """Unmodified VGGT-SLAM default (kept for a uniform factory)."""
    needs_points = False


class NetvladRetrieval(ImageRetrieval):
    """NetVLAD global descriptor (hloc GlobalDesc), image-based like SALAD."""
    needs_points = False

    def __init__(self, input_size=224):
        # deliberately skip ImageRetrieval.__init__ (loads SALAD); build NetVLAD
        from hloc import extractors
        from hloc.utils.base_model import dynamic_load
        conf = {"model": {"name": "netvlad"}, "preprocessing": {"resize_max": 1024}}
        Model = dynamic_load(extractors, conf["model"]["name"])
        self.model = Model(conf["model"]).eval().to(device)
        self.input_size = input_size
        print("[NetvladRetrieval] hloc NetVLAD (4096-d)")

    @torch.no_grad()
    def get_batch_descriptors(self, imgs):
        # imgs: (B,3,H,W) float in [0,1] (VGGT frames are already normalized to model input);
        # NetVLAD wants [0,1] RGB. VGGT frames come in [0,1] before ImageNet norm.
        out = []
        for img in imgs:
            t = img[None].to(device).float()
            if t.max() > 1.5:
                t = t / 255.0
            desc = self.model({"image": t})["global_descriptor"]
            out.append(desc.squeeze(0))
        return _l2n(torch.stack(out))

    def get_all_submap_embeddings(self, submap):
        return self.get_batch_descriptors(submap.get_all_frames())


class SolidRetrieval(ImageRetrieval):
    """SOLiD rsolid on VGGT's per-frame predicted point clouds (geometric)."""
    needs_points = True

    def __init__(self, profile="gs_rgbd", conf_subsample=6, mean_remove=None):
        import solid
        self.solid = solid
        self.cfg = solid.load_config(profile)
        # VGGT-TUNED PROFILE. LoopSplat's gs_rgbd uses max_range=10 (metres), but VGGT
        # predicts (roughly metric) depth with range ~0.5-2, so with max_range=10 only
        # the first ~6 of 40 range bins are ever populated -> the range histogram has
        # almost no resolving power and different places alias. Matching max_range to
        # VGGT's actual scale spreads points across all bins, and that fine RANGE
        # resolution is exactly what a translation-variant descriptor needs to tell
        # places apart. Sweep on office_loop: mr=8 num_range=60 num_height=20 fov±10
        # makes the true end->start loop the GLOBAL nearest with a 110% margin and NO
        # temporal gap (204->0 d=0.074 vs 2nd-best 0.156). Override via env if needed.
        self.cfg.max_range = float(os.environ.get("SOLID_MAX_RANGE", "8.0"))
        self.cfg.num_range = int(os.environ.get("SOLID_NUM_RANGE", "60"))
        self.cfg.num_height = int(os.environ.get("SOLID_NUM_HEIGHT", "20"))
        _fov = float(os.environ.get("SOLID_FOV", "10"))
        self.cfg.fov_up = _fov
        self.cfg.fov_down = -_fov
        self.cfg.min_range = float(os.environ.get("SOLID_MIN_RANGE", "0.05"))
        self.cfg.voxel_size = float(os.environ.get("SOLID_VOXEL", "0.03"))
        self.ext = solid.Extractor(self.cfg)
        self.sub = int(os.environ.get("SOLID_SUB", str(conf_subsample)))   # per-frame point subsample (1 = none)
        # SCALE: VGGT predicts (roughly) metric depth, so its per-submap point maps
        # are already at a CONSISTENT scale across submaps (median range 0.5-0.7 on
        # office_loop). SOLiD's rsolid is a *range* histogram and is NOT
        # translation/scale-invariant -> that consistent metric range is exactly the
        # signal that tells apart two different places. An earlier per-cloud
        # scale-normalization DESTROYED it (every cloud rescaled to the same size ->
        # different places looked identical, 204->0 dropped to rank 4). Keep raw
        # metric range by default; SOLID_SCALE_NORM=1 restores the old behavior.
        self.scale_norm = os.environ.get("SOLID_SCALE_NORM", "0") == "1"
        # mean-removal HELPS on real metric depth (LoopSplat) but HURTS on VGGT here
        # (raw metric rsolid already discriminates: 204->0 is rank 1 at 0.027 without
        # it, rank 3 with it). Default off for VGGT; override with SOLID_MEAN_REMOVE.
        if mean_remove is None:
            mean_remove = os.environ.get("SOLID_MEAN_REMOVE", "0") == "1"
        self.mean_remove = mean_remove
        # SOLID_NO_FLOOR=1 skips floor-plane gravity alignment. On KITTI the camera is
        # already ~level (road normal ~ camera +y), so floor-align is near-identity and
        # can be skipped (also avoids the segment_plane wall-pick / RANSAC-random issue).
        self.use_floor = os.environ.get("SOLID_NO_FLOOR", "0") != "1"
        # VGGT point maps are in OpenCV camera coordinates (x right, y down,
        # z forward), while SOLiD assumes a z-up scan (x/y ground plane).  Turning
        # floor RANSAC off must therefore skip only plane estimation, not this fixed
        # axis conversion.  The old code fed camera y into azimuth and camera z into
        # height whenever SOLID_NO_FLOOR=1, which is especially damaging on KITTI.
        self.camera_z_up = os.environ.get("SOLID_CAMERA_Z_UP", "1") == "1"
        # weight rsolid bins by image brightness (appearance) instead of point count
        self.intensity = os.environ.get("SOLID_INTENSITY", "0") == "1"
        self.channel_norm = os.environ.get("SOLID_CHANNEL_NORM", "1") == "1"
        if self.intensity:
            self.cfg.use_weight = True
            self.ext = solid.Extractor(self.cfg)
        # Azimuth-preserving colour context. Standard rsolid marginalizes azimuth,
        # so similarly shaped roads become indistinguishable.  We append an RGB
        # range×azimuth histogram and compare it over circular angle shifts.  This
        # keeps yaw invariance while retaining where appearance occurs around the
        # sensor. Set SOLID_AZIMUTH=1 to enable it.
        self.azimuth = os.environ.get("SOLID_AZIMUTH", "0") == "1"
        self.num_angle = int(os.environ.get("SOLID_NUM_ANGLE", str(self.cfg.num_angle)))
        self.az_weight = float(os.environ.get("SOLID_AZ_WEIGHT", "1.0"))
        self._cur_colors = []
        self._sum = None
        self._cnt = 0
        self._cur_clouds = []                        # per-submap gravity-aligned clouds
        self._clouds_by_id = {}                      # submap_id -> [per-frame RAW camera clouds] for ICP
        self._dump = {}                              # submap_id -> [clouds] (SOLID_DUMP)
        # GEOMETRIC VERIFICATION. office is so self-similar that a mid submap can be
        # descriptor-closer to the start than the true end->start loop (aliases beat
        # the real loop, and which one wins flips every VGGT inference). Appearance
        # (descriptor / image_match_ratio) therefore cannot isolate the true loop.
        # So after SOLiD proposes a candidate, actually REGISTER the two frame clouds
        # (floor-aligned, multi-init ICP) and keep it only if the geometry aligns
        # (fitness >= SOLID_ICP_MIN). SOLID_ICP_MIN=0 -> log fitness only (calibrate).
        self.icp_min = float(os.environ.get("SOLID_ICP_MIN", "0.0"))
        # temporal-gap filter: SOLiD's range histogram changes slowly, so the
        # nearest submap is usually a RECENT one (conveyor matching, not a loop).
        # Require the matched submap to be >= min_submap_gap (frame-id units) before
        # the query so only genuine large-gap revisits count as loop closures.
        self.min_submap_gap = int(os.environ.get("SOLID_MIN_SUBMAP_GAP", "80"))
        # Optional odometry-proximity gate. Descriptor aliases are common indoors;
        # only compare frame pairs whose current odometry estimates are spatially
        # close. Zero disables the gate. The solver attaches tentative world-frame
        # camera centers before retrieval.
        self.odom_radius = float(os.environ.get("SOLID_ODOM_RADIUS", "0"))
        print(f"[SolidRetrieval] SOLiD rsolid on VGGT point maps "
              f"(profile={profile} mean_remove={mean_remove} min_gap={self.min_submap_gap} "
              f"camera_z_up={self.camera_z_up} azimuth={self.azimuth})")

    def find_loop_closures(self, map, submap, max_similarity_thres=0.80, max_loop_closures=0):
        from vggt_slam.loop_closure import LoopMatch, LoopMatchQueue
        if max_loop_closures <= 0:
            return []
        q_id = submap.get_id()
        q_vecs = submap.get_all_retrieval_vectors()
        mq = LoopMatchQueue(max_size=max_loop_closures)
        for qi in range(len(q_vecs)):
            qv = q_vecs[qi]
            best_d, best_sid, best_fi = 1e9, None, None
            for sid, sm in map.submaps.items():
                if sm.get_lc_status() or sid == q_id:
                    continue
                if q_id - sid < self.min_submap_gap:      # skip recent (adjacency) submaps
                    continue
                embs = sm.get_all_retrieval_vectors()
                if embs is None:
                    continue
                for fi in range(len(embs)):
                    if self.odom_radius > 0:
                        qc = getattr(submap, "solid_world_centers", None)
                        sc = getattr(sm, "solid_world_centers", None)
                        if qc is None or sc is None:
                            continue
                        spatial_d = float(np.linalg.norm(qc[qi] - sc[fi]))
                        if spatial_d > self.odom_radius:
                            continue
                    d = self._descriptor_distance(embs[fi], qv)
                    if d < best_d:
                        best_d, best_sid, best_fi = d, sid, fi
            if best_sid is not None and best_d < max_similarity_thres:
                if self.odom_radius > 0:
                    spatial_d = float(np.linalg.norm(
                        submap.solid_world_centers[qi] -
                        map.submaps[best_sid].solid_world_centers[best_fi]))
                    print(f"[SOLID-ODOM] submap {q_id}->{best_sid} "
                          f"spatial_d={spatial_d:.3f} radius={self.odom_radius:.3f}")
                fit = self._icp_fitness_by_ids(q_id, qi, best_sid, best_fi)
                keep = fit >= self.icp_min
                print(f"[SOLID-ICP] submap {q_id}->{best_sid} desc_d={best_d:.3f} "
                      f"icp_fitness={fit:.3f}" + ("" if keep else "  REJECT(alias)"))
                if keep:
                    mq.add(LoopMatch(best_d, q_id, qi, best_sid, best_fi))
        return mq.get_matches()

    def _icp_fitness_by_ids(self, q_id, qi, s_id, si):
        """ICP-align the two matched frame clouds; return fitness (inlier fraction).
        Returns 1.0 (don't block) if clouds are unavailable."""
        try:
            cq = self._clouds_by_id.get(int(q_id))
            cs = self._clouds_by_id.get(int(s_id))
            if not cq or not cs or qi >= len(cq) or si >= len(cs):
                return 1.0
            return self._icp_fitness(cq[qi], cs[si])
        except Exception as e:
            print("[SOLID-ICP] error:", repr(e))
            return 1.0

    def _icp_fitness(self, pcq, pcd):
        import open3d as o3d
        def prep(pc):
            pc = self._floor_gravity(np.asarray(pc, np.float32))     # remove pitch/roll
            o = o3d.geometry.PointCloud()
            o.points = o3d.utility.Vector3dVector(pc.astype(np.float64))
            o = o.voxel_down_sample(0.04)
            o.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
            return o
        src, tgt = prep(pcq), prep(pcd)
        if len(src.points) < 30 or len(tgt.points) < 30:
            return 0.0
        dist = 0.08
        best = 0.0
        for deg in (0, 90, 180, 270):                               # multi-yaw init (SOLiD is azimuth-invariant)
            th = np.deg2rad(deg); c, s = np.cos(th), np.sin(th)
            T = np.eye(4); T[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
            r = o3d.pipelines.registration.registration_icp(
                src, tgt, dist, T,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
            best = max(best, r.fitness)
        return best

    @staticmethod
    def _camera_to_z_up(pc):
        """OpenCV camera (right, down, forward) -> right-handed z-up scan."""
        return np.column_stack((pc[:, 0], pc[:, 2], -pc[:, 1])).astype(np.float32)

    @staticmethod
    def _floor_gravity(pc):
        """Gravity-align by the FLOOR plane (same as the winning LoopSplat SOLiD):
        rotate so the dominant floor-plane normal -> +Z. Robust to camera tilt
        (unlike a fixed camera-axis swap). Falls back to a min-variance PCA axis."""
        try:
            import open3d as o3d
            o = o3d.geometry.PointCloud()
            o.points = o3d.utility.Vector3dVector(pc.astype(np.float64))
            model, _ = o.segment_plane(0.05 * (np.linalg.norm(pc, axis=1).mean() + 1e-6),
                                       3, 100)
            n = np.array(model[:3])
        except Exception:
            _, _, Vt = np.linalg.svd(pc - pc.mean(0)); n = Vt[2]
        n = n / (np.linalg.norm(n) + 1e-9)
        if np.dot(n, np.array([0, -1.0, 0])) < 0:      # point "up" (camera up ~ -y)
            n = -n
        z = np.array([0, 0, 1.0]); v = np.cross(n, z); s = np.linalg.norm(v); c = float(n @ z)
        if s < 1e-8:
            return pc.astype(np.float32)
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))
        return ((R @ pc.T).T).astype(np.float32)

    def _azimuth_color_descriptor(self, pc, col):
        """RGB range×azimuth context with per-channel power/L2 normalization."""
        nr, na = int(self.cfg.num_range), self.num_angle
        rho = np.linalg.norm(pc[:, :2], axis=1)
        theta = np.mod(np.arctan2(pc[:, 1], pc[:, 0]), 2 * np.pi)
        elev = np.degrees(np.arctan2(pc[:, 2], rho + 1e-12))
        valid = ((rho >= self.cfg.min_range) & (rho < self.cfg.max_range) &
                 (elev >= self.cfg.fov_down) & (elev <= self.cfg.fov_up))
        ir = np.floor(rho[valid] * nr / self.cfg.max_range).astype(np.int64)
        ia = np.floor(theta[valid] * na / (2 * np.pi)).astype(np.int64)
        hist = np.zeros((3, nr, na), np.float32)
        for ch in range(3):
            np.add.at(hist[ch], (ir, ia), col[valid, ch])
            # Root normalization reduces domination by dense road/sky bins.
            hist[ch] = np.sqrt(hist[ch])
            hist[ch] /= np.linalg.norm(hist[ch]) + 1e-12
        return hist.reshape(-1)

    def _descriptor_distance(self, a, b):
        """Fused rsolid/context distance, minimizing context over circular yaw."""
        if not self.azimuth:
            return float((a - b).norm())
        base_dim = (3 if self.intensity else 1) * int(self.cfg.num_range)
        # The raw rsolid magnitude is orders of magnitude larger than the
        # normalized context histogram. Normalize the two modalities separately;
        # otherwise whole-vector L2 normalization effectively erases context.
        ab = a[:base_dim] / (a[:base_dim].norm() + 1e-12)
        bb = b[:base_dim] / (b[:base_dim].norm() + 1e-12)
        da = float((ab - bb).norm())
        shape = (3, int(self.cfg.num_range), self.num_angle)
        aa = a[base_dim:].reshape(shape)
        bb = b[base_dim:].reshape(shape)
        aa = aa / (aa.norm() + 1e-12)
        bb = bb / (bb.norm() + 1e-12)
        # Circular correlation gives the minimum L2 distance over every yaw shift:
        # ||a-roll(b)||² = ||a||²+||b||²-2<a,roll(b)>. FFT avoids a Python loop over
        # all angle bins and materially reduces online candidate-search latency.
        fa = torch.fft.rfft(aa, dim=2)
        fb = torch.fft.rfft(bb, dim=2)
        corr = torch.fft.irfft(torch.conj(fa) * fb, n=self.num_angle, dim=2).sum((0, 1))
        dz2 = (aa.square().sum() + bb.square().sum() - 2 * corr.max()).clamp_min(0)
        dz = float(torch.sqrt(dz2))
        return (da + self.az_weight * dz) / (1.0 + self.az_weight)

    def _descriptor_from_points(self, pts_hw3, conf=None, conf_thr=None, col_hw3=None):
        pc = pts_hw3.reshape(-1, 3)
        col = col_hw3.reshape(-1, 3) if col_hw3 is not None else None
        if conf is not None:
            m = conf.reshape(-1) >= (conf_thr if conf_thr is not None else 0)
            pc = pc[m]
            if col is not None: col = col[m]
        pc = pc[::self.sub]
        if col is not None: col = col[::self.sub]
        good = np.isfinite(pc).all(1)
        pc = pc[good].astype(np.float32)
        if col is not None: col = col[good].astype(np.float32)
        if len(pc) < 50:
            self._cur_colors.append(None)
            base_dim = (3 if self.intensity else 1) * int(self.cfg.num_range)
            context_dim = 3 * int(self.cfg.num_range) * self.num_angle if self.azimuth else 0
            return np.zeros(base_dim + context_dim, np.float32)
        self._cur_clouds.append(pc.copy())                      # RAW camera cloud (pre-transform) for SOLID_DUMP
        self._cur_colors.append(col.copy() if col is not None else None)
        if self.scale_norm:
            rng = np.linalg.norm(pc, axis=1)
            r90 = np.percentile(rng, 90)
            if r90 > 1e-6:
                pc = pc * (0.8 * self.cfg.max_range / r90)
        # gravity-align by the floor plane -> z-up (rotation preserves point order,
        # so color stays aligned). Skippable on KITTI (already level).
        if self.use_floor:
            pc = self._floor_gravity(pc)
        elif self.camera_z_up:
            pc = self._camera_to_z_up(pc)
        # COLOUR SOLiD (SOLID_INTENSITY=1): weight the rsolid bins by each image colour
        # channel and concatenate -> R-SOLiD + G-SOLiD + B-SOLiD (3*num_range). Encodes
        # appearance, not just geometry, so places that are geometrically aliased
        # (roads/rooms identical in 3D) separate by colour. num_range stays fixed; the
        # extra discrimination comes from the 3 channels, not from inflating bins.
        if self.intensity and col is not None and len(col) == len(pc):
            chans = [np.asarray(self.ext.extract(pc, list(col[:, ch].astype(np.float32))).rsolid,
                                np.float32) for ch in range(3)]
            if self.channel_norm:
                chans = [x / (np.linalg.norm(x) + 1e-12) for x in chans]
            base = np.concatenate(chans)                        # R(nr)+G(nr)+B(nr)
        else:
            base = np.asarray(self.ext.extract(pc).rsolid, np.float32)
        if self.azimuth:
            if col is None:
                # Geometry-only context uses occupancy replicated over channels so
                # descriptor shape stays fixed if colour is unexpectedly absent.
                col = np.ones((len(pc), 3), np.float32)
            context = self._azimuth_color_descriptor(pc, col)
            return np.concatenate((base, context))
        return base

    def _rsolid_from_agg(self, cloud):
        """One rsolid from the whole submap-aggregate cloud (centered, submap frame)."""
        pc = cloud[np.isfinite(cloud).all(1)][::self.sub].astype(np.float32)
        if len(pc) < 50:
            return np.zeros(self.cfg.num_range, np.float32), pc
        rng = np.linalg.norm(pc, axis=1); r90 = np.percentile(rng, 90)
        if r90 > 1e-6:
            pc = pc * (0.8 * self.cfg.max_range / r90)          # canonical scale
        pc = self._floor_gravity(pc)                            # z-up (reliable on dense agg)
        return np.asarray(self.ext.extract(pc).rsolid, np.float32), pc

    def get_all_submap_embeddings(self, submap):
        raw_dump = None
        agg = getattr(submap, "solid_cloud", None)
        if agg is not None:                                     # submap-aggregate path
            r, used = self._rsolid_from_agg(agg)
            raws = [r]; raw_dump = agg
        else:                                                   # per-frame fallback
            pcs = submap.pointclouds
            conf = getattr(submap, "conf", None); thr = getattr(submap, "conf_threshold", None)
            cols = getattr(submap, "colors", None)
            self._cur_clouds = []; self._cur_colors = []
            raws = [self._descriptor_from_points(pcs[i], conf[i] if conf is not None else None, thr,
                                                 cols[i] if cols is not None else None)
                    for i in range(len(pcs))]
            self._clouds_by_id[int(submap.get_id())] = list(self._cur_clouds)   # for ICP geometric verification
        if self.mean_remove:
            for r in raws:
                self._sum = r.copy() if self._sum is None else self._sum + r
                self._cnt += 1
            mean = self._sum / self._cnt
        else:
            mean = 0.0
        vecs = [_l2n(torch.from_numpy(r - mean).float().to(device)) for r in raws]
        dpath = os.environ.get("SOLID_DUMP")                    # dump RAW clouds (+colors) for offline tuning
        if dpath:
            import pickle
            self._dump[int(submap.get_id())] = {
                "clouds": raw_dump if raw_dump is not None else list(self._cur_clouds),
                "colors": list(self._cur_colors),
            }
            with open(dpath, "wb") as f:
                pickle.dump(self._dump, f)
        return torch.stack(vecs)


def build_retrieval(kind="salad", **kw):
    kind = kind.lower()
    if kind == "salad":
        return SaladRetrieval()
    if kind == "netvlad":
        return NetvladRetrieval()
    if kind == "solid":
        return SolidRetrieval(**kw)
    raise ValueError(f"unknown retrieval backend: {kind}")
