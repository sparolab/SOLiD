"""Render side-by-side VGGT-SLAM loop before/after snapshots as an orbit GIF."""
import argparse
import os

import imageio.v2 as imageio
import numpy as np
import open3d as o3d

from vggt_slam.o3d_loop_vis import camera_wireframe


def cloud(snapshot):
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(snapshot["points"])
    p.colors = o3d.utility.Vector3dVector(snapshot["colors"])
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--before", required=True)
    ap.add_argument("--after", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=72)
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--width", type=int, default=1600)
    ap.add_argument("--height", type=int, default=900)
    ap.add_argument("--point-size", type=float, default=2.0)
    ap.add_argument("--elev", type=float, default=24.0)
    ap.add_argument("--map", help="Optional common dense PLY/PCD map; overlays both trajectories")
    ap.add_argument("--map-voxel", type=float, default=0.01)
    ap.add_argument("--cam-stride", type=int, default=2)
    ap.add_argument("--camera-to-z-up", action="store_true",
                    help="Rotate VGGT/OpenCV-like world axes (x,y,z)->(x,z,-y)")
    ap.add_argument("--orbit-path", choices=("circle", "figure8"), default="circle")
    ap.add_argument("--elev-amp", type=float, default=12.0,
                    help="Figure-eight elevation amplitude in degrees")
    ap.add_argument("--elev-cycles", type=float, default=2.0,
                    help="Number of vertical oscillations per orbit")
    args = ap.parse_args()

    before = dict(np.load(args.before)); after = dict(np.load(args.after))
    axis_T = np.eye(4)
    if args.camera_to_z_up:
        axis_T[:3, :3] = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        for snap in (before, after):
            snap["points"] = (axis_T[:3, :3] @ snap["points"].T).T
            snap["poses"] = np.stack([axis_T @ pose for pose in snap["poses"]])
    extent = max(np.ptp(before["points"], axis=0).max(), np.ptp(after["points"], axis=0).max())
    scale = max(extent * 0.012, 1e-3)
    w0 = camera_wireframe(before["poses"], [0.9, 0.05, 0.05], scale, args.cam_stride)
    w1 = camera_wireframe(after["poses"], [0.05, 0.75, 0.05], scale, args.cam_stride)

    renderer = o3d.visualization.rendering.OffscreenRenderer(args.width, args.height)
    renderer.scene.set_background([1, 1, 1, 1])
    pm = o3d.visualization.rendering.MaterialRecord()
    pm.shader = "defaultUnlit"; pm.point_size = args.point_size
    lm = o3d.visualization.rendering.MaterialRecord()
    lm.shader = "unlitLine"; lm.line_width = 2.5
    if args.map:
        common_map = o3d.io.read_point_cloud(args.map)
        if args.map_voxel > 0:
            common_map = common_map.voxel_down_sample(args.map_voxel)
        if args.camera_to_z_up:
            common_map.transform(axis_T)
        renderer.scene.add_geometry("common_map", common_map, pm)
        map_points = np.asarray(common_map.points)
        print(f"common map: {len(map_points)} points", flush=True)
    else:
        shift = np.array([extent * 1.25, 0, 0])
        c0, c1 = cloud(before), cloud(after)
        c0.translate(-shift / 2); c1.translate(shift / 2)
        w0.translate(-shift / 2); w1.translate(shift / 2)
        renderer.scene.add_geometry("before_map", c0, pm)
        renderer.scene.add_geometry("after_map", c1, pm)
        map_points = np.vstack((np.asarray(c0.points), np.asarray(c1.points)))
    renderer.scene.add_geometry("before_cameras_red", w0, lm)
    renderer.scene.add_geometry("after_cameras_green", w1, lm)

    all_points = np.vstack((map_points, np.asarray(w0.points), np.asarray(w1.points)))
    center = (all_points.min(0) + all_points.max(0)) / 2
    span = np.linalg.norm(all_points.max(0) - all_points.min(0))
    radius = span * 0.9
    elev = np.deg2rad(args.elev)
    frames = []
    for i in range(args.frames):
        az = -np.pi / 3 + 2 * np.pi * i / args.frames
        frame_elev = elev
        if args.orbit_path == "figure8":
            # Radius stays constant while elevation changes smoothly. Fractional
            # cycle counts are allowed for deliberately slow vertical motion.
            frame_elev += np.deg2rad(args.elev_amp) * np.sin(
                args.elev_cycles * (az + np.pi / 3))
        eye = center + radius * np.array([np.cos(frame_elev) * np.cos(az),
                                          np.cos(frame_elev) * np.sin(az), np.sin(frame_elev)])
        renderer.setup_camera(55.0, center, eye, [0, 0, 1])
        frames.append(np.asarray(renderer.render_to_image()))
        if (i + 1) % 12 == 0:
            print(f"rendered {i + 1}/{args.frames}", flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    imageio.mimsave(args.out, frames, fps=args.fps, loop=0)
    print("saved", args.out)


if __name__ == "__main__":
    main()
