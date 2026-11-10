# README GIFs

The scripts that made the GIFs of the README, from SOLiD sessions (`save_dir` of the ROS nodes or `--out` of the macOS tool).

| Script | GIF | Input |
| --- | --- | --- |
| `quad3d.py` | `fig/solid_demo_v2.gif`: four platforms, online loop closing | one session per platform with `closed/` (`solid_close_loops`): `--data <root>` or `--panel lidar=<session> ...` |
| `ms.py` | `fig/solid_multi_session_v2.gif`: multi-session alignment | LiDAR and radar sessions aligned by `solid_align_sessions`: `--lidar <dir> --radar <dir>` |

```bash
pip install opencv-python scipy gtsam      # and ffmpeg
python fig/gen/quad3d.py solid_demo.gif --data /path/to/sessions
python fig/gen/ms.py solid_multi_session.gif --lidar /path/to/lidar/sessions --radar /path/to/radar/sessions
```

`icons/` are the platform icons drawn in the corners.
