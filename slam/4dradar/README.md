# slam/4dradar — SOLiD × GaRLILEO (4D radar-inertial, ROS2)

**Target:** [GaRLILEO](https://github.com/ChiyunNoh/GaRLILEO) — legged-robot
odometry fusing **4D radar** Doppler + IMU + leg kinematics with an S2 gravity
factor, **ROS2 (Humble, native)**. Companion pkg: SPOT_ego_Velocity.
(This is the workspace's 4D-radar representative; the earlier separate go-rio
target was dropped as redundant.)

**Role:** decoupled loop-candidate module. Subscribes to GaRLILEO's
`(radar cloud, odometry)`, runs `solid_core`, publishes loop candidates.
Registration/PGO downstream.

**Flavor:** ROS2 ament package, links `solid_core` (`../../cpp`). Tested on
Humble; ament code is largely distro-portable (Humble↔Jazzy minor changes).

**Profile:** `radar_4d` — sparse radar, RCS/power weighting, no voxel; use
GaRLILEO's gravity-aligned frame for the descriptor's FOV binning.

Status: **planned** (needs ROS2 Humble — Docker on this 24.04 machine).
