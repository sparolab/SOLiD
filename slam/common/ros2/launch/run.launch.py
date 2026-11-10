"""One entry point: an odometry + SOLiD (loop candidates, online loop closing), ROS 2.

    ros2 launch solid_ros run.launch.py odometry:=georio save_dir:=/out

odometry: fast_lio | kiss_icp | georio | garlileo | legkilo | fast_lio_mid360
Build the odometry first (thirdparty/README.md, or the images in docker/slam).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ODOMETRY = {
    "fast_lio": "lidar/ros2/fastlio_solid.launch.py",
    "kiss_icp": "lidar/ros2/kiss_solid.launch.py",
    "georio": "radar/ros2/georio_solid.launch.py",
    "garlileo": "legged/ros2/garlileo_solid.launch.py",
    "legkilo": "legged/ros2/legkilo_solid.launch.py",
    "fast_lio_mid360": "humanoid/ros2/fastlio_mid360_solid.launch.py",
}


def _include(context):
    name = LaunchConfiguration("odometry").perform(context)
    if name not in ODOMETRY:
        raise RuntimeError(f"odometry '{name}' is not one of {sorted(ODOMETRY)}")
    path = os.path.join(get_package_share_directory("solid_ros"), "platforms", ODOMETRY[name])
    args = {"save_dir": LaunchConfiguration("save_dir").perform(context)}
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=args.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("odometry", default_value="fast_lio", description=" | ".join(ODOMETRY)),
        DeclareLaunchArgument("save_dir", default_value=""),
        OpaqueFunction(function=_include),
    ])
