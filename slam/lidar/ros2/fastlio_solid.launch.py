"""FAST-LIO (ROS 2 branch of hku-mars/FAST_LIO) + SOLiD, ROS 2. Tested setup: Kimera-Multi sparkal1.

    ros2 launch slam/lidar/ros2/fastlio_solid.launch.py save_dir:=/out
    ros2 bag play <sparkal1 ROS 2 bag> --clock

FAST-LIO's /Odometry is the IMU pose; SOLiD takes the raw LiDAR scan and the LiDAR-in-IMU
extrinsic from config/kimera_sparkal1.yaml, and closes loops online (loop_closing.enable).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("fastlio_config", default_value="fast_lio_kimera_sparkal1.yaml"),
        DeclareLaunchArgument("solid_config", default_value=os.path.join(HERE, "..", "config", "kimera_sparkal1.yaml")),
        DeclareLaunchArgument("cloud_topic", default_value="/sparkal1/lidar_points"),
        DeclareLaunchArgument("save_dir", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("fast_lio"), "launch", "mapping.launch.py")),
            launch_arguments={"config_path": HERE,
                              "config_file": LaunchConfiguration("fastlio_config"),
                              "rviz": "false", "use_sim_time": "true"}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("solid_ros"), "launch", "solid.launch.py")),
            launch_arguments={"config": LaunchConfiguration("solid_config"),
                              "cloud_topic": LaunchConfiguration("cloud_topic"), "odom_topic": "/Odometry",
                              "save_dir": LaunchConfiguration("save_dir"),
                              "rviz": "false", "use_sim_time": "true"}.items()),
    ])
