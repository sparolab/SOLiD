"""FAST-LIO (ROS 2 branch) on a Livox Mid-360 + SOLiD loop candidates (e.g. KILVO, Unitree G1).

    ros2 launch slam/humanoid/ros2/fastlio_mid360_solid.launch.py save_dir:=/out
    ros2 bag play <bag with /livox/lidar (livox_ros_driver2/CustomMsg) and /livox/imu> --clock
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("solid_config", default_value="livox_mid360"),
        DeclareLaunchArgument("save_dir", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("fast_lio"), "launch", "mapping.launch.py")),
            launch_arguments={"config_file": "mid360.yaml", "rviz": "false",
                              "use_sim_time": "true"}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("solid_ros"), "launch", "solid.launch.py")),
            launch_arguments={"config": LaunchConfiguration("solid_config"),
                              "cloud_topic": "/cloud_registered_body", "odom_topic": "/Odometry",
                              "save_dir": LaunchConfiguration("save_dir"),
                              "rviz": "false", "use_sim_time": "true"}.items()),
    ])
