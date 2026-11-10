"""Leg-KILO (kinematic-inertial-LiDAR odometry, legged robots) + SOLiD loop closing, ROS 2.

    ros2 launch slam/legged/ros2/legkilo_solid.launch.py save_dir:=/out
    ros2 bag play <Leg-KILO ROS 2 bag> --clock
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    legkilo_share = get_package_share_directory("legkilo")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),  # SOLiD view: loop-closed map, trajectory, loops
        DeclareLaunchArgument("legkilo_config",
                              default_value=os.path.join(legkilo_share, "config", "legkilo_go1_velodyne.yaml")),
        DeclareLaunchArgument("solid_config", default_value=os.path.join(HERE, "..", "config", "legkilo.yaml")),
        DeclareLaunchArgument("save_dir", default_value=""),
        Node(package="legkilo", executable="legkilo_node", name="legkilo_node", output="log",
             arguments=["--config_file", LaunchConfiguration("legkilo_config")],
             parameters=[{"use_sim_time": True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("solid_ros"), "launch", "solid.launch.py")),
            launch_arguments={"config": LaunchConfiguration("solid_config"),
                              "cloud_topic": "/cloud_registered", "odom_topic": "/Odometry",
                              "save_dir": LaunchConfiguration("save_dir"),
                              "rviz": LaunchConfiguration("rviz"), "use_sim_time": "true"}.items()),
    ])
