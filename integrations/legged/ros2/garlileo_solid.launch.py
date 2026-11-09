"""GaRLILEO (radar-leg-inertial odometry, legged robots) + SOLiD loop candidates, ROS 2.

    ros2 launch integrations/legged/ros2/garlileo_solid.launch.py garlileo_config:=<config.yaml> save_dir:=/out
    ros2 bag play <GaRLILEO sequence> --clock

GaRLILEO stamps /Odometry relative to its first IMU message; tools/garlileo_odom_relay.py
republishes it with absolute stamps on /garlileo/odometry so it pairs with the radar scans.
The TI radar gives ~30 points per scan, so garlileo.yaml merges 50 scans per keyframe.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    garlileo_share = get_package_share_directory("garlileo")
    return LaunchDescription([
        DeclareLaunchArgument("garlileo_config",
                              default_value=os.path.join(garlileo_share, "dataset", "GaRLILEO", "config.yaml")),
        DeclareLaunchArgument("solid_config", default_value=os.path.join(HERE, "..", "config", "garlileo.yaml")),
        DeclareLaunchArgument("save_dir", default_value=""),
        Node(package="spot_msgs", executable="spot_msgs_node", name="spot_msgs_node", output="log"),
        Node(package="garlileo", executable="garlileo_node_exe", name="garlileo_node", output="log",
             parameters=[{"config_path": LaunchConfiguration("garlileo_config"), "use_sim_time": True}]),
        ExecuteProcess(cmd=["python3", os.path.join(HERE, "..", "tools", "garlileo_odom_relay.py")],
                       output="log"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("solid_ros"), "launch", "solid.launch.py")),
            launch_arguments={"config": LaunchConfiguration("solid_config"),
                              "cloud_topic": "/ti_mmwave/radar_scan_pcl_0",
                              "odom_topic": "/garlileo/odometry",
                              "save_dir": LaunchConfiguration("save_dir"),
                              "rviz": "false", "use_sim_time": "true"}.items()),
    ])
