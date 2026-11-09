"""SOLiD multi-session alignment (ROS 2).

    ros2 launch solid_multi_session align.launch.py sessions:="/data/00a /data/00b" save_dir:=/data/aligned
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("solid_multi_session")
    return LaunchDescription([
        DeclareLaunchArgument("sessions", description="session folders, space separated; first = central"),
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "default.yaml")),
        DeclareLaunchArgument("save_dir", default_value=""),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="solid_multi_session", executable="align_node", name="solid_multi_session",
             output="screen",
             parameters=[{"sessions": LaunchConfiguration("sessions"),
                          "config": LaunchConfiguration("config"),
                          "save_dir": LaunchConfiguration("save_dir")}]),
        Node(package="rviz2", executable="rviz2", name="solid_ms_rviz",
             arguments=["-d", os.path.join(share, "rviz", "align.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
