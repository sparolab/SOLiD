"""SOLiD loop candidate node (ROS 2).

    ros2 launch solid_ros solid.launch.py config:=kitti cloud_topic:=/cloud odom_topic:=/odom
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = get_package_share_directory("solid_ros")
    config = LaunchConfiguration("config").perform(context)
    if "/" not in config:  # a name in config/
        config = os.path.join(share, "config", config + ".yaml")
    return [
        Node(
            package="solid_ros",
            executable="solid_node",
            name="solid",
            output="screen",
            parameters=[{
                "config": config,
                "save_dir": LaunchConfiguration("save_dir"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            remappings=[
                ("cloud", LaunchConfiguration("cloud_topic")),
                ("odom", LaunchConfiguration("odom_topic")),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="solid_rviz",
            arguments=["-d", os.path.join(share, "rviz", "solid.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="kitti",
                              description="name in config/ (kitti, ouster64, radar_4d) or a YAML path"),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("save_dir", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_nodes),
    ])
