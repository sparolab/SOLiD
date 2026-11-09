"""GeoRIO (4D radar-inertial odometry) + SOLiD loop candidates, ROS 2.

    ros2 launch integrations/radar/ros2/georio_solid.launch.py georio_config:=<GeoRIO yaml> save_dir:=/out
    ros2 bag play <SNAIL-Radar ROS 2 bag>

GeoRIO publishes /current_scan (the deskewed radar scan in the odometry frame) and
/odometry_georio; SOLiD moves the scan back to the sensor frame (cloud_in_world: true).
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
    georio_share = get_package_share_directory("geo_rio")
    return LaunchDescription([
        DeclareLaunchArgument("georio_config",
                              default_value=os.path.join(georio_share, "config", "config_snail.yaml")),
        DeclareLaunchArgument("solid_config", default_value=os.path.join(HERE, "..", "config", "georio.yaml")),
        DeclareLaunchArgument("save_dir", default_value=""),
        Node(package="geo_rio", executable="GEORIO", name="GEORIO", output="log",
             parameters=[LaunchConfiguration("georio_config")]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("solid_ros"), "launch", "solid.launch.py")),
            launch_arguments={"config": LaunchConfiguration("solid_config"),
                              "cloud_topic": "/current_scan", "odom_topic": "/odometry_georio",
                              "save_dir": LaunchConfiguration("save_dir"),
                              "rviz": "false", "use_sim_time": "false"}.items()),
    ])
