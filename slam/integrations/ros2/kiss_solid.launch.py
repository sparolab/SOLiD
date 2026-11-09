"""KISS-ICP (ROS 2, v1.3) + SOLiD loop candidates.

    ros2 launch slam/integrations/ros2/kiss_solid.launch.py topic:=/sparkal1/lidar_points save_dir:=/out

/kiss/frame is the deskewed, range-cropped scan in the input cloud's frame (needs
publish_debug_clouds); /kiss/odometry is the pose of that frame with the same stamp.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    kiss = Node(
        package="kiss_icp", executable="kiss_icp_node", name="kiss_icp_node", output="log",
        remappings=[("pointcloud_topic", LaunchConfiguration("topic"))],
        parameters=[{
            "use_sim_time": True,
            "base_frame": "", "lidar_odom_frame": "odom_lidar",
            "publish_odom_tf": False, "invert_odom_tf": False,
            "publish_debug_clouds": True,
            "data.deskew": False, "data.max_range": 100.0, "data.min_range": 1.6,
            "mapping.max_points_per_voxel": 20,
            "adaptive_threshold.initial_threshold": 2.0,
            "adaptive_threshold.min_motion_th": 0.1,
        }])
    solid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("solid_ros"), "launch", "solid.launch.py")),
        launch_arguments={
            "config": LaunchConfiguration("solid_config"),
            "cloud_topic": "/kiss/frame", "odom_topic": "/kiss/odometry",
            "save_dir": LaunchConfiguration("save_dir"),
            "rviz": "false", "use_sim_time": "true"}.items())
    return LaunchDescription([
        DeclareLaunchArgument("topic", default_value="/sparkal1/lidar_points"),
        DeclareLaunchArgument("solid_config",
                              default_value="vlp16"),
        DeclareLaunchArgument("save_dir", default_value=""),
        kiss, solid,
    ])
