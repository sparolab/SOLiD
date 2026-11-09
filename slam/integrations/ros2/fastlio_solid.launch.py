"""sparo-fast-lio (ROS 2) + SOLiD loop candidates on Kimera-Multi.

    ros2 launch slam/integrations/ros2/fastlio_solid.launch.py robot:=sparkal1 save_dir:=/out

visualization_frame=lidar makes /<robot>/odometry the LiDAR pose and
/<robot>/cloud_registered_lidar the deskewed scan in the LiDAR frame (z up), same stamp,
so SOLiD runs with cloud_in_world: false.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def _nodes(context):
    robot = LaunchConfiguration("robot").perform(context)
    fl_cfg = os.path.join(get_package_share_directory("sparo_fast_lio"),
                          "config", "kimera_multi", robot + ".yaml")
    fast_lio = Node(
        package="sparo_fast_lio", executable="sparo_lio_mapping", name="lio_mapping",
        namespace=robot, output="log",
        remappings=[("lidar", "lidar_points"), ("imu", "forward/imu")],
        parameters=[fl_cfg, {
            "use_sim_time": True,
            "common.lidar_frame": robot + "/velodyne_link",
            "common.imu_frame": robot + "/forward_imu_optical_frame",
            "common.map_frame": robot + "/odom",
            "common.base_frame": robot + "/base_link",
            "common.visualization_frame": "lidar",
            "gravity_alignment.enable_gravity_alignment": False,
        }])
    solid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("solid_ros"), "launch", "solid.launch.py")),
        launch_arguments={
            "config": LaunchConfiguration("solid_config").perform(context),
            "cloud_topic": f"/{robot}/cloud_registered_lidar",
            "odom_topic": f"/{robot}/odometry",
            "save_dir": LaunchConfiguration("save_dir").perform(context),
            "rviz": "false", "use_sim_time": "true"}.items())
    # sparo-fast-lio looks up base_frame <- lidar_frame at start-up and never starts its worker
    # without it; the Kimera-Multi bags carry no TF, so publish identity (base outputs unused here).
    base_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_to_lidar",
        output="log",
        arguments=["--frame-id", robot + "/base_link", "--child-frame-id", robot + "/velodyne_link"])
    return [base_tf, fast_lio, solid]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="sparkal1"),
        DeclareLaunchArgument("solid_config",
                              default_value="vlp16"),
        DeclareLaunchArgument("save_dir", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
