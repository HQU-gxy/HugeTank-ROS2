from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory("hugetank_link")
    launch_dir = os.path.join(bringup_dir, "launch")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "hugetank_link.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "lidar.launch.py")
                )
            ),
            launch_ros.actions.Node(
                parameters=[
                    get_package_share_directory("slam_toolbox")
                    + "/config/mapper_params_online_sync.yaml"
                ],
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                remappings=[("odom", "odom_combined")],
            ),
        ]
    )
