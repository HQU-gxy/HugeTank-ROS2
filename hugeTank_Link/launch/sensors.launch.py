import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


# def launch(launch_descriptor, argv):
def generate_launch_description():
    bringup_dir = get_package_share_directory("hugetank_link")
    launch_dir = os.path.join(bringup_dir, "launch")
    lidar_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "lidar.launch.py")),
    )
    wheeltec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "camera.launch.py")),
    )

    return LaunchDescription([lidar_ros, wheeltec_camera])
