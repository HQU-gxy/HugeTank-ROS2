import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# def launch(launch_descriptor, argv):
def generate_launch_description():
    # Lidar and the PointCloud2LaserScan transformer
    lslidar_dir = get_package_share_directory("lslidar_driver")
    unilidar_dir = get_package_share_directory("unitree_lidar_ros2")
    pc2ls_dir = get_package_share_directory("pointcloud_to_laserscan")

    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lslidar_dir, "launch", "lslidar_c32_launch.py")
        )
    )
    
    unilidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unilidar_dir, "launch.py")
        )
    )


    point_to_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pc2ls_dir, "launch", "pointcloud_to_laserscan_launch.py")
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(unilidar_launch)
    ld.add_action(point_to_scan)
    return ld
