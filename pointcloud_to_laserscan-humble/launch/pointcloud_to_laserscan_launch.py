from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="scanner",
                default_value="scanner",
                description="Namespace for sample topics",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                # remappings=[("cloud_in", "/point_cloud_raw"), ("scan", "/scan")],
                remappings=[("cloud_in", "/unilidar/cloud"), ("scan", "/scan")],
                parameters=[
                    {
                        "target_frame": "base_footprint",
                        # "target_frame": "laser",
                        "transform_tolerance": 0.01,
                        "min_height": -0.03,
                        "max_height": 2.0,
                        "angle_min": -3.14159,  # -M_PI/2
                        "angle_max": 3.14159,  # M_PI/2
                        "angle_increment": 0.0087,  # M_PI/360.0
                        "scan_time": 0.1,
                        "range_min": 1.0,
                        "range_max": 200.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                name="pointcloud_to_laserscan",
            ),
        ]
    )
