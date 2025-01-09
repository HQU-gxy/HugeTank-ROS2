import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    tank_link_dir = get_package_share_directory("hugetank_link")
    tank_launch_dir = os.path.join(tank_link_dir, "launch")

    tank_nav_dir = get_package_share_directory("hugetank_nav2")
    tank_nav_launcher = os.path.join(tank_nav_dir, "launch")
    map_dir = os.path.join(tank_nav_dir, "map")
   
    map_file = LaunchConfiguration(
        "map", default=os.path.join(map_dir, "aisle.yaml")
    )

    param_dir = os.path.join(tank_nav_dir, "param")
    param_file = LaunchConfiguration(
        "params", default=os.path.join(param_dir, "huge_tank_params_stupid.yaml")
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=map_file,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params",
                default_value=param_file,
                description="Full path to param file to load",
            ),
            Node(
                name="waypoint_cycle",
                package="nav2_waypoint_cycle",
                executable="nav2_waypoint_cycle",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [tank_launch_dir, "/hugetank_link.launch.py"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [tank_launch_dir, "/lidar.launch.py"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([tank_nav_launcher, "/bringup_launch.py"]),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": param_file,
                }.items(),
            ),
        ]
    )
