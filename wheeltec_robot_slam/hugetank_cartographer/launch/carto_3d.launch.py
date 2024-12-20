"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-configuration_directory",
            LaunchConfiguration("cartographer_config_dir"),
            "-configuration_basename",
            LaunchConfiguration("configuration_basename"),
        ],
        remappings=[("points2", "point_cloud_raw"), ("imu", "/imu/data")],
        output="screen",
    )

    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        parameters=[{"use_sim_time": True}, {"resolution": 0.05}],
        # remappings=[('odom','odom'),
        #    ('imu','/imu/data')],
    )

    bringup_dir = get_package_share_directory("hugetank_link")
    launch_dir = os.path.join(bringup_dir, "launch")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "hugetank_link.launch.py")
                ),
                launch_arguments={"carto_slam": "true"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "lidar.launch.py")
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument(
                "cartographer_config_dir",
                default_value=os.path.join(
                    get_package_share_directory("cartographer"), "config"
                ),
            ),
            DeclareLaunchArgument(
                "configuration_basename", default_value="carto_3d.lua"
            ),
            cartographer_node,
            cartographer_occupancy_grid_node,
        ]
    )
