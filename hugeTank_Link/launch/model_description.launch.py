import os

from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from math import pi


def generate_robot_node(robot_urdf, child):
    return launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"robot_state_publisher_{child}",
        arguments=[
            os.path.join(
                get_package_share_directory("hugetank_link"), "urdf", robot_urdf
            )
        ],
    )


def generate_static_transform_publisher_node(translation, rotation, parent, child):
    return launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"base_to_{child}",
        arguments=[
            translation[0],
            translation[1],
            translation[2],
            rotation[0],
            rotation[1],
            rotation[2],
            parent,
            child,
        ],
    )


def generate_launch_description():
    hugeTank = LaunchConfiguration("hugeTank", default="true")

    hugeTank_Action = GroupAction(
        condition=IfCondition(hugeTank),
        actions=[
            generate_robot_node("huge_tank.urdf", "huge_tank"),
            # generate_static_transform_publisher_node(['0.11', '0', '0.28'], ['-1.57', '0', '0'], 'base_footprint', 'laser'),
            generate_static_transform_publisher_node(
                ["0.25", "0", "0.45"],
                ["0", str(-pi / 2), str(pi)],
                # [ str(3 * pi / 4), str(pi / 2),"0"],
                "base_footprint",
                "unilidar_lidar",
            ),
            generate_static_transform_publisher_node(
                ["0.2", "0", "0.22"], ["0", "0", "0"], "base_footprint", "camera_link"
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(hugeTank_Action)
    return ld
