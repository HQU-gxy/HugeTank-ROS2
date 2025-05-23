import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("hugetank_link")
    launch_dir = os.path.join(bringup_dir, "launch")

    ekf_config = Path(
        get_package_share_directory("hugetank_link"), "config", "ekf.yaml"
    )
    ekf_carto_config = Path(
        get_package_share_directory("hugetank_link"), "config", "ekf_carto.yaml"
    )

    imu_config = Path(
        get_package_share_directory("hugetank_link"), "config", "imu.yaml"
    )

    carto_slam = LaunchConfiguration("carto_slam", default="false")

    carto_slam_dec = DeclareLaunchArgument("carto_slam", default_value="false")

    serial_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "base_serial.launch.py"))
    )

    robot_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "wheeltec_ekf.launch.py")
        ),
        launch_arguments={"carto_slam": carto_slam}.items(),
    )

    base_to_link = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_link",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"],
    )
    base_to_gyro = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_gyro",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "gyro_link"],
    )

    imu_filter_node = launch_ros.actions.Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[imu_config],
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=[
            os.path.join(
                get_package_share_directory("hugetank_link"), "urdf", "huge_tank.urdf"
            )
        ],
    )

    tank_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "model_description.launch.py")
        ),
        launch_arguments={"hugeTank": "true"}.items(),
    )

    ld = LaunchDescription()

    ld.add_action(carto_slam_dec)
    ld.add_action(serial_link)
    ld.add_action(base_to_link)
    ld.add_action(base_to_gyro)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(imu_filter_node)
    ld.add_action(robot_ekf)
    ld.add_action(tank_model)

    return ld
