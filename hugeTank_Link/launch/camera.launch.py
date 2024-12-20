import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# def launch(launch_descriptor, argv):
def generate_launch_description():
    astra_dir = get_package_share_directory("astra_camera")

    Astra_S = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(astra_dir, "launch", "astra_mini.launch.py")
        ),
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(Astra_S)

    return ld
