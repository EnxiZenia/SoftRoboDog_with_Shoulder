import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sensor_launch_file = os.path.join(
        get_package_share_directory('soft_robot_bringub'), 'launch', 'sensors.launch.py'
    )

    control_motion_launch_file = os.path.join(
        get_package_share_directory('soft_robot_bringub'), 'launch', 'control_motion_autonomous.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensor_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_motion_launch_file)
        ),
    ])