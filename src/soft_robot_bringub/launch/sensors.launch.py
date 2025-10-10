from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


sllidar_launch_file = os.path.join(
    get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a2m8_launch.py'
)

def generate_launch_description():
    return LaunchDescription([
        # Launch joystick_publisher node


        # Node(
        #     package='bno055',
        #     executable='bno055',
        #     name='bno055',
        #     output='screen',
        #     respawn=True,  
        #     respawn_delay=2.0 
        # ),
        
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_file)
        ),
        
    ])
