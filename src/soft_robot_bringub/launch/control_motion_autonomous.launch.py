import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument

# Path to the EKF configuration file
config_file = os.path.join(
    get_package_share_directory('soft_robot_bringub'), 'config', 'ekf_config.yaml'
)

def generate_launch_description():
    return LaunchDescription([

        
        Node(
            package='cmd_vel_to_joystick',
            executable='cmd_vel_to_joystick',
            name='cmd_vel_to_joystick',
            output='screen'
        ),
        
        # Launch motor_commands.py node
        Node(
            package='motor_commands',
            executable='motor_commands',
            name='motor_commands',
            output='screen'
        ),
        
        # Launch transmit_command.py node
        Node(
            package='motor_commands',
            executable='transmit_command',
            name='transmit_command',
            output='screen'
        ),
        
        # Launch motor_serial node
        Node(
            package='motor_serial',
            executable='motor_serial',
            name='motor_serial',
            output='screen'
        ),

    ])
