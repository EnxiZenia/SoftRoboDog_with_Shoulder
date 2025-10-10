from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Path to the EKF configuration file
config_file1 = os.path.join(
    get_package_share_directory('soft_robot_bringub'), 'config', 'ekf_config.yaml'
)

config_file2 = os.path.join(
    get_package_share_directory('soft_robot_bringub'), 'config', 'gmapping_config.yaml'
)

config_file3 = os.path.join(
    get_package_share_directory('soft_robot_bringub'), 'config', 'amcl_config.yaml'
)

config_file4 = os.path.join(
    get_package_share_directory('soft_robot_bringub'), 'config', 'my_map.yaml'
)
def generate_launch_description():
    return LaunchDescription([


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_static_lidar',
            arguments=['0.0', '0.0', '0.26', '1.57', '0', '0','base_link', 'laser']
        ),



        # Static transform between base_link and bno055 (IMU)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='tf_static_bno055',
        #     # arguments=['0.0', '0.0', '0.16', '4.71', '0', '0', 'base_link', 'bno055']
        #     # arguments=['0.0', '0.0', '0.16', '1.57', '0', '0', 'base_link', 'bno055']
        #     arguments=['0.0', '0.0', '0.16', '0', '0',  '0', 'base_link', 'bno055']

        # ),

        Node(
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            name='laser_scan_matcher',
            output='screen'
        ),

        # Launch the EKF localization node (no need for IncludeLaunchDescription)
        Node(
            package='robot_localization',
            executable='ekf_node',  # this is the EKF localization node
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file1]  # path to the EKF config file
        ),

        # comment it if you have saved map. if you want to do the mapping from begging, then use this node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config_file2]  # SLAM Toolbox config file
        )
        # then run in another terminal the following:ros2 run nav2_map_server map_saver_cli -f my_map
        
    ])
