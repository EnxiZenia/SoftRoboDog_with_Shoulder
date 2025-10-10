
🟢 joystick_publisher (Joystick Interface)

    Node: joy_subscriber

    Subscribes: /joy (sensor_msgs/Joy)

    Publishes: /joystick_command (JoystickParameters)

Purpose:
Reads input from a physical joystick and translates it into structured commands such as:

    Turn/position direction

    Velocity

    Enable/disable flag

Used during manual control.


🟢 motor_command.py (Motion Controller)

    Node: motor_command_subscriber

    Subscribes: /joystick_command (JoystickParameters)

    Publishes: /motor_command (MotorParameters)

Purpose:
Transforms joystick commands into per-wheel control parameters (rotational and vertical motion) using wave functions and reference values. 


🟢 transmit_command.py (Command Translator)

    Node: motor_command_subscriber

    Subscribes: /motor_command (MotorParameters)

    Publishes: /StmData (Stm32Data)

Purpose:
Processes each wheel’s motor values, applies pre-tensioning offsets, and formats the data into discrete motor angles. Acts as a final stage before sending to the microcontroller.

Each of these nodes is tightly coupled through a series of custom ROS messages defined in my_parameters/msg

🟢 sllidar_ros2 (LIDAR)

    Launch Node: sllidar_ros2

    Publishes: /scan (sensor_msgs/LaserScan)

Purpose:
Launches and manages the SLLIDAR device, continuously streaming 2D laser scan data used for mapping, localization, and odometry.

🟢 bno055 (IMU)

    Launch Node: bno055

    Publishes: /imu (sensor_msgs/Imu)

Purpose:
Provides orientation data via an onboard IMU sensor. Currently not used in the navigation or odometry pipeline.
🟢 ros2_laser_scan (Odometry Estimation)

    Launch Node: csm (from ros2_laser_scan package)

    Subscribes: /scan

    Publishes: /odom (nav_msgs/Odometry)

Purpose:
Performs 2D scan-matching to estimate odometry using LIDAR data. Implements Canonical Scan Matcher (CSM) for robust movement tracking without wheel encoders.

🟢 cmd_vel_to_joystick (Autonomous Control Bridge)

    Node: cmd_vel_to_joystick

    Subscribes: /cmd_vel (geometry_msgs/Twist)

    Publishes: /joystick_command (JoystickParameters)

Purpose:
Converts autonomous navigation commands (cmd_vel) into joystick-style messages. This allows the robot to use the same control pipeline for both manual and autonomous driving by mimicking joystick input.















🚀 Launch System Overview

This section describes how to launch the robot system in both manual and autonomous modes, across the Raspberry Pi (control) and PC (navigation & mapping).
🤖 Raspberry Pi (Low-Level Control & Motion Execution)

All Raspberry Pi launch files are part of the soft_robot_bringub package. Depending on the mode (manual or autonomous), different launch files are used.
🕹️ Manual Mode (Teleoperation)

Launch Command:

ros2 launch soft_robot_bringub control_motion_joystick.launch.py

What it launches:

    joy_linux_node: Reads input from the physical joystick

    joystick_publisher: Converts joystick input into control messages

    motor_commands: Generates wheel-specific commands (angles, movement)

    transmit_command: Prepares values for STM32 transmission

    motor_serial: Sends values via serial to STM32

🤖 Autonomous Mode

Launch Command:

ros2 launch soft_robot_bringub bringup1.launch.py

What it includes:

    sensors.launch.py:

        joy_linux_node: Still active for possible manual override

        sllidar_ros2: Launches the SLLIDAR node

        (Optional) bno055: IMU (currently commented out)

    control_motion_autonomous.launch.py:

        cmd_vel_to_joystick: Converts navigation commands to joystick-style messages

        motor_commands: Generates motor parameters

        transmit_command: Prepares motor messages

        motor_serial: Sends values to STM32

🧠 PC (High-Level Localization & Navigation)

Run the following in two separate terminals from inside your PC's ros2_ws workspace.
📍 1. Mapping & Localization

Launch Command:

ros2 launch soft_robot_bringub mapping_and_localization.launch.py

What it does:

    Sets static transform for LIDAR

    Runs laser_scan_matcher for odometry estimation from LIDAR

    Starts the robot_localization EKF node using parameters from config/ekf_config.yaml

    Optional nodes like SLAM toolbox and IMU transform are defined but currently commented out.

🧭 2. Navigation & RViz

Launch Command:

ros2 launch soft_robot_bringub navigation.launch.py

What it does:

    Launches the full nav2_bringup stack with your map and parameters

    Loads RViz with default navigation config

Custom parameters used:

    my_map12.yaml → saved map

    nav2_params.yaml → Nav2 configuration

    rviz2 → default Nav2 visualization setup

🗂️ Configuration Files

All parameter files (for EKF, GMapping, AMCL, Nav2, and map YAMLs) are located in the:

soft_robot_bringub/config/

Make sure to adjust them as needed for your environment and tuning.
💾 Saving a Map

After running autonomous navigation or SLAM, you can save the map using:

ros2 run nav2_map_server map_saver_cli -f my_map

This saves my_map.yaml and the associated .pgm file to your current directory.





# Source the ROS 2 workspace (must be done in every new terminal)
source install/setup.bash

# SSH into the Raspberry Pi with X11 forwarding enabled (useful for GUI apps) the ssh can change, check the connecton here http://192.168.0.1/ the username and password is "admin" then choose DHCP and then DHCP Clients list. check the ssh there
ssh -X pi2@192.168.0.103
password: raspberry

# Create a new C++ ROS 2 package using ament_cmake build system
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>

# Create a new Python ROS 2 package using ament_python build system
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>

# Launch the SLLIDAR node to visualize real-time laser scan output
ros2 launch sllidar_ros2 view_sllidar_a2m8_launch.py

# View the current TF tree as a PDF (generates frames.pdf)
ros2 run tf2_tools view_frames

# Open the generated PDF (used after view_frames) and change it to the correct pdf file
evince frames.pdf

# Save the current map using the Nav2 map saver tool
ros2 run nav2_map_server map_saver_cli -f my_map

# Build only the 'my_parameters' package
colcon build --packages-select my_parameters

# Build all the main control packages on the Raspberry Pi (dont build the lidar package!! it takes so long time. dont build the csm on raspberrypi, it takes so long time)
colcon build --packages-select joystick_publisher motor_serial motor_commands cmd_vel_to_joystick soft_robot_bringub my_parameters

# Build only the 'soft_robot_bringub' package (launch and config files)
colcon build --packages-select soft_robot_bringub

# Launch LIDAR-based odometry and localization (runs on PC)
ros2 launch soft_robot_bringub mapping_and_localization.launch.py

# Launch the full navigation stack (Nav2) along with RViz (runs on pc)
ros2 launch soft_robot_bringub navigation.launch.py

# run this when you want to manually control the robot with joystick without the sensors (runs on pi)
ros2 launch soft_robot_bringub control_motion_joystick.launch.py

# Full autonomous bringup on Raspberry Pi (sensors + motor control + cmd_vel) (runs on pi)
ros2 launch soft_robot_bringub bringup1.launch.py
