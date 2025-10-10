cmake_minimum_required(VERSION 3.8)
project(my_parameters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)  # <<-- important for Time/Duration

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorParameters.msg"
  "msg/JoystickParameters.msg"
  "msg/Stm32Data.msg"
  "msg/DebugParameter.msg"
  "msg/ObstacleProximityLog.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_package()
