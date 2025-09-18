cmake_minimum_required(VERSION 3.5)
project(falco_local_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(message_filters REQUIRED)
find_package(joy REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JoystickData.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# Build the node
add_executable(joystick_interface 
src/joystick_interface.cpp
)
ament_target_dependencies(joystick_interface 
  rclcpp 
  sensor_msgs
)

# THIS LINE LINKS GENERATED MESSAGES TO THE NODE
rosidl_target_interfaces(joystick_interface
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Build executables
add_executable(localPlanner 
  src/localPlannerMain.cpp
  src/localPlanner.cpp
)
ament_target_dependencies(localPlanner
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_conversions
  PCL
  message_filters
  tf2_geometry_msgs
  joy
  tf2_sensor_msgs
)
rosidl_target_interfaces(localPlanner ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pathFollower 
src/pathFollower.cpp
src/pathFollowerMain.cpp
)
ament_target_dependencies(pathFollower
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  joy
)
rosidl_target_interfaces(pathFollower ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Install executables
install(TARGETS
  localPlanner
  pathFollower
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install path files
install(DIRECTORY
  paths
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  joystick_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()