cmake_minimum_required(VERSION 3.22.1)
project(falco_local_planner)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(joy REQUIRED)

# Find none ROS dependencies
find_package(PCL REQUIRED COMPONENTS common io filters)


# -----------------------------------------------------------------------------
# local_planner
# -----------------------------------------------------------------------------
add_executable (local_planner 
    src/local_planner.cpp
    src/LocalPlanner.cpp
)

target_include_directories(local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(local_planner
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

# -----------------------------------------------------------------------------
# path_follower
# -----------------------------------------------------------------------------
add_executable(path_follower 
    src/path_follower.cpp
    src/PathFollower.cpp
)
target_include_directories(path_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies( path_follower
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

# -----------------------------------------------------------------------------
# joystick_interface
# -----------------------------------------------------------------------------

add_executable (joystick_interface 
    src/joystick_interface.cpp
    src/JoystickInterface.cpp
)

target_include_directories(joystick_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies (joystick_interface 
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# -----------------------------------------------------------------------------
# install
# -----------------------------------------------------------------------------

install (
    TARGETS
        joystick_interface
        local_planner
        path_follower
    DESTINATION
        lib/${PROJECT_NAME}
)

install (
    DIRECTORY
        launch
        paths
    DESTINATION
        share/${PROJECT_NAME}
)

ament_package()