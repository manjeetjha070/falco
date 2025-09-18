# falco_local_planner

## Overview
`falco_local_planner` is a ROS2 package that implements a **local planner** for autonomous and manual navigation.  
It integrates sensor data, TF transforms, and planning logic to generate feasible local trajectories.  
The package is designed to work within the ROS2 navigation stack and can be customized for different robots and environments.

## Features
- ROS2 C++ implementation (`rclcpp`)  
- Integration with common ROS2 messages (`sensor_msgs`, `nav_msgs`, `geometry_msgs`)  
- Support for TF2 transformations (`tf2`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_sensor_msgs`)  
- Point Cloud Library (PCL) support for 3D perception (`pcl_conversions`, `pcl_msgs`)  
- Custom ROS2 messages (in `msg/`)  
- Launch files for easy startup (in `launch/`)  

## Dependencies
This package depends on the following ROS2 packages and libraries:

- `rclcpp`  
- `std_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`  
- `tf2`, `tf2_ros`, `tf2_sensor_msgs`, `tf2_geometry_msgs`  
- `pcl_conversions`, `PCL`, `pcl_msgs`  
- `message_filters`  
- `joy`  
- `rosidl_default_generators`, `rosidl_default_runtime`  

Make sure you have installed these via your ROS2 distribution.  

## Build Instructions
Clone the package into your ROS2 workspace (`src/` folder) and build with `colcon`:

```bash
cd ~/ros2_ws/src
git clone <your-repo-url>
cd ~/ros2_ws
colcon build --packages-select falco_local_planner
source install/setup.bash
```

## Running the Package


Launch files are provided in the launch/ directory:

```bash
ros2 launch falco_local_planner local_planner.launch.py
```


## Usage & Configuration

The planner can be customized by editing the parameters inside the launch files.
Some common settings you can adjust include:

- `Vehicle size → define the vehicle’s footprint and dimensions for safe local planning`

- `Odometry source → select the topic providing odometry information`

- `Sensor inputs → adjust which LiDAR or camera topics are used`

- `Planner parameters → tuning parameters such as planning horizon, speed limits, or safety margins`

Example: open a launch file in the launch/ directory and modify parameters like:

```python
vehicle_length = 2.5   # meters
vehicle_width  = 1.5   # meters
odom_topic     = "/odom"
```


This makes it easy to adapt the planner to different robot platforms.

## Custom Messages

This package defines custom messages located in the msg/ directory.
These messages are built automatically when you compile the package.
To check available messages:

```bash
ros2 interface list | grep falco_local_planner
```

## Development

- `Source code: src/`

- `Headers: include/`

- `Messages: msg/`

- `Launch files: launch/`

- `Configurations/paths: paths/`

