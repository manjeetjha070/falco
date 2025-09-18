#ifndef PATH_FOLLOWER_HPP
#define PATH_FOLLOWER_HPP

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "falco_local_planner/msg/joystick_data.hpp"

using namespace std;

const double PI = 3.1415926;

class PathFollower : public rclcpp::Node {
public:
  PathFollower();

  void run();

private:
  // Parameters
  double sensorOffsetX;
  double sensorOffsetY;
  int pubSkipNum;
  bool twoWayDrive;
  double lookAheadDis;
  double yawRateGain;
  double stopYawRateGain;
  double maxYawRate;
  double maxSpeed;
  double maxAccel;
  double switchTimeThre;
  double dirDiffThre;
  double stopDisThre;
  double slowDwnDisThre;
  bool useInclRateToSlow;
  double inclRateThre;
  double slowRate1;
  double slowRate2;
  double slowTime1;
  double slowTime2;
  bool useInclToStop;
  double inclThre;
  double stopTime;
  bool noRotAtStop;
  bool noRotAtGoal;
  bool autonomyMode;
  double autonomySpeed;
  double joyToSpeedDelay;
  std::string vehicle;

  // State
  float joySpeed = 0, joySpeedRaw = 0, joyYaw = 0;
  int safetyStop = 0;
  float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
  float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
  float vehicleXRec = 0, vehicleYRec = 0, vehicleZRec = 0;
  float vehicleRollRec = 0, vehiclePitchRec = 0, vehicleYawRec = 0;
  float vehicleYawRate = 0, vehicleSpeed = 0;

  double odomTime = 0, joyTime = 0, slowInitTime = 0, stopInitTime = 0;
  int pathPointID = 0;
  bool pathInit = false, navFwd = true;
  double switchTime = 0;

  int pubSkipCount = 0;

  nav_msgs::msg::Path path;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
  rclcpp::Subscription<falco_local_planner::msg::JoystickData>::SharedPtr subJoystick;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSpeed;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subStop;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubSpeed;
  geometry_msgs::msg::TwistStamped cmd_vel;

  // Callbacks
  void odomHandler(const nav_msgs::msg::Odometry::SharedPtr odomIn);
  void pathHandler(const nav_msgs::msg::Path::SharedPtr pathIn);
  void joystickCallback(const  falco_local_planner::msg::JoystickData::SharedPtr msg);
  void speedHandler(const std_msgs::msg::Float32::SharedPtr speed);
  void stopHandler(const std_msgs::msg::Int8::SharedPtr stop);
};

#endif // PATH_FOLLOWER_HPP
