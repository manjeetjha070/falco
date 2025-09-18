#include "pathFollower.hpp"


PathFollower::PathFollower() : Node("pathFollower")
{
  // Declare parameters
  sensorOffsetX     = this->declare_parameter<double>("sensorOffsetX", 0.0);
  sensorOffsetY     = this->declare_parameter<double>("sensorOffsetY", 0.0);
  pubSkipNum        = this->declare_parameter<int>("pubSkipNum", 0);
  twoWayDrive       = this->declare_parameter<bool>("twoWayDrive", true);
  lookAheadDis      = this->declare_parameter<double>("lookAheadDis", 0.0);
  yawRateGain       = this->declare_parameter<double>("yawRateGain", 0.0);
  stopYawRateGain   = this->declare_parameter<double>("stopYawRateGain", 0.0);
  maxYawRate        = this->declare_parameter<double>("maxYawRate", 0.0);
  maxSpeed          = this->declare_parameter<double>("maxSpeed", 0.0);
  maxAccel          = this->declare_parameter<double>("maxAccel", 0.0);
  switchTimeThre    = this->declare_parameter<double>("switchTimeThre", 0.0);
  dirDiffThre       = this->declare_parameter<double>("dirDiffThre", 0.0);
  stopDisThre       = this->declare_parameter<double>("stopDisThre", 0.0);
  slowDwnDisThre    = this->declare_parameter<double>("slowDwnDisThre", 0.0);
  useInclRateToSlow = this->declare_parameter<bool>("useInclRateToSlow", false);
  inclRateThre      = this->declare_parameter<double>("inclRateThre", 0.0);
  slowRate1         = this->declare_parameter<double>("slowRate1", 0.0);
  slowRate2         = this->declare_parameter<double>("slowRate2", 0.0);
  slowTime1         = this->declare_parameter<double>("slowTime1", 0.0);
  slowTime2         = this->declare_parameter<double>("slowTime2", 0.0);
  useInclToStop     = this->declare_parameter<bool>("useInclToStop", false);
  inclThre          = this->declare_parameter<double>("inclThre", 0.0);
  stopTime          = this->declare_parameter<double>("stopTime", 0.0);
  noRotAtStop       = this->declare_parameter<bool>("noRotAtStop", false);
  noRotAtGoal       = this->declare_parameter<bool>("noRotAtGoal", false);
  autonomyMode      = this->declare_parameter<bool>("autonomyMode", false);
  autonomySpeed     = this->declare_parameter<double>("autonomySpeed", 0.0);
  joyToSpeedDelay   = this->declare_parameter<double>("joyToSpeedDelay", 0.0);
  vehicle           = this->declare_parameter<std::string>("vehicle", "robot");

  // Subscribers
  subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
      "state_estimation", 5, std::bind(&PathFollower::odomHandler, this, std::placeholders::_1));
  subPath = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 5, std::bind(&PathFollower::pathHandler, this, std::placeholders::_1));
  subJoystick = this->create_subscription<falco_local_planner::msg::JoystickData>(
      "/joystick_data", 10,std::bind(&PathFollower::joystickCallback, this, std::placeholders::_1));
  subSpeed = this->create_subscription<std_msgs::msg::Float32>(
      "/speed", 5, std::bind(&PathFollower::speedHandler, this, std::placeholders::_1));
  subStop = this->create_subscription<std_msgs::msg::Int8>(
      "/stop", 5, std::bind(&PathFollower::stopHandler, this, std::placeholders::_1));

  // Publisher
  pubSpeed = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);
  cmd_vel.header.frame_id = vehicle;

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;
    
    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

// ===============================
// Callbacks
// ===============================
void PathFollower::odomHandler(const nav_msgs::msg::Odometry::SharedPtr odomIn) 
{ 
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();

  double roll, pitch, yaw;
  tf2::Quaternion tf_quat;
  tf2::fromMsg(odomIn->pose.pose.orientation, tf_quat);
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;
  
  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || 
        fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }
}
void PathFollower::pathHandler(const nav_msgs::msg::Path::SharedPtr pathIn) 
{ 
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;  
}

void PathFollower::joystickCallback(const falco_local_planner::msg::JoystickData::SharedPtr msg)
{
    joyTime = msg->joy_time;
    joySpeed = msg->joy_speed;
    joyYaw = msg->joy_yaw;
    autonomyMode = msg->autonomy_mode;
}

void PathFollower::speedHandler(const std_msgs::msg::Float32::SharedPtr speed) 
{ 
  rclcpp::Clock clock;
  double speedTime = clock.now().seconds();  
  //double speedTime = this->now().seconds();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }  
}
void PathFollower::stopHandler(const std_msgs::msg::Int8::SharedPtr stop) 
{ 
  safetyStop = stop->data;
 }

// ===============================
// Run Loop
// ===============================
void PathFollower::run()
{
  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(this->get_node_base_interface());

    if (pathInit) {
      // Transform vehicle position into recorded frame
      tf2::Quaternion q;
      q.setRPY(0, 0, vehicleYawRec);   // only yaw rotation
      tf2::Transform tf(q);
      tf.setOrigin(tf2::Vector3(vehicleXRec, vehicleYRec, vehicleZRec));

      // Current vehicle position in world
      tf2::Vector3 vehicle_world(vehicleX, vehicleY, vehicleZ);

      // Transform current vehicle into recorded frame
      tf2::Vector3 vehicle_rel = tf.inverse() * vehicle_world;

      // Extract relative coords
      float vehicleXRel = vehicle_rel.x();
      float vehicleYRel = vehicle_rel.y();
      float vehicleZRel = vehicle_rel.z();  // optional

      

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      if (twoWayDrive) {
        rclcpp::Clock clock;
        double time = clock.now().seconds();
        //double time = node->now().seconds();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
      else vehicleYawRate = -yawRateGain * dirDiff;

      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;

      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }
      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) joySpeed3 *= slowRate2;

      if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = rclcpp::Time(static_cast<int64_t>(odomTime * 1e9));
        //cmd_vel.header.stamp = node->now();
        if (fabs(vehicleSpeed) <= maxAccel / 100.0) cmd_vel.twist.linear.x = 0;
        else cmd_vel.twist.linear.x = vehicleSpeed;
        cmd_vel.twist.angular.z = vehicleYawRate;
        pubSpeed->publish(cmd_vel);

        pubSkipCount = pubSkipNum;
      }
         }

    
    rate.sleep();
  }
}
