#include <falco_local_planner/PathFollower.h>

PathFollower::PathFollower() : Node("pathFollower")
{
  // Declare parameters  
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
  noRotAtGoal       = this->declare_parameter<bool>("noRotAtGoal", false);  
  autonomySpeed     = this->declare_parameter<double>("autonomySpeed", 0.0);

  // Subscribers
  subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
      "state_estimation", 5, std::bind(&PathFollower::odomHandler, this, std::placeholders::_1));
  subPath = this->create_subscription<nav_msgs::msg::Path>(
      "path", 5, std::bind(&PathFollower::pathHandler, this, std::placeholders::_1)); 
  
  subStop = this->create_subscription<std_msgs::msg::Int8>(
      "stop", 5, std::bind(&PathFollower::stopHandler, this, std::placeholders::_1));

  // Publisher
  pubSpeed = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 5);

  
  joySpeed = autonomySpeed / maxSpeed;
  
  if (joySpeed < 0) joySpeed = 0;
  else if (joySpeed > 1.0) joySpeed = 1.0;
  
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
  vehicleX = odomIn->pose.pose.position.x ;
  vehicleY = odomIn->pose.pose.position.y ;
  vehicleZ = odomIn->pose.pose.position.z;
  
  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || 
        fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if (!pathInit || path.poses.empty()) return;

  // Transform vehicle position into recorded frame (only yaw)
  tf2::Quaternion q;
  q.setRPY(0, 0, vehicleYawRec);
  tf2::Transform tf(q);
  tf.setOrigin(tf2::Vector3(vehicleXRec, vehicleYRec, vehicleZRec));

  // Current vehicle position in world
  tf2::Vector3 vehicle_world(vehicleX, vehicleY, vehicleZ);

  // Transform current vehicle into recorded frame
  tf2::Vector3 vehicle_rel = tf.inverse() * vehicle_world;

  float vehicleXRel = vehicle_rel.x();
  float vehicleYRel = vehicle_rel.y();
  float vehicleZRel = vehicle_rel.z();

  int pSize = path.poses.size();
  float endDisX = path.poses[pSize - 1].pose.position.x - vehicleXRel;
  float endDisY = path.poses[pSize - 1].pose.position.y - vehicleYRel;
  float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

  float disX, disY, dis;
  while (pathPointID < pSize - 1) {
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

  if (pSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
    vehicleYawRate = 0;
  }
  if (pSize <= 1) {
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
  if (safetyStop >= 1) vehicleYawRate = 0;

  pubSkipCount--;
  if (pubSkipCount < 0) {   
    if (fabs(vehicleSpeed) <= maxAccel / 100.0) cmd_vel.linear.x = 0;
    else cmd_vel.linear.x = vehicleSpeed;
    cmd_vel.angular.z = vehicleYawRate;
    pubSpeed->publish(cmd_vel);

    pubSkipCount = pubSkipNum;
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



void PathFollower::stopHandler(const std_msgs::msg::Int8::SharedPtr stop) 
{ 
  safetyStop = stop->data;
 }

