#ifndef LOCAL_PLANNER_HPP
#define LOCAL_PLANNER_HPP

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"

#include "falco_local_planner/msg/joystick_data.hpp"

using namespace std;
using namespace std::chrono_literals;

const double PI = 3.1415926;
#define PLOTPATHSET 1 

class LocalPlanner : public rclcpp::Node
{
public:
  LocalPlanner();
  void run();

private:
  // === TF2 buffer and listener ===
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // === Parameters ===
  std::string pathFolder, odom, vehicle;
  double vehicleLength, vehicleWidth,vehicleHeight, sensorOffsetX, sensorOffsetY;
  bool twoWayDrive, useTerrainAnalysis, checkObstacle, checkRotObstacle;
  double laserVoxelSize, terrainVoxelSize, adjacentRange;
  double obstacleHeightThre, groundHeightThre, costHeightThre, costScore;
  bool useCost;
  const int laserCloudStackNum = 1;
  int laserCloudCount = 0;
  int pointPerPathThre;
  double minRelZ, maxRelZ, maxSpeed, dirWeight, dirThre;
  bool dirToVehicle;
  double pathScale, minPathScale, pathScaleStep;
  bool pathScaleBySpeed;
  double minPathRange, pathRangeStep;
  bool pathRangeBySpeed, pathCropByGoal;
  bool autonomyMode;
  double autonomySpeed, joyToSpeedDelay, joyToCheckObstacleDelay;
  double goalClearRange, goalX, goalY,goalZ;
  
  

  float joySpeed = 0, joySpeedRaw = 0, joyDir = 0;

  // === Constants ===
  static constexpr int pathNum = 343;
  static constexpr int groupNum = 7;
  float gridVoxelSize = 0.02;
  float searchRadius = 0.45;
  float gridVoxelOffsetX = 3.2;
  float gridVoxelOffsetY = 4.5;
  static constexpr int gridVoxelNumX = 161;
  static constexpr int gridVoxelNumY = 451;
  static constexpr int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

  // === Point clouds ===
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud, laserCloudCrop, laserCloudDwz;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud, terrainCloudCrop, terrainCloudDwz;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[1];
  pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud, plannerCloudCrop, boundaryCloud, addedObstacles;
  pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[7];
#if PLOTPATHSET == 1
  pcl::PointCloud<pcl::PointXYZI>::Ptr paths[343], freePaths;
#endif

  // === Path data ===
  int pathList[pathNum] = {0};
  float endDirPathList[pathNum] = {0};
  int clearPathList[36 * pathNum] = {0};
  float pathPenaltyList[36 * pathNum] = {0};
  float clearPathPerGroupScore[36 * groupNum] = {0};
  std::vector<int> correspondences[gridVoxelNum];

  // === State ===
  bool newLaserCloud = false, newTerrainCloud = false;
  double odomTime = 0, joyTime = 0;
  float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
  float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

  // === Filters ===
  pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;

  // === ROS2 Subscribers & Publishers ===
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud, subTerrainCloud;
  rclcpp::Subscription<falco_local_planner::msg::JoystickData>::SharedPtr subJoystick;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subGoal;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSpeed;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subBoundary;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subAddedObstacles;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subCheckObstacle;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laserCloudDwzPub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plannerCloudPub;
#if PLOTPATHSET == 1
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFreePaths;
#endif

  // === Private methods ===
  void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom);
  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg_in);
  void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr terrainCloud2);
  void joystickCallback(const falco_local_planner::msg::JoystickData::SharedPtr msg);
  void goalHandler(const geometry_msgs::msg::PointStamped::SharedPtr goal);
  void speedHandler(const std_msgs::msg::Float32::SharedPtr speed);
  void boundaryHandler(const geometry_msgs::msg::PolygonStamped::SharedPtr boundary);
  void addedObstaclesHandler(const sensor_msgs::msg::PointCloud2::SharedPtr addedObstacles2);
  void checkObstacleHandler(const std_msgs::msg::Bool::SharedPtr checkObs);
  tf2::Vector3 transformPointWithYaw(float x, float y, float z,float yaw, float scale);

  int readPlyHeader(FILE *filePtr);
  void readStartPaths();
#if PLOTPATHSET == 1
  void readPaths();
#endif
  void readPathList();
  void readCorrespondences();
};

#endif // LOCAL_PLANNER_HPP
