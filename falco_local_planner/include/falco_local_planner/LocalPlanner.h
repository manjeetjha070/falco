#ifndef INCLUDED_FALCO_LOCAL_PLANNER_LOCALPLANNER_H
#define INCLUDED_FALCO_LOCAL_PLANNER_LOCALPLANNER_H


#include <cmath>

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/float32.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#define FALCO_LOCAL_PLANNER_PLOTPATHSET 1

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner ();
    

private:
    // === TF2 buffer and listener ===
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    // === Parameters ===
    std::string pathFolder, frameIdOdom, frameIdvehicle;
    double vehicleLength, vehicleWidth, vehicleHeight;
    bool twoWayDrive, useTerrainAnalysis, checkObstacle, checkRotObstacle;
    double laserVoxelSize, terrainVoxelSize, adjacentRange;
    double obstacleHeightThre, groundHeightThre, costHeightThre, costScore;
    bool useCost;
    const int laserCloudStackNum = 1;
    int laserCloudCount          = 0;
    int pointPerPathThre;
    double minRelZ, maxRelZ, maxSpeed, dirWeight, dirThre;
    bool dirToVehicle;
    double pathScale, minPathScale, pathScaleStep;
    bool pathScaleBySpeed;
    double minPathRange, pathRangeStep;
    bool pathRangeBySpeed, pathCropByGoal;

    double autonomySpeed, joyToSpeedDelay, joyToCheckObstacleDelay;
    double goalClearRange;

    geometry_msgs::msg::PointStamped goal;


    float joySpeed = 0, joyDir = 0;

    // === Constants ===
    static constexpr int pathNum       = 343;
    static constexpr int groupNum      = 7;
    float gridVoxelSize                = 0.02;
    float searchRadius                 = 0.45;
    float gridVoxelOffsetX             = 3.2;
    float gridVoxelOffsetY             = 4.5;
    static constexpr int gridVoxelNumX = 161;
    static constexpr int gridVoxelNumY = 451;
    static constexpr int gridVoxelNum  = gridVoxelNumX * gridVoxelNumY;

    // === Point clouds ===
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud, laserCloudCrop, laserCloudDwz;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[1];
    pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud, plannerCloudCrop;
    pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[7];
#if FALCO_LOCAL_PLANNER_PLOTPATHSET == 1
    pcl::PointCloud<pcl::PointXYZI>::Ptr paths[343], freePaths;
#endif

    // === Path data ===
    int pathList[pathNum]                       = {0};
    float endDirPathList[pathNum]               = {0};
    int clearPathList[36 * pathNum]             = {0};
    float pathPenaltyList[36 * pathNum]         = {0};
    float clearPathPerGroupScore[36 * groupNum] = {0};
    std::vector<int> correspondences[gridVoxelNum];

    // === State ===
    bool newLaserCloud = false, newTerrainCloud = false;
    double odomTime = 0, joyTime = 0;
    float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
    float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

    // === Filters ===
    pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter;

    // === ROS2 Subscribers & Publishers ===
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subGoal;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subJoyDir;


    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laserCloudDwzPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plannerCloudPub;
#if FALCO_LOCAL_PLANNER_PLOTPATHSET == 1
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFreePaths;
#endif

    // === Private methods ===    
    void laserCloudHandler (const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg_in);
    void goalHandler (const geometry_msgs::msg::PointStamped::SharedPtr newGoal);
    void joyDirCallback(const std_msgs::msg::Float32::SharedPtr msg);

    tf2::Vector3 transformPointWithYaw (float x, float y, float z, float yaw, float scale);

    int readPlyHeader (FILE* filePtr);
    void readStartPaths ();
#if FALCO_LOCAL_PLANNER_PLOTPATHSET == 1
    void readPaths ();
#endif
    void readPathList ();
    void readCorrespondences ();
};

#endif // INCLUDED_FALCO_LOCAL_PLANNER_LOCALPLANNER_H
