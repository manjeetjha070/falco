
#include <falco_local_planner/LocalPlanner.h>

#include <cmath>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


LocalPlanner::LocalPlanner ()
    : Node ("localPlanner")
{
    // Initialize tf2 buffer and listener for transform management
    tfBuffer                = std::make_shared<tf2_ros::Buffer> (get_clock());
    tfListener              = std::make_shared<tf2_ros::TransformListener> (*tfBuffer);

    // Declare parameters with default values
    pathFolder              = declare_parameter<std::string> ("pathFolder", "");
    vehicleLength           = declare_parameter<double> ("vehicleLength", 0.6);
    vehicleWidth            = declare_parameter<double> ("vehicleWidth", 0.6);
    vehicleHeight           = declare_parameter<double> ("vehicleHeight", 0.5);
    twoWayDrive             = declare_parameter<bool> ("twoWayDrive", true);

    laserVoxelSize          = declare_parameter<double> ("laserVoxelSize", 0.05);
    useTerrainAnalysis      = declare_parameter<bool> ("useTerrainAnalysis", false);
    checkObstacle           = declare_parameter<bool> ("checkObstacle", true);
    checkRotObstacle        = declare_parameter<bool> ("checkRotObstacle", false);
    adjacentRange           = declare_parameter<double> ("adjacentRange", 3.5);
    obstacleHeightThre      = declare_parameter<double> ("obstacleHeightThre", 0.2);
    groundHeightThre        = declare_parameter<double> ("groundHeightThre", 0.1);
    costHeightThre          = declare_parameter<double> ("costHeightThre", 0.1);
    costScore               = declare_parameter<double> ("costScore", 0.02);
    useCost                 = declare_parameter<bool> ("useCost", false);
    pointPerPathThre        = declare_parameter<int> ("pointPerPathThre", 2);
    minRelZ                 = declare_parameter<double> ("minRelZ", -0.5);
    maxRelZ                 = declare_parameter<double> ("maxRelZ", 0.25);
    maxSpeed                = declare_parameter<double> ("maxSpeed", 1.0);
    dirWeight               = declare_parameter<double> ("dirWeight", 0.02);
    dirThre                 = declare_parameter<double> ("dirThre", 90.0);
    dirToVehicle            = declare_parameter<bool> ("dirToVehicle", false);
    pathScale               = declare_parameter<double> ("pathScale", 1.0);
    minPathScale            = declare_parameter<double> ("minPathScale", 0.75);
    pathScaleStep           = declare_parameter<double> ("pathScaleStep", 0.25);
    pathScaleBySpeed        = declare_parameter<bool> ("pathScaleBySpeed", true);
    minPathRange            = declare_parameter<double> ("minPathRange", 1.0);
    pathRangeStep           = declare_parameter<double> ("pathRangeStep", 0.5);
    pathRangeBySpeed        = declare_parameter<bool> ("pathRangeBySpeed", true);
    pathCropByGoal          = declare_parameter<bool> ("pathCropByGoal", true);
    autonomySpeed           = declare_parameter<double> ("autonomySpeed", 1.0);
    goalClearRange          = declare_parameter<double> ("goalClearRange", 0.5);
    frameIdOdom             = declare_parameter<std::string> ("frameIdOdom", "robot/odom");
    frameIdvehicle          = declare_parameter<std::string> ("frameIdvehicle", "robot");

    // Initialize point clouds
    laserCloud              = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudCrop          = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudDwz           = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>());
    plannerCloud            = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>());
    plannerCloudCrop        = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>());
    freePaths               = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>());

    for (int i = 0; i < laserCloudStackNum; i++)
        laserCloudStack[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>());
    for (int i = 0; i < groupNum; i++)
        startPaths[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>());
#if FALCO_LOCAL_PLANNER_PLOTPATHSET == 1
    for (int i = 0; i < pathNum; i++)
        paths[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>());
#endif

    // Initialize filters
    laserDwzFilter.setLeafSize (laserVoxelSize, laserVoxelSize, laserVoxelSize);


    // Create subscribers
    
    subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2> (
        "/registered_scan", 5, std::bind (&LocalPlanner::laserCloudHandler, this, std::placeholders::_1));

    subGoal = create_subscription<geometry_msgs::msg::PointStamped> (
        "/way_point", 5, std::bind (&LocalPlanner::goalHandler, this, std::placeholders::_1));

    subJoyDir = create_subscription<std_msgs::msg::Float32>(
        "/joy_dir", 5, std::bind(&LocalPlanner::joyDirCallback, this, std::placeholders::_1));


    // Create publishers
    pubPath          = create_publisher<nav_msgs::msg::Path> ("/path", 5);
    laserCloudDwzPub = create_publisher<sensor_msgs::msg::PointCloud2> ("laser_cloud_dwz", 10);
    plannerCloudPub  = create_publisher<sensor_msgs::msg::PointCloud2> ("/planner_cloud_crop", 2);

#if FALCO_LOCAL_PLANNER_PLOTPATHSET == 1
    pubFreePaths = create_publisher<sensor_msgs::msg::PointCloud2> ("/free_paths", 2);
#endif

    // Read path files
    RCLCPP_INFO (get_logger(), "Reading path files.");

    joySpeed = autonomySpeed / maxSpeed;
    if (joySpeed < 0)
        joySpeed = 0;
    else if (joySpeed > 1.0)
        joySpeed = 1.0;

    readStartPaths();
#if FALCO_LOCAL_PLANNER_PLOTPATHSET == 1
    readPaths();
#endif
    readPathList();
    readCorrespondences();
    RCLCPP_INFO (get_logger(), "Initialization complete.");
}

// Callback & Utility Methods

void LocalPlanner::laserCloudHandler (const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg_in)
{
    sensor_msgs::msg::PointCloud2 laserCloud2; // Raw message, not a shared_ptr

    try
    {
        tfBuffer->transform (*lidar_msg_in, laserCloud2, frameIdvehicle, tf2::durationFromSec (0.1));

        // You can now use laserCloud2
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN (get_logger(), "TF transform failed: %s", ex.what());
    }

    // Convert incoming cloud (already in vehicle frame) directly to PCL
    laserCloud->clear();
    pcl::fromROSMsg (laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    float halfLength = vehicleLength / 2.0;
    float halfWidth  = vehicleWidth / 2.0;
    float halfHeight = vehicleHeight / 2.0;
    float minRange   = std::sqrt (halfLength * halfLength + halfWidth * halfWidth + halfHeight * halfHeight);

    // Crop near-vehicle points
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++)
    {
        point        = laserCloud->points[i];
        float pointX = point.x;
        float pointY = point.y;
        
        float dis    = std::sqrt (pointX * pointX + pointY * pointY);
        if (dis < adjacentRange && dis > minRange)
        {
            laserCloudCrop->push_back (point);
        }
    }

    // Voxel filter
    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud (laserCloudCrop);
    laserDwzFilter.filter (*laserCloudDwz);

    // Publish filtered incoming cloud using incoming stamp/frame
    sensor_msgs::msg::PointCloud2 laserCloudDwzMsg;
    pcl::toROSMsg (*laserCloudDwz, laserCloudDwzMsg);
    laserCloudDwzMsg.header.stamp    = lidar_msg_in->header.stamp;
    laserCloudDwzMsg.header.frame_id = frameIdvehicle; 
    laserCloudDwzPub->publish (laserCloudDwzMsg);

    plannerCloudCrop->clear();
    for (const auto &pt : laserCloudDwz->points)
    {
        float dis = std::sqrt (pt.x * pt.x + pt.y * pt.y);
        if (dis < adjacentRange && (pt.z > minRelZ && pt.z < maxRelZ))
        {
            plannerCloudCrop->push_back (pt);
        }
    }

    // publish planner cloud
    sensor_msgs::msg::PointCloud2 plannerCloudMsg;
    pcl::toROSMsg (*plannerCloudCrop, plannerCloudMsg);
    plannerCloudMsg.header.stamp    = lidar_msg_in->header.stamp;
    plannerCloudMsg.header.frame_id = frameIdvehicle;
    plannerCloudPub->publish (plannerCloudMsg);

    // remaining planner logic unchanged but use lidar_msg_in->header.stamp for published outputs below
    float pathRange = adjacentRange;
    if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;
    if (pathRange < minPathRange) pathRange = minPathRange;
    float relativeGoalDis = adjacentRange;

    float relativeGoalX   = goal.point.x;
    float relativeGoalY   = goal.point.y;
    try
    {
        geometry_msgs::msg::PointStamped goal_in = goal;
        geometry_msgs::msg::PointStamped goal_tf;
        tfBuffer->transform(goal_in, goal_tf, frameIdvehicle, tf2::durationFromSec(0.1));
        relativeGoalX = goal_tf.point.x;
        relativeGoalY = goal_tf.point.y;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(get_logger(), "Goal transform to %s failed: %s. Using goal as-is.", frameIdvehicle.c_str(), ex.what());
    }

    relativeGoalDis       = sqrt (relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
    joyDir                = atan2 (relativeGoalY, relativeGoalX) * 180 / M_PI;

    if (!twoWayDrive)
    {
        if (joyDir > 90.0)
            joyDir = 90.0;
        else if (joyDir < -90.0)
            joyDir = -90.0;
    }

    bool pathFound     = false;
    float defPathScale = pathScale;
    if (pathScaleBySpeed) pathScale = defPathScale * joySpeed;
    if (pathScale < minPathScale) pathScale = minPathScale;

    while (pathScale >= minPathScale && pathRange >= minPathRange)
    {
        for (int i = 0; i < 36 * pathNum; i++)
        {
            clearPathList[i]   = 0;
            pathPenaltyList[i] = 0;
        }
        for (int i = 0; i < 36 * groupNum; i++)
        {
            clearPathPerGroupScore[i] = 0;
        }

        float minObsAngCW  = -180.0;
        float minObsAngCCW = 180.0;
        float diameter = std::sqrt ((vehicleLength / 2.0) * (vehicleLength / 2.0)
                                    + (vehicleWidth / 2.0) * (vehicleWidth / 2.0));
        float angOffset          = atan2 (vehicleWidth, vehicleLength) * 180.0 / M_PI;
        int plannerCloudCropSize = plannerCloudCrop->points.size();
        for (int i = 0; i < plannerCloudCropSize; i++)
        {
            float x   = plannerCloudCrop->points[i].x / pathScale;
            float y   = plannerCloudCrop->points[i].y / pathScale;
            float z   = plannerCloudCrop->points[i].z / pathScale;
            float h   = plannerCloudCrop->points[i].intensity;
            float dis = std::sqrt (x * x + y * y);

            if (dis < pathRange / pathScale
                && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle)
            {
                for (int rotDir = 0; rotDir < 36; rotDir++)
                {
                    float rotAng  = (10.0 * rotDir - 180.0) * M_PI / 180;
                    float angDiff = fabs (joyDir - (10.0 * rotDir - 180.0));
                    if (angDiff > 180.0) angDiff = 360.0 - angDiff;
                    if ((angDiff > dirThre && !dirToVehicle)
                        || (fabs (10.0 * rotDir - 180.0) > dirThre && fabs (joyDir) <= 90.0 && dirToVehicle)
                        || ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs (joyDir) > 90.0
                            && dirToVehicle))
                    {
                        continue;
                    }

                    tf2::Vector3 rotated = transformPointWithYaw (x, y, z, -rotAng, 1.0);
                    float x2             = rotated.x();
                    float y2             = rotated.y();

                    float scaleY =
                        x2 / gridVoxelOffsetX
                        + searchRadius / gridVoxelOffsetY * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

                    int indX = int ((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
                    int indY = int ((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
                    if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY)
                    {
                        int ind                   = gridVoxelNumY * indX + indY;
                        int blockedPathByVoxelNum = correspondences[ind].size();
                        for (int j = 0; j < blockedPathByVoxelNum; j++)
                        {
                            if (h > obstacleHeightThre || !useTerrainAnalysis)
                            {
                                clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                            }
                            else
                            {
                                if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h
                                    && h > groundHeightThre)
                                {
                                    pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                                }
                            }
                        }
                    }
                }
            }

            if (dis < diameter / pathScale
                && (fabs (x) > vehicleLength / pathScale / 2.0 || fabs (y) > vehicleWidth / pathScale / 2.0)
                && (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle)
            {
                float angObs = atan2 (y, x) * 180.0 / M_PI;
                if (angObs > 0)
                {
                    if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
                    if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
                }
                else
                {
                    if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
                    if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
                }
            }
        }

        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;

        for (int i = 0; i < 36 * pathNum; i++)
        {
            int rotDir    = int (i / pathNum);
            float angDiff = fabs (joyDir - (10.0 * rotDir - 180.0));
            if (angDiff > 180.0)
            {
                angDiff = 360.0 - angDiff;
            }
            if ((angDiff > dirThre && !dirToVehicle)
                || (fabs (10.0 * rotDir - 180.0) > dirThre && fabs (joyDir) <= 90.0 && dirToVehicle)
                || ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs (joyDir) > 90.0
                    && dirToVehicle))
            {
                continue;
            }

            if (clearPathList[i] < pointPerPathThre)
            {
                float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
                if (penaltyScore < costScore) penaltyScore = costScore;

                float dirDiff = fabs (joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0));
                if (dirDiff > 360.0)
                {
                    dirDiff -= 360.0;
                }
                if (dirDiff > 180.0)
                {
                    dirDiff = 360.0 - dirDiff;
                }

                float rotDirW;
                if (rotDir < 18)
                    rotDirW = fabs (fabs (rotDir - 9) + 1);
                else
                    rotDirW = fabs (fabs (rotDir - 27) + 1);
                float score = (1 - sqrt (sqrt (dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW
                              * penaltyScore;
                if (score > 0)
                {
                    clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
                }
            }
        }

        float maxScore      = 0;
        int selectedGroupID = -1;
        for (int i = 0; i < 36 * groupNum; i++)
        {
            int rotDir   = int (i / groupNum);
            float rotAng = (10.0 * rotDir - 180.0) * M_PI / 180;
            float rotDeg = 10.0 * rotDir;
            if (rotDeg > 180.0) rotDeg -= 360.0;
            if (maxScore < clearPathPerGroupScore[i]
                && ((rotAng * 180.0 / M_PI > minObsAngCW && rotAng * 180.0 / M_PI < minObsAngCCW)
                    || (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle))
            {
                maxScore        = clearPathPerGroupScore[i];
                selectedGroupID = i;
            }
        }

        if (selectedGroupID >= 0)
        {
            int rotDir             = int (selectedGroupID / groupNum);
            float rotAng           = (10.0 * rotDir - 180.0) * M_PI / 180;

            selectedGroupID        = selectedGroupID % groupNum;
            int selectedPathLength = startPaths[selectedGroupID]->points.size();
            nav_msgs::msg::Path path;
            path.poses.resize (selectedPathLength);
            for (int i = 0; i < selectedPathLength; i++)
            {
                float x   = startPaths[selectedGroupID]->points[i].x;
                float y   = startPaths[selectedGroupID]->points[i].y;
                float z   = startPaths[selectedGroupID]->points[i].z;
                float dis = sqrt (x * x + y * y);

                if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale)
                {
                    tf2::Vector3 rotated          = transformPointWithYaw (x, y, z, rotAng, pathScale);
                    path.poses[i].pose.position.x = rotated.x();
                    path.poses[i].pose.position.y = rotated.y();
                    path.poses[i].pose.position.z = rotated.z();
                }
                else
                {
                    path.poses.resize (i);
                    break;
                }
            }

            path.header.stamp    = lidar_msg_in->header.stamp;
            path.header.frame_id = frameIdvehicle;
            pubPath->publish (path);

#if FALCO_LOCAL_PLANNER_PLOTPATHSET == 1
            freePaths->clear();
            for (int i = 0; i < 36 * pathNum; i++)
            {
                int rotDir   = int (i / pathNum);
                float rotAng = (10.0 * rotDir - 180.0) * M_PI / 180;
                float rotDeg = 10.0 * rotDir;
                if (rotDeg > 180.0) rotDeg -= 360.0;
                float angDiff = fabs (joyDir - (10.0 * rotDir - 180.0));
                if (angDiff > 180.0)
                {
                    angDiff = 360.0 - angDiff;
                }
                if ((angDiff > dirThre && !dirToVehicle)
                    || (fabs (10.0 * rotDir - 180.0) > dirThre && fabs (joyDir) <= 90.0 && dirToVehicle)
                    || ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs (joyDir) > 90.0
                        && dirToVehicle)
                    || !(
                        (rotAng * 180.0 / M_PI > minObsAngCW && rotAng * 180.0 / M_PI < minObsAngCCW)
                        || (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle))
                {
                    continue;
                }

                if (clearPathList[i] < pointPerPathThre)
                {
                    int freePathLength = paths[i % pathNum]->points.size();
                    for (int j = 0; j < freePathLength; j++)
                    {
                        point     = paths[i % pathNum]->points[j];

                        float x   = point.x;
                        float y   = point.y;
                        float z   = point.z;

                        float dis = std::sqrt (x * x + y * y);
                        if (dis <= pathRange / pathScale
                            && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal))
                        {
                            tf2::Vector3 rotated = transformPointWithYaw (x, y, z, rotAng, pathScale);

                            point.x              = rotated.x();
                            point.y              = rotated.y();
                            point.z              = rotated.z();
                            point.intensity      = 1.0;

                            freePaths->push_back (point);
                        }
                    }
                }
            }

            sensor_msgs::msg::PointCloud2 freePaths2;
            pcl::toROSMsg (*freePaths, freePaths2);
            freePaths2.header.stamp    = lidar_msg_in->header.stamp;
            freePaths2.header.frame_id = frameIdvehicle;
            pubFreePaths->publish (freePaths2);
#endif
        }

        if (selectedGroupID < 0)
        {
            if (pathScale >= minPathScale + pathScaleStep)
            {
                pathScale -= pathScaleStep;
                pathRange  = adjacentRange * pathScale / defPathScale;
            }
            else
            {
                pathRange -= pathRangeStep;
            }
        }
        else
        {
            pathFound = true;
            break;
        }
    }
    pathScale = defPathScale;

    if (!pathFound)
    {
        nav_msgs::msg::Path path;
        path.poses.resize (1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp    = lidar_msg_in->header.stamp;
        path.header.frame_id = frameIdvehicle;
        pubPath->publish (path);

#if FALCO_LOCAL_PLANNER_PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::msg::PointCloud2 freePaths2;
        pcl::toROSMsg (*freePaths, freePaths2);
        freePaths2.header.stamp    = lidar_msg_in->header.stamp;
        freePaths2.header.frame_id = frameIdvehicle;
        pubFreePaths->publish (freePaths2);
#endif
    }
}


void LocalPlanner::goalHandler (const geometry_msgs::msg::PointStamped::SharedPtr newGoal)
{
    goal = *newGoal;
}

void LocalPlanner::joyDirCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    joyDir = msg->data;    
}

int LocalPlanner::readPlyHeader (FILE* filePtr)
{
    char str[50];
    int val, pointNum;
    std::string strCur, strLast;
    while (strCur != "end_header")
    {
        val = fscanf (filePtr, "%s", str);
        if (val != 1)
        {
            RCLCPP_ERROR (get_logger(), "Error reading input files, exit.");
            exit (1);
        }

        strLast = strCur;
        strCur  = std::string (str);

        if (strCur == "vertex" && strLast == "element")
        {
            val = fscanf (filePtr, "%d", &pointNum);
            if (val != 1)
            {
                RCLCPP_ERROR (get_logger(), "Error reading input files, exit.");
                exit (1);
            }
        }
    }

    return pointNum;
}
void LocalPlanner::readStartPaths ()
{
    std::string fileName = pathFolder + "/startPaths.ply";

    FILE* filePtr   = fopen (fileName.c_str(), "r");
    if (filePtr == NULL)
    {
        RCLCPP_ERROR (get_logger(), "Cannot read input files, exit.");
        exit (1);
    }

    int pointNum = readPlyHeader (filePtr);

    pcl::PointXYZ point;
    int val1, val2, val3, val4, groupID;
    for (int i = 0; i < pointNum; i++)
    {
        val1 = fscanf (filePtr, "%f", &point.x);
        val2 = fscanf (filePtr, "%f", &point.y);
        val3 = fscanf (filePtr, "%f", &point.z);
        val4 = fscanf (filePtr, "%d", &groupID);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1)
        {
            RCLCPP_ERROR (get_logger(), "Error reading input files, exit.");
            exit (1);
        }

        if (groupID >= 0 && groupID < groupNum)
        {
            startPaths[groupID]->push_back (point);
        }
    }

    fclose (filePtr);
}

#if FALCO_LOCAL_PLANNER_PLOTPATHSET == 1
void LocalPlanner::readPaths ()
{
    std::string fileName = pathFolder + "/paths.ply";

    FILE* filePtr   = fopen (fileName.c_str(), "r");
    if (filePtr == NULL)
    {
        RCLCPP_ERROR (get_logger(), "Cannot read input files, exit.");
        exit (1);
    }

    int pointNum = readPlyHeader (filePtr);

    pcl::PointXYZI point;
    int pointSkipNum   = 30;
    int pointSkipCount = 0;
    int val1, val2, val3, val4, val5, pathID;
    for (int i = 0; i < pointNum; i++)
    {
        val1 = fscanf (filePtr, "%f", &point.x);
        val2 = fscanf (filePtr, "%f", &point.y);
        val3 = fscanf (filePtr, "%f", &point.z);
        val4 = fscanf (filePtr, "%d", &pathID);
        val5 = fscanf (filePtr, "%f", &point.intensity);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1)
        {
            RCLCPP_ERROR (get_logger(), "Error reading input files, exit.");
            exit (1);
        }

        if (pathID >= 0 && pathID < pathNum)
        {
            pointSkipCount++;
            if (pointSkipCount > pointSkipNum)
            {
                paths[pathID]->push_back (point);
                pointSkipCount = 0;
            }
        }
    }

    fclose (filePtr);
}
#endif
void LocalPlanner::readPathList ()
{
    std::string fileName = pathFolder + "/pathList.ply";

    FILE* filePtr   = fopen (fileName.c_str(), "r");
    if (filePtr == NULL)
    {
        RCLCPP_ERROR (get_logger(), "Cannot read input files, exit.");
        exit (1);
    }

    if (pathNum != readPlyHeader (filePtr))
    {
        RCLCPP_ERROR (get_logger(), "Incorrect path number, exit.");
        exit (1);
    }

    int val1, val2, val3, val4, val5, pathID, groupID;
    float endX, endY, endZ;
    for (int i = 0; i < pathNum; i++)
    {
        val1 = fscanf (filePtr, "%f", &endX);
        val2 = fscanf (filePtr, "%f", &endY);
        val3 = fscanf (filePtr, "%f", &endZ);
        val4 = fscanf (filePtr, "%d", &pathID);
        val5 = fscanf (filePtr, "%d", &groupID);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1)
        {
            RCLCPP_ERROR (get_logger(), "Error reading input files, exit.");
            exit (1);
        }

        if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum)
        {
            pathList[pathID]       = groupID;
            endDirPathList[pathID] = 2.0 * atan2 (endY, endX) * 180 / M_PI;
        }
    }

    fclose (filePtr);
}
void LocalPlanner::readCorrespondences ()
{
    std::string fileName = pathFolder + "/correspondences.txt";

    FILE* filePtr   = fopen (fileName.c_str(), "r");
    if (filePtr == NULL)
    {
        RCLCPP_ERROR (get_logger(), "Cannot read input files, exit.");
        exit (1);
    }

    int val1, gridVoxelID, pathID;
    for (int i = 0; i < gridVoxelNum; i++)
    {
        val1 = fscanf (filePtr, "%d", &gridVoxelID);
        if (val1 != 1)
        {
            RCLCPP_ERROR (get_logger(), "Error reading input files, exit.");
            exit (1);
        }

        while (1)
        {
            val1 = fscanf (filePtr, "%d", &pathID);
            if (val1 != 1)
            {
                RCLCPP_ERROR (get_logger(), "Error reading input files, exit.");
                exit (1);
            }

            if (pathID != -1)
            {
                if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum)
                {
                    correspondences[gridVoxelID].push_back (pathID);
                }
            }
            else
            {
                break;
            }
        }
    }

    fclose (filePtr);
}

tf2::Vector3 LocalPlanner::transformPointWithYaw (float x, float y, float z, float yaw, float scale)
{
    tf2::Quaternion q;
    q.setRPY (0, 0, yaw); // yaw rotation
    tf2::Transform transform (q, tf2::Vector3 (0, 0, 0));

    tf2::Vector3 point (x, y, z);
    tf2::Vector3 rotated  = transform * point;

    // Apply scaling
    rotated              *= scale;

    return rotated;
}

