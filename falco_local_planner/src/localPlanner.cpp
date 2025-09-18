#include "localPlanner.hpp"

LocalPlanner::LocalPlanner() : Node("localPlanner")
{
    // Initialize tf2 buffer and listener for transform management
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare parameters with default values
    pathFolder              = this->declare_parameter<std::string>("pathFolder", "");
    vehicleLength           = this->declare_parameter<double>("vehicleLength", 0.6);
    vehicleWidth            = this->declare_parameter<double>("vehicleWidth", 0.6);
    vehicleHeight           = this->declare_parameter<double>("vehicleHeight", 0.5);
    sensorOffsetX           = this->declare_parameter<double>("sensorOffsetX", 0.0);
    sensorOffsetY           = this->declare_parameter<double>("sensorOffsetY", 0.0);
    twoWayDrive             = this->declare_parameter<bool>("twoWayDrive", true);
    laserVoxelSize          = this->declare_parameter<double>("laserVoxelSize", 0.05);
    terrainVoxelSize        = this->declare_parameter<double>("terrainVoxelSize", 0.2);
    useTerrainAnalysis      = this->declare_parameter<bool>("useTerrainAnalysis", false);
    checkObstacle           = this->declare_parameter<bool>("checkObstacle", true);
    checkRotObstacle        = this->declare_parameter<bool>("checkRotObstacle", false);
    adjacentRange           = this->declare_parameter<double>("adjacentRange", 3.5);
    obstacleHeightThre      = this->declare_parameter<double>("obstacleHeightThre", 0.2);
    groundHeightThre        = this->declare_parameter<double>("groundHeightThre", 0.1);
    costHeightThre          = this->declare_parameter<double>("costHeightThre", 0.1);
    costScore               = this->declare_parameter<double>("costScore", 0.02);
    useCost                 = this->declare_parameter<bool>("useCost", false);
    pointPerPathThre        = this->declare_parameter<int>("pointPerPathThre", 2);
    minRelZ                 = this->declare_parameter<double>("minRelZ", -0.5);
    maxRelZ                 = this->declare_parameter<double>("maxRelZ", 0.25);
    maxSpeed                = this->declare_parameter<double>("maxSpeed", 1.0);
    dirWeight               = this->declare_parameter<double>("dirWeight", 0.02);
    dirThre                 = this->declare_parameter<double>("dirThre", 90.0);
    dirToVehicle            = this->declare_parameter<bool>("dirToVehicle", false);
    pathScale               = this->declare_parameter<double>("pathScale", 1.0);
    minPathScale            = this->declare_parameter<double>("minPathScale", 0.75);
    pathScaleStep           = this->declare_parameter<double>("pathScaleStep", 0.25);
    pathScaleBySpeed        = this->declare_parameter<bool>("pathScaleBySpeed", true);
    minPathRange            = this->declare_parameter<double>("minPathRange", 1.0);
    pathRangeStep           = this->declare_parameter<double>("pathRangeStep", 0.5);
    pathRangeBySpeed        = this->declare_parameter<bool>("pathRangeBySpeed", true);
    pathCropByGoal          = this->declare_parameter<bool>("pathCropByGoal", true);
    autonomyMode            = this->declare_parameter<bool>("autonomyMode", false);
    autonomySpeed           = this->declare_parameter<double>("autonomySpeed", 1.0);
    joyToSpeedDelay         = this->declare_parameter<double>("joyToSpeedDelay", 2.0);
    joyToCheckObstacleDelay = this->declare_parameter<double>("joyToCheckObstacleDelay", 5.0);
    goalClearRange          = this->declare_parameter<double>("goalClearRange", 0.5);
    goalX                   = this->declare_parameter<double>("goalX", 0.0);
    goalY                   = this->declare_parameter<double>("goalY", 0.0);
    odom                    = this->declare_parameter<std::string>("odom", "robot/odom");
    vehicle                 = this->declare_parameter<std::string>("vehicle", "robot");

    

    // Initialize point clouds
    laserCloud        = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudCrop    = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudDwz     = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloud      = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudCrop  = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudDwz   = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    plannerCloud      = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    plannerCloudCrop  = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    boundaryCloud     = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    addedObstacles    = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    freePaths         = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    for (int i = 0; i < laserCloudStackNum; i++)
        laserCloudStack[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    for (int i = 0; i < groupNum; i++)
        startPaths[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
#if PLOTPATHSET == 1
    for (int i = 0; i < pathNum; i++)
        paths[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
#endif

    // Initialize filters
    laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
    terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

    // Create subscribers
    subOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "/state_estimation", 5, std::bind(&LocalPlanner::odometryHandler, this, std::placeholders::_1));
    subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/registered_scan", 5, std::bind(&LocalPlanner::laserCloudHandler, this, std::placeholders::_1));
    subTerrainCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/terrain_map", 5, std::bind(&LocalPlanner::terrainCloudHandler, this, std::placeholders::_1));
    subJoystick = this->create_subscription<falco_local_planner::msg::JoystickData>(
        "/joystick_data", 10, std::bind(&LocalPlanner::joystickCallback, this, std::placeholders::_1));
    subGoal = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/way_point", 5, std::bind(&LocalPlanner::goalHandler, this, std::placeholders::_1));
    subSpeed = this->create_subscription<std_msgs::msg::Float32>(
        "/speed", 5, std::bind(&LocalPlanner::speedHandler, this, std::placeholders::_1));
    subBoundary = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/navigation_boundary", 5, std::bind(&LocalPlanner::boundaryHandler, this, std::placeholders::_1));
    subAddedObstacles = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/added_obstacles", 5, std::bind(&LocalPlanner::addedObstaclesHandler, this, std::placeholders::_1));
    subCheckObstacle = this->create_subscription<std_msgs::msg::Bool>(
        "/check_obstacle", 5, std::bind(&LocalPlanner::checkObstacleHandler, this, std::placeholders::_1));

    // Create publishers
    pubPath = this->create_publisher<nav_msgs::msg::Path>("/path", 5);
    laserCloudDwzPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_dwz", 10);
    plannerCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/planner_cloud_crop", 2);

#if PLOTPATHSET == 1
    pubFreePaths = this->create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", 2);
#endif

    // Read path files
    RCLCPP_INFO(this->get_logger(), "Reading path files.");
    if (autonomyMode) {
        joySpeed = autonomySpeed / maxSpeed;
        if (joySpeed < 0) joySpeed = 0;
        else if (joySpeed > 1.0) joySpeed = 1.0;
    }
    readStartPaths();
#if PLOTPATHSET == 1
    readPaths();
#endif
    readPathList();
    readCorrespondences();
    RCLCPP_INFO(this->get_logger(), "Initialization complete.");
}

// Callback & Utility Methods
void LocalPlanner::odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom) 
{ odomTime = rclcpp::Time(odom->header.stamp).seconds();

    double roll, pitch, yaw;
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odom->pose.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
    vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
    vehicleZ = odom->pose.pose.position.z;
}
void LocalPlanner::laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg_in) 
{ sensor_msgs::msg::PointCloud2 laserCloud2; // Raw message, not a shared_ptr

    try {
    
    tf_buffer_->transform(*lidar_msg_in, laserCloud2, odom, tf2::durationFromSec(0.1));
    
    // You can now use laserCloud2
    } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
    }


    if (!useTerrainAnalysis) {
    laserCloud->clear();
    pcl::fromROSMsg(laserCloud2, *laserCloud);
    pcl::PointXYZI point;
    float halfLength = vehicleLength / 2.0;
    float halfWidth  = vehicleWidth / 2.0;
    float halfHeight = vehicleHeight / 2.0;

    float minRange = std::sqrt(
        halfLength * halfLength +
        halfWidth  * halfWidth +
        halfHeight * halfHeight
    );



    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
        point = laserCloud->points[i];

        float pointX = point.x;
        float pointY = point.y;
        float pointZ = point.z;

        float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
        if (dis < adjacentRange && dis > minRange) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        laserCloudCrop->push_back(point);
        }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);

     sensor_msgs::msg::PointCloud2 laserCloudDwzMsg;
      pcl::toROSMsg(*laserCloudDwz, laserCloudDwzMsg);     
      laserCloudDwzMsg.header.stamp = rclcpp::Time(odomTime * 1e9);
      laserCloudDwzMsg.header.frame_id = odom;
      laserCloudDwzPub->publish(laserCloudDwzMsg);


    newLaserCloud = true;
    } 
}
void LocalPlanner::terrainCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr terrainCloud2) 
{ if (useTerrainAnalysis) {
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];

        float pointX = point.x;
        float pointY = point.y;
        float pointZ = point.z;

        float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
        if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point);
        }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);

    newTerrainCloud = true;
    }
}
void LocalPlanner::joystickCallback(const falco_local_planner::msg::JoystickData::SharedPtr msg)
{
    joyTime = msg->joy_time;
    joySpeed = msg->joy_speed;
    joyDir = msg->joy_dir;
    autonomyMode = msg->autonomy_mode;
    checkObstacle = msg->check_obstacle;
}
void LocalPlanner::goalHandler(const geometry_msgs::msg::PointStamped::SharedPtr goal) 
{   
    goalX = goal->point.x;
    goalY = goal->point.y;
    goalZ = goal->point.z;
    
}
void LocalPlanner::speedHandler(const std_msgs::msg::Float32::SharedPtr speed) 
{  
    double speedTime = this->now().seconds();

    if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
    }
}
void LocalPlanner::boundaryHandler(const geometry_msgs::msg::PolygonStamped::SharedPtr boundary) 
{ 
    boundaryCloud->clear();
    pcl::PointXYZI point, point1, point2;
    int boundarySize = boundary->polygon.points.size();

    if (boundarySize >= 1) {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
    }

    for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

    if (point1.z == point2.z) {
        float disX = point1.x - point2.x;
        float disY = point1.y - point2.y;
        float dis = sqrt(disX * disX + disY * disY);

        int pointNum = int(dis / terrainVoxelSize) + 1;
        for (int pointID = 0; pointID < pointNum; pointID++) {
        point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
        point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
        point.z = 0;
        point.intensity = 100.0;

        for (int j = 0; j < pointPerPathThre; j++) {
            boundaryCloud->push_back(point);
        }
        }
    }
    } 
}
void LocalPlanner::addedObstaclesHandler(const sensor_msgs::msg::PointCloud2::SharedPtr addedObstacles2) 
{ 
    addedObstacles->clear();
    pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

    int addedObstaclesSize = addedObstacles->points.size();
    for (int i = 0; i < addedObstaclesSize; i++) {
        addedObstacles->points[i].intensity = 200.0;
    }  
}
void LocalPlanner::checkObstacleHandler(const std_msgs::msg::Bool::SharedPtr checkObs) 
{ 
    double checkObsTime = this->now().seconds();

    if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay) {
    checkObstacle = checkObs->data;
    }
}

int LocalPlanner::readPlyHeader(FILE *filePtr) 
{ 
    char str[50];
    int val, pointNum;
    string strCur, strLast;
    while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
        val = fscanf(filePtr, "%d", &pointNum);
        if (val != 1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading input files, exit.");
        exit(1);
        }
    }
    }

    return pointNum;  
}
void LocalPlanner::readStartPaths() 
{ 
    string fileName = pathFolder + "/startPaths.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
    RCLCPP_ERROR(this->get_logger(), "Cannot read input files, exit.");
    exit(1);
    }

    int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZ point;
    int val1, val2, val3, val4, groupID;
    for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
        startPaths[groupID]->push_back(point);
    }
    }

    fclose(filePtr); 
}
#if PLOTPATHSET == 1
void LocalPlanner::readPaths() 
{ 
    string fileName = pathFolder + "/paths.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
    RCLCPP_ERROR(this->get_logger(), "Cannot read input files, exit.");
    exit(1);
    }

    int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZI point;
    int pointSkipNum = 30;
    int pointSkipCount = 0;
    int val1, val2, val3, val4, val5, pathID;
    for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum) {
        pointSkipCount++;
        if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
        }
    }
    }

    fclose(filePtr);
}
#endif
void LocalPlanner::readPathList() 
{ 
    string fileName = pathFolder + "/pathList.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
    RCLCPP_ERROR(this->get_logger(), "Cannot read input files, exit.");
    exit(1);
    }

    if (pathNum != readPlyHeader(filePtr)) {
    RCLCPP_ERROR(this->get_logger(), "Incorrect path number, exit.");
    exit(1);
    }

    int val1, val2, val3, val4, val5, pathID, groupID;
    float endX, endY, endZ;
    for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
        pathList[pathID] = groupID;
        endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
    }
    }

    fclose(filePtr); 
}
void LocalPlanner::readCorrespondences() 
{ 
    string fileName = pathFolder + "/correspondences.txt";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
    RCLCPP_ERROR(this->get_logger(), "Cannot read input files, exit.");
    exit(1);
    }

    int val1, gridVoxelID, pathID;
    for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    while (1) {
        val1 = fscanf(filePtr, "%d", &pathID);
        if (val1 != 1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading input files, exit.");
        exit(1);
        }

        if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
            correspondences[gridVoxelID].push_back(pathID);
        }
        } else {
        break;
        }
    }
    }

    fclose(filePtr); 
}

tf2::Vector3 LocalPlanner::transformPointWithYaw(
            float x, float y, float z,
            float yaw, float scale)
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);  // yaw rotation
            tf2::Transform transform(q, tf2::Vector3(0, 0, 0));

            tf2::Vector3 point(x, y, z);
            tf2::Vector3 rotated = transform * point;

            // Apply scaling
            rotated *= scale;

            return rotated;
        }

void LocalPlanner::run() 
{ 
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
    rclcpp::spin_some(this->get_node_base_interface());

    if (newLaserCloud || newTerrainCloud) {
        pcl::PointXYZI point;
        if (newLaserCloud) {
        newLaserCloud = false;

        laserCloudStack[laserCloudCount]->clear();
        *laserCloudStack[laserCloudCount] = *laserCloudDwz;
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

        plannerCloud->clear();
        for (int i = 0; i < laserCloudStackNum; i++) {
            *plannerCloud += *laserCloudStack[i];
        }
        }

        if (newTerrainCloud) {
        newTerrainCloud = false;

        plannerCloud->clear();
        *plannerCloud = *terrainCloudDwz;
        }

        // Convert to ROS PointCloud2
        sensor_msgs::msg::PointCloud2 cloud_msg_in;
        pcl::toROSMsg(*plannerCloud, cloud_msg_in);
        cloud_msg_in.header.frame_id = odom;  // <- source frame
        cloud_msg_in.header.stamp = rclcpp::Time(odomTime * 1e9);  // <- make sure TF at this time is available

        sensor_msgs::msg::PointCloud2 cloud_msg_out;

        try {
            tf_buffer_->transform(cloud_msg_in, cloud_msg_out, vehicle, tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
            return;
        }

        //Convert TF-transformed PointCloud2 into a temporary PCL cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(cloud_msg_out, *transformedCloud);

        // Filter the transformed cloud and store result into plannerCloudCrop
        plannerCloudCrop->clear();
        for (const auto& point : transformedCloud->points) {
            float dis = sqrt(point.x * point.x + point.y * point.y);
            if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
                plannerCloudCrop->push_back(point);  
            }
        }      
       
        sensor_msgs::msg::PointCloud2 plannerCloudMsg;
        pcl::toROSMsg(*plannerCloudCrop, plannerCloudMsg);
        plannerCloudMsg.header.stamp = rclcpp::Time(odomTime * 1e9);
        plannerCloudMsg.header.frame_id = vehicle;  
        plannerCloudPub->publish(plannerCloudMsg);        

        // Helper lambda for transforming and filtering a cloud
        auto transformAndFilter = [&](const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
            // Convert to ROS PointCloud2
            sensor_msgs::msg::PointCloud2 cloud_msg_in;
            pcl::toROSMsg(*inputCloud, cloud_msg_in);
            cloud_msg_in.header.frame_id = odom;  // or actual source frame
            cloud_msg_in.header.stamp =rclcpp::Time(odomTime * 1e9);

            // Transform
            sensor_msgs::msg::PointCloud2 cloud_msg_out;
            try {
                tf_buffer_->transform(cloud_msg_in, cloud_msg_out, vehicle, tf2::durationFromSec(0.1));
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF transform failed for cloud: %s", ex.what());
                return;
            }

            // Convert back to PCL
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(cloud_msg_out, *transformed);

            // Filter and append to plannerCloudCrop
            for (const auto& point : transformed->points) {
                float dis = std::sqrt(point.x * point.x + point.y * point.y);
                if (dis < adjacentRange) {
                    plannerCloudCrop->push_back(point);
                }
            }
        };

        // Call the lambda for each input cloud
        transformAndFilter(boundaryCloud);
        transformAndFilter(addedObstacles);    


        float pathRange = adjacentRange;
        if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;
        if (pathRange < minPathRange) pathRange = minPathRange;
        float relativeGoalDis = adjacentRange;

        if (autonomyMode) {
        // Build the vehicle pose as a TF transform
        tf2::Quaternion q;
        q.setRPY(vehicleRoll, vehiclePitch, vehicleYaw);
        tf2::Transform vehicle_tf(q);
        vehicle_tf.setOrigin(tf2::Vector3(vehicleX, vehicleY, vehicleZ));

        // Build the goal point in world (odom/map) frame
        tf2::Vector3 goal_world(goalX, goalY, goalZ);

        // Compute the goal relative to vehicle frame
        tf2::Vector3 goal_vehicle = vehicle_tf.inverse() * goal_world;

        
        float relativeGoalX = goal_vehicle.x();
        float relativeGoalY = goal_vehicle.y();
        float relativeGoalZ = goal_vehicle.z();  


        relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
        joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;

        if (!twoWayDrive) {
            if (joyDir > 90.0) joyDir = 90.0;
            else if (joyDir < -90.0) joyDir = -90.0;
        }
        }

        bool pathFound = false;
        float defPathScale = pathScale;
        if (pathScaleBySpeed) pathScale = defPathScale * joySpeed;
        if (pathScale < minPathScale) pathScale = minPathScale;

        while (pathScale >= minPathScale && pathRange >= minPathRange) {
        for (int i = 0; i < 36 * pathNum; i++) {
            clearPathList[i] = 0;
            pathPenaltyList[i] = 0;
        }
        for (int i = 0; i < 36 * groupNum; i++) {
            clearPathPerGroupScore[i] = 0;
        }

        float minObsAngCW = -180.0;
        float minObsAngCCW = 180.0;
        float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
        float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;
        int plannerCloudCropSize = plannerCloudCrop->points.size();
        for (int i = 0; i < plannerCloudCropSize; i++) {
            float x = plannerCloudCrop->points[i].x / pathScale;
            float y = plannerCloudCrop->points[i].y / pathScale;
            float z = plannerCloudCrop->points[i].z / pathScale;
            float h = plannerCloudCrop->points[i].intensity;
            float dis = sqrt(x * x + y * y);

            if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle) {
            for (int rotDir = 0; rotDir < 36; rotDir++) {
                float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
                float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
                if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
                }
                if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                    ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                continue;
                }

                tf2::Vector3 rotated = transformPointWithYaw(x, y, z, -rotAng, 1.0);                
                float x2 = rotated.x();
                float y2 = rotated.y();

                float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY 
                            * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

                int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
                int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
                if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                int ind = gridVoxelNumY * indX + indY;
                int blockedPathByVoxelNum = correspondences[ind].size();
                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                    if (h > obstacleHeightThre || !useTerrainAnalysis) {
                    clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                    } else {
                    if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > groundHeightThre) {
                        pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                    }
                    }
                }
                }
            }
            }

            if (dis < diameter / pathScale && (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) && 
                (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle) {
            float angObs = atan2(y, x) * 180.0 / PI;
            if (angObs > 0) {
                if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
                if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
            } else {
                if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
                if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
            }
            }
        }

        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;

        for (int i = 0; i < 36 * pathNum; i++) {
            int rotDir = int(i / pathNum);
            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
            if (angDiff > 180.0) {
            angDiff = 360.0 - angDiff;
            }
            if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
            continue;
            }

            if (clearPathList[i] < pointPerPathThre) {
            float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
            if (penaltyScore < costScore) penaltyScore = costScore;

            float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0));
            if (dirDiff > 360.0) {
                dirDiff -= 360.0;
            }
            if (dirDiff > 180.0) {
                dirDiff = 360.0 - dirDiff;
            }

            float rotDirW;
            if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1);
            else rotDirW = fabs(fabs(rotDir - 27) + 1);
            float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;
            if (score > 0) {
                clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
            }
            }
        }

        float maxScore = 0;
        int selectedGroupID = -1;
        for (int i = 0; i < 36 * groupNum; i++) {
            int rotDir = int(i / groupNum);
            float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
            float rotDeg = 10.0 * rotDir;
            if (rotDeg > 180.0) rotDeg -= 360.0;
            if (maxScore < clearPathPerGroupScore[i] && ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
                (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
            maxScore = clearPathPerGroupScore[i];
            selectedGroupID = i;
            }
        }

        if (selectedGroupID >= 0) {
            int rotDir = int(selectedGroupID / groupNum);
            float rotAng = (10.0 * rotDir - 180.0) * PI / 180;

            selectedGroupID = selectedGroupID % groupNum;
            int selectedPathLength = startPaths[selectedGroupID]->points.size();
            nav_msgs::msg::Path path;
            path.poses.resize(selectedPathLength);
            for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrt(x * x + y * y);

            if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
                tf2::Vector3 rotated = transformPointWithYaw(x, y, z, rotAng, pathScale);
                path.poses[i].pose.position.x = rotated.x();
                path.poses[i].pose.position.y = rotated.y();
                path.poses[i].pose.position.z = rotated.z();
            } else {
                path.poses.resize(i);
                break;
            }
            }

            path.header.stamp = this->now();
            path.header.frame_id = vehicle;
            pubPath->publish(path);

#if PLOTPATHSET == 1
            freePaths->clear();
            for (int i = 0; i < 36 * pathNum; i++) {
            int rotDir = int(i / pathNum);
            float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
            float rotDeg = 10.0 * rotDir;
            if (rotDeg > 180.0) rotDeg -= 360.0;
            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
            if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
            }
            if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle) || 
                !((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
                (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
                continue;
            }            

            if (clearPathList[i] < pointPerPathThre) {
                int freePathLength = paths[i % pathNum]->points.size();
                for (int j = 0; j < freePathLength; j++) {
                point = paths[i % pathNum]->points[j];

                float x = point.x;
                float y = point.y;
                float z = point.z;

                float dis = sqrt(x * x + y * y);
                if (dis <= pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal)) {

                    tf2::Vector3 rotated = transformPointWithYaw(x, y, z, rotAng, pathScale);

                    point.x = rotated.x();
                    point.y = rotated.y();
                    point.z = rotated.z();
                    point.intensity = 1.0;

                    freePaths->push_back(point);
                }
                }
            }
            }

            sensor_msgs::msg::PointCloud2 freePaths2;
            pcl::toROSMsg(*freePaths, freePaths2);
            freePaths2.header.stamp = this->now();
            freePaths2.header.frame_id = vehicle;
            pubFreePaths->publish(freePaths2);
#endif
        }

        if (selectedGroupID < 0) {
            if (pathScale >= minPathScale + pathScaleStep) {
            pathScale -= pathScaleStep;
            pathRange = adjacentRange * pathScale / defPathScale;
            } else {
            pathRange -= pathRangeStep;
            }
        } else {
            pathFound = true;
            break;
        }
        }
        pathScale = defPathScale;

        if (!pathFound) {
        nav_msgs::msg::Path path;
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = this->now();
        path.header.frame_id = vehicle;
        pubPath->publish(path);

#if PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::msg::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = this->now();
        freePaths2.header.frame_id = vehicle;
        pubFreePaths->publish(freePaths2);
#endif
        }
    }

    rate.sleep();
    }  
}
