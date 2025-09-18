from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sensor_offset_x = LaunchConfiguration('sensorOffsetX')
    sensor_offset_y = LaunchConfiguration('sensorOffsetY')
    two_way_drive = LaunchConfiguration('twoWayDrive')
    max_speed = LaunchConfiguration('maxSpeed')
    autonomy_mode = LaunchConfiguration('autonomyMode')
    autonomy_speed = LaunchConfiguration('autonomySpeed')
    joy_to_speed_delay = LaunchConfiguration('joyToSpeedDelay')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('sensorOffsetX', default_value='0.0'),
        DeclareLaunchArgument('sensorOffsetY', default_value='0.0'),
        DeclareLaunchArgument('twoWayDrive', default_value='true'),
        DeclareLaunchArgument('maxSpeed', default_value='1.0'),
        DeclareLaunchArgument('autonomyMode', default_value='true'),
        DeclareLaunchArgument('autonomySpeed', default_value='1.0'),
        DeclareLaunchArgument('joyToSpeedDelay', default_value='2.0'),

        # Joypackage
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        

        # Node: localPlanner
        Node(
            package='falco_local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            remappings=[
                ('registered_scan','robot/scan/points'), 
                ('state_estimation','robot/odometry'),  
            ],
            parameters=[{
                'pathFolder': os.path.join(
                    get_package_share_directory('falco_local_planner'), 'paths'),
                'vehicleLength': 2.2,
                'vehicleWidth': 1.2,
                'vehicleHeight': 0.5,
                'sensorOffsetX': sensor_offset_x,
                'sensorOffsetY': sensor_offset_y,
                'twoWayDrive': two_way_drive,
                'laserVoxelSize': 0.05,
                'terrainVoxelSize': 0.2,
                'useTerrainAnalysis': False,
                'checkObstacle': True,
                'checkRotObstacle': True,
                'adjacentRange': 3.5,
                'obstacleHeightThre': 0.15,
                'groundHeightThre': 0.1,
                'costHeightThre': 0.1,
                'costScore': 0.02,
                'useCost': False,
                'pointPerPathThre': 2,
                'minRelZ': 0.1,
                'maxRelZ': 5.0,
                'maxSpeed': max_speed,
                'dirWeight': 0.02,
                'dirThre': 90.0,
                'dirToVehicle': False,
                'pathScale': 1.0,
                'minPathScale': 0.75,
                'pathScaleStep': 0.25,
                'pathScaleBySpeed': True,
                'minPathRange': 1.0,
                'pathRangeStep': 0.5,
                'pathRangeBySpeed': True,
                'pathCropByGoal': True,
                'autonomyMode': autonomy_mode,
                'autonomySpeed': autonomy_speed,
                'joyToSpeedDelay': joy_to_speed_delay,
                'joyToCheckObstacleDelay': 5.0,
                'goalClearRange': 0.5,
                'goalX': 0.0,
                'goalY': 0.0,
                'odom': "robot/odom",
                'vehicle': "robot",
                'use_sim_time': True,
            }]
        ),

        # Node: pathFollower
        Node(
            package='falco_local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            remappings=[                
                ('state_estimation','robot/odometry'),  
            ],
            parameters=[{
                'sensorOffsetX': sensor_offset_x,
                'sensorOffsetY': sensor_offset_y,
                'pubSkipNum': 1,
                'twoWayDrive': two_way_drive,
                'lookAheadDis': 0.5,
                'yawRateGain': 7.5,
                'stopYawRateGain': 7.5,
                'maxYawRate': 45.0,
                'maxSpeed': max_speed,
                'maxAccel': 1.0,
                'switchTimeThre': 1.0,
                'dirDiffThre': 0.1, 
                'stopDisThre': 0.2,
                'slowDwnDisThre': 1.0,
                'useInclRateToSlow': False,
                'inclRateThre': 120.0,
                'slowRate1': 0.25,
                'slowRate2': 0.5,
                'slowTime1': 2.0,
                'slowTime2': 2.0,
                'useInclToStop': False,
                'inclThre': 45.0,
                'stopTime': 5.0,
                'noRotAtStop': False,
                'noRotAtGoal': True,
                'autonomyMode': autonomy_mode,
                'autonomySpeed': autonomy_speed,
                'joyToSpeedDelay': joy_to_speed_delay,
                'vehicle': "robot",
                'use_sim_time': True,
            }]
        ),
        # Node: pathFollower
        Node(
            package='falco_local_planner',
            executable='joystick_interface',
            name='joystick_interface',
            output='screen',            
            parameters=[{                
                'twoWayDrive': two_way_drive,                
                'noRotAtGoal': True,                
                'use_sim_time': True,
            }]
        ),

        
    ])
