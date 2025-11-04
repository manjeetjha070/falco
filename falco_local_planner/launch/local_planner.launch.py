from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():    
    two_way_drive = LaunchConfiguration('twoWayDrive')
    max_speed = LaunchConfiguration('maxSpeed')    
    autonomy_speed = LaunchConfiguration('autonomySpeed')
    joy_to_speed_delay = LaunchConfiguration('joyToSpeedDelay')
    frameIdOdom = LaunchConfiguration('frameIdOdom')
    frameIdvehicle = LaunchConfiguration('frameIdvehicle')

    return LaunchDescription([
        # Declare launch arguments        
        DeclareLaunchArgument('twoWayDrive', default_value='true'),
        DeclareLaunchArgument('maxSpeed', default_value='1.0'),        
        DeclareLaunchArgument('autonomySpeed', default_value='1.0'),
        DeclareLaunchArgument('joyToSpeedDelay', default_value='2.0'),
        DeclareLaunchArgument('frameIdOdom', default_value='robot/odom'),
        DeclareLaunchArgument('frameIdvehicle', default_value='robot'),

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
            executable='local_planner',
            name='local_planner',
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
                'twoWayDrive': two_way_drive,
                'laserVoxelSize': 0.05,
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
                'autonomySpeed': autonomy_speed,
                'joyToSpeedDelay': joy_to_speed_delay,
                'joyToCheckObstacleDelay': 5.0,
                'goalClearRange': 0.5,
                'goalX': 0.0,
                'goalY': 0.0,
                'frameIdOdom': frameIdOdom,
                'frameIdvehicle': frameIdvehicle,
                'use_sim_time': True,
            }]
        ),

        # Node: pathFollower
        Node(
            package='falco_local_planner',
            executable='path_follower',
            name='path_follower',
            output='screen',
            remappings=[                
                ('state_estimation','robot/odometry'),  
            ],
            parameters=[{
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
                'noRotAtGoal': True,                
                'autonomySpeed': autonomy_speed,
                'use_sim_time': True,
            }]
        ),

        # Node: joystick_interface
        Node(
            package='falco_local_planner',
            executable='joystick_interface',
            name='joystick_interface',
            output='screen',
            parameters=[{
                'twoWayDrive': two_way_drive,
                'use_sim_time': True,
            }]            
        ),
       

        
    ])
