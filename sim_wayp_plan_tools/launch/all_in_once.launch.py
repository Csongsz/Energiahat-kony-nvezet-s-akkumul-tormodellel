from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'gazebo_bridge.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'rviz1.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'waypoint_loader.launch.py'])
        ),

        TimerAction(
            period=4.0,
            actions=[
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'waypoint_to_target.launch.py']),
            launch_arguments={'path_topic': 'visualization_path'}.items() 
        ),
    ]
),        

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='wayp_plan_tools',
                    executable='single_goal_pursuit',
                    namespace='sim1',
                    output='screen',
                    parameters=[
                        {"cmd_topic": "/model/vehicle_blue/cmd_vel"}, 
                        {"wheelbase": 1.0},
                        {"waypoint_topic": "targetpoints"},
                    ],
                ),
            ]
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='wayp_plan_tools',
                    executable='energy_optimizer_node',
                    name='energy_optimizer',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
            ]
        ),
    ])