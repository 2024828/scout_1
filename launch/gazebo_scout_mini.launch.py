import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    yaw_angle = LaunchConfiguration('yaw')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # Gazebo 启动
        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                PathJoinSubstitution([
                    FindPackageShare('scout'),
                    'worlds',
                    'home.sdf'
                ])
            ],
            output='screen'
        ),

        # 发布 robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    'xacro',
                    PathJoinSubstitution([
                        FindPackageShare('scout'),
                        'urdf',
                        'scout_mini.urdf.xacro'
                    ])
                ]),
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        # 启动 RViz，加载配置
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('scout'),
                    'rviz',
                    'scout_nav.rviz'
                ])
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # 延迟插入机器人到 Gazebo
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_entity',
                    arguments=[
                        '-name', 'scout_mini',
                        '-topic', 'robot_description',
                        '-x', x_pos,
                        '-y', y_pos,
                        '-z', '0.1',
                        '-Y', yaw_angle
                    ],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                )
            ]
        )
    ])
