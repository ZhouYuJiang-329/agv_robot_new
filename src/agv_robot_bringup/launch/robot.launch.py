"""
Robot bringup launch file
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    bringup_pkg = FindPackageShare('agv_robot_bringup')
    
    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )
    
    # Config files
    robot_description = PathJoinSubstitution([bringup_pkg, 'config', 'robot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([bringup_pkg, 'rviz', 'robot.rviz'])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=LaunchConfiguration('use_sim')
    )
    
    # Task Manager
    task_manager = Node(
        package='agv_robot_task_manager',
        executable='task_manager_node',
        name='task_manager',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim')
        }]
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=LaunchConfiguration('use_rviz')
    )
    
    return LaunchDescription([
        use_sim_arg,
        use_rviz_arg,
        robot_state_publisher,
        joint_state_publisher,
        task_manager,
        rviz,
    ])
