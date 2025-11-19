import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_navigation')
    pkg_path = Path(pkg_share)
    
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=str(pkg_path / 'config' / 'slam_online_async.yaml'),
        description='Full path to the SLAM Toolbox parameters file'
    )
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='True')
    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Namespace')
    declare_scan_topic = DeclareLaunchArgument('scan_topic', default_value='/scan', description='Laser scan topic')

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace')
    scan_topic = LaunchConfiguration('scan_topic')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', scan_topic)
        ]
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(pkg_path / 'rviz' / 'mapping_config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    namespaced_group = GroupAction([
        PushRosNamespace(namespace),
        slam_toolbox_node,
    ])

    return LaunchDescription([
        declare_slam_params,
        declare_use_sim_time,
        declare_use_rviz,
        declare_namespace,
        declare_scan_topic,
        namespaced_group,
        rviz_node
    ])
