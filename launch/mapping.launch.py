# Filename: mapping.launch.py
# Location: ~/ros2_ws/src/rover_navigation/launch/

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    """
    This launch file starts the SLAM process for mapping an environment.
    """
    
    # Get package directory
    pkg_share = get_package_share_directory('rover_navigation')
    pkg_path = Path(pkg_share)
    
    #                              Launch Arguments                                      
    
    # SLAM parameters
    declare_slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=str(pkg_path / 'config' / 'slam_online_async.yaml'),
        description='Full path to the SLAM Toolbox parameters file'
    )
    
    # Simulation time
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        choices=['True', 'False'],
        description='Use simulation time if true'
    )
    
    # Visualization
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        choices=['True', 'False'],
        description='Whether to start RViz2 for visualization'
    )
    
    # RViz configuration for mapping
    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=str(pkg_path / 'rviz' / 'mapping_config.rviz'),
        description='Path to RViz configuration file for mapping'
    )
    
    # Namespace
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )
    
    # Frame IDs
    declare_base_frame_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='base_link',
        description='Base frame ID for the robot'
    )
    
    declare_odom_frame_arg = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom',
        description='Odometry frame ID'
    )
    
    declare_laser_frame_arg = DeclareLaunchArgument(
        'laser_frame_id',
        default_value='laser_frame',
        description='Laser scanner frame ID'
    )
    
    # Laser scan topic
    declare_scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic name for laser scan data'
    )


    #                           Launch Configurations                                   

    
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    namespace = LaunchConfiguration('namespace')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    laser_frame_id = LaunchConfiguration('laser_frame_id')
    scan_topic = LaunchConfiguration('scan_topic')


    #                                Nodes to Launch                                     

    
    # Static Transform Publisher (base_link -> laser_frame)
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '--x', '0.1',
            '--y', '0.0',
            '--z', '0.2',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', base_frame_id,
            '--child-frame-id', laser_frame_id
        ],
        output='screen'
    )
    
    # RF2O Laser-based odometry (used for mapping)
    rf2o_odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': scan_topic,
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': base_frame_id,
            'odom_frame_id': odom_frame_id,
            'freq': 10.0,
        }]
        # NOTE: The redundant remapping was removed here. The 'laser_scan_topic'
        # parameter correctly handles the topic name.
    )
    
    # SLAM Toolbox Node (The Cartographer)
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
            # This remapping is necessary because SLAM Toolbox does not
            # have a parameter to set the scan topic.
            ('/scan', scan_topic),
        ]
    )
    
    # RViz2 Node (for visualization)
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    

    #                            Group with Namespace                                    

    
    namespaced_group = GroupAction([
        PushRosNamespace(namespace),
        static_tf_base_to_laser,
        rf2o_odometry_node,
        slam_toolbox_node,
        rviz_node,
    ])


    #                            Build Launch Description                                

    
    return LaunchDescription([
        # Declare all arguments
        declare_slam_params_file_arg,
        declare_use_sim_time_arg,
        declare_use_rviz_arg,
        declare_rviz_config_arg,
        declare_namespace_arg,
        declare_base_frame_arg,
        declare_odom_frame_arg,
        declare_laser_frame_arg,
        declare_scan_topic_arg,
        
        # Launch all nodes
        namespaced_group,
    ])