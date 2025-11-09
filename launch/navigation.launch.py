"""
ROS2 Navigation Launch File for Rover
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def validate_files(context, *args, **kwargs):
    """Validate that required files exist before launching."""
    map_file = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    
    if not os.path.isfile(map_file):
        return [Node(
            package='ros2',
            executable='echo',
            arguments=[f'ERROR: Map file not found: {map_file}'],
            output='screen'
        )]
    
    if not os.path.isfile(params_file):
        return [Node(
            package='ros2',
            executable='echo',
            arguments=[f'ERROR: Params file not found: {params_file}'],
            output='screen'
        )]
    
    return []


def generate_launch_description():
    """Generate the launch description for the navigation system."""
    
    # Get package directory
    pkg_share = get_package_share_directory('rover_navigation')
    pkg_path = Path(pkg_share)

    
    #                                 LAUNCH ARGUMENTS                                     

    
    # Map configuration
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=str(pkg_path / 'maps' / 'my_map.yaml'),
        description='Full path to the map YAML file'
    )
    
    # Navigation parameters
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=str(pkg_path / 'config' / 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
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
    
    # RViz configuration
    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=str(pkg_path / 'rviz' / 'nav2_config.rviz'),
        description='Path to RViz configuration file'
    )
    
    # Odometry source selection
    declare_odometry_source_arg = DeclareLaunchArgument(
        'odometry_source',
        default_value='robot',
        choices=['laser', 'robot', 'none'],
        description='Odometry source: "laser" for rf2o, "robot" for EKF, "none" to disable'
    )
    
    # Namespace (useful for multi-robot systems)
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes (leave empty for global namespace)'
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
    
    
    #                          Launch Configurations                                    
       
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    odometry_source = LaunchConfiguration('odometry_source')
    namespace = LaunchConfiguration('namespace')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    laser_frame_id = LaunchConfiguration('laser_frame_id')
    scan_topic = LaunchConfiguration('scan_topic')
    

    #                                Static Transforms                                   
    
    # Transform from base_link to laser_frame
    # Adjust these values based on your robot's physical configuration
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '--x', '0.1',      # 10cm forward
            '--y', '0.0',      # centered
            '--z', '0.2',      # 20cm up
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', base_frame_id,
            '--child-frame-id', laser_frame_id
        ],
        output='screen'
    )
    
    
    #                            Odometry Nodes                             
    
    # RF2O Laser-based odometry
    rf2o_odometry_node = Node(
        condition=LaunchConfigurationEquals('odometry_source', 'laser'),
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
            'init_pose_from_topic': '',
            'freq': 10.0,
            'verbose': False,
        }],
        remappings=[
            ('/scan', scan_topic),
        ]
    )
    
    # Robot localization (EKF) for sensor fusion
    # Fuses wheel odometry, IMU, and optionally other sensors
    ekf_localization_node = Node(
        condition=LaunchConfigurationEquals('odometry_source', 'robot'),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            str(pkg_path / 'config' / 'ekf.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ]
    )
    
    
    #                                Nav2 Bringup                                        

    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'use_composition': 'False',  # Set to True for better performance
        }.items()
    )
    
    #                               RViz Visualization                                  
    
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

    
    # Group all nodes under a namespace if specified
    namespaced_group = GroupAction([
        PushRosNamespace(namespace),
        static_tf_base_to_laser,
        rf2o_odometry_node,
        ekf_localization_node,
        nav2_bringup,
        rviz_node,
    ])
    

    #                           Build Launch Description                                
   
    
    return LaunchDescription([
        # Declare all arguments
        declare_map_arg,
        declare_params_file_arg,
        declare_use_sim_time_arg,
        declare_use_rviz_arg,
        declare_rviz_config_arg,
        declare_odometry_source_arg,
        declare_namespace_arg,
        declare_base_frame_arg,
        declare_odom_frame_arg,
        declare_laser_frame_arg,
        declare_scan_topic_arg,
        
        # Validate files exist
        OpaqueFunction(function=validate_files),
        
        # Launch all nodes
        namespaced_group,
    ])