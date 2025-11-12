# navigation.launch.py (Version 3 

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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def validate_files(context, *args, **kwargs):
    """Validate that required files exist before launching."""
    map_file = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    
    if not os.path.isfile(map_file):
        raise FileNotFoundError(f"Map file not found: {map_file}")
    
    if not os.path.isfile(params_file):
        raise FileNotFoundError(f"Params file not found: {params_file}")
    
    return []


def generate_launch_description():
    """Generate the launch description for the navigation system."""
    
    # Get package directory
    pkg_share = get_package_share_directory('rover_navigation')
    pkg_path = Path(pkg_share)

    #                              Launch Arguments                                      

    
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=str(pkg_path / 'my_map.yaml'), # Corrected path
        description='Full path to the map YAML file'
    )
    
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=str(pkg_path / 'config' / 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time if true'
    )
    
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz2 for visualization'
    )
    
    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=str(pkg_path / 'rviz' / 'nav2_config.rviz'),
        description='Path to RViz configuration file'
    )
    
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )
    
    declare_base_frame_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='base_link',
        description='Base frame ID for the robot'
    )
    
    declare_laser_frame_arg = DeclareLaunchArgument(
        'laser_frame_id',
        default_value='laser_frame',
        description='Laser scanner frame ID'
    )
    
    #                           Launch Configurations                                    
    
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    namespace = LaunchConfiguration('namespace')
    base_frame_id = LaunchConfiguration('base_frame_id')
    laser_frame_id = LaunchConfiguration('laser_frame_id')


    #                                Static Transforms                                   

    # Transform from base_link to laser_frame
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '--x', '0.1',
            '--y', '0.0',
            '--z', '0.2',
            '--frame-id', base_frame_id,
            '--child-frame-id', laser_frame_id
        ],
        output='screen'
    )
    


    #                               Nav2 Bringup                                        

    
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
    )
    
  
    #                          Group with Namespace                                    

    
    namespaced_group = GroupAction([
        PushRosNamespace(namespace),
        static_tf_base_to_laser,
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
        declare_namespace_arg,
        declare_base_frame_arg,
        declare_laser_frame_arg,
        
        # Validate files exist (optional but good practice)
        OpaqueFunction(function=validate_files),
        
        # Launch all nodes
        namespaced_group,
    ])