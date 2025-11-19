import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_navigation')
    pkg_path = Path(pkg_share)

    declare_map = DeclareLaunchArgument(
        'map', default_value=str(pkg_path / 'config' / 'my_map.yaml'), description='Path to map yaml'
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=str(pkg_path / 'config' / 'nav2_params.yaml'), description='Path to Nav2 params'
    )
    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='True')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'use_namespace': 'True',
            'autostart': 'True'
        }.items()
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(pkg_path / 'rviz' / 'nav2_config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_namespace,
        declare_use_sim_time,
        declare_use_rviz,
        nav2_bringup,
        rviz_node
    ])
