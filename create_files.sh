#!/bin/bash

echo "Creating directory structure..."
mkdir -p launch config rviz models

echo "Writing CMakeLists.txt..."
cat << 'EOF' > CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto()
endif()

ament_package()
EOF

echo "Writing package.xml..."
cat << 'EOF' > package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rover_navigation</name>
  <version>0.0.1</version>
  <description>Navigation and Mapping package for Rover</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>ros2launch</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>tf2_ros</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

echo "Writing mapping.launch.py..."
cat << 'EOF' > launch/mapping.launch.py
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
EOF

echo "Writing navigation.launch.py..."
cat << 'EOF' > launch/navigation.launch.py
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
EOF

echo "Writing slam_online_async.yaml..."
cat << 'EOF' > config/slam_online_async.yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.
    stack_size_to_use: 40000000
    enable_interactive_mode: true
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1  
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true 
    loop_match_minimum_chain_size: 10           
    loop_match_maximum_variance_coarse: 3.0  
    loop_match_minimum_response_coarse: 0.35    
    loop_match_minimum_response_fine: 0.45
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1 
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03
EOF

echo "Copying default Nav2 params and performing basic edits..."
if [ -f "/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml" ]; then
    cp /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml config/nav2_params.yaml
    
    # Auto-replace scan topics to /scan
    sed -i 's|topic: scan|topic: /scan|g' config/nav2_params.yaml
    sed -i 's|scan_topic: scan|scan_topic: /scan|g' config/nav2_params.yaml
    
    echo "SUCCESS: nav2_params.yaml copied and topics updated to /scan."
    echo "PLEASE EDIT config/nav2_params.yaml MANUALLY TO SET YOUR ROBOT_RADIUS."
else
    echo "ERROR: Could not find default nav2_params.yaml. Is 'ros-jazzy-nav2-bringup' installed?"
fi

echo "------------------------------------------------"
echo "All files created successfully."
echo "1. Edit config/nav2_params.yaml to set your robot radius."
echo "2. Run: colcon build --symlink-install"
echo "3. Run: source install/setup.bash"
echo "------------------------------------------------"
