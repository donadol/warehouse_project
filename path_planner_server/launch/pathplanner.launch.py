import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation/Gazebo clock if true'
    )

    # Get package directories
    path_planner_dir = get_package_share_directory('path_planner_server')

    planner_yaml = os.path.join(path_planner_dir, 'config', 'planner_sim.yaml')
    controller_yaml = os.path.join(path_planner_dir, 'config', 'controller_sim.yaml')
    bt_navigator_yaml = os.path.join(path_planner_dir, 'config', 'bt_navigator_sim.yaml')
    recovery_yaml = os.path.join(path_planner_dir, 'config', 'recoveries_sim.yaml')
    rviz_config_dir = os.path.join(path_planner_dir, 'rviz', 'pathplanning.rviz')

    # Planner server node
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml,
                    {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
    )

    # Controller server node
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml,
                    {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
    )

    # BT Navigator node
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml,
                    {'use_sim_time': use_sim_time}]
    )

    # Recoveries server node
    recoveries_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        output='screen',
        parameters=[recovery_yaml,
                    {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
    )

    # Lifecycle manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'recoveries_server',
                                    'bt_navigator']}]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        planner_server_node,
        controller_server_node,
        bt_navigator_node,
        recoveries_server_node,
        lifecycle_manager_node,
        rviz_node
    ])