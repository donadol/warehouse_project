import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get the use_sim_time value
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # Get package directories
    path_planner_dir = get_package_share_directory('path_planner_server')

    # Select configuration files based on use_sim_time
    config_suffix = 'sim' if use_sim_time == 'True' else 'real'
    planner_yaml = os.path.join(path_planner_dir, 'config', f'planner_{config_suffix}.yaml')
    controller_yaml = os.path.join(path_planner_dir, 'config', f'controller_{config_suffix}.yaml')
    bt_navigator_yaml = os.path.join(path_planner_dir, 'config', f'bt_navigator_{config_suffix}.yaml')
    recovery_yaml = os.path.join(path_planner_dir, 'config', f'recoveries_{config_suffix}.yaml')
    rviz_config_dir = os.path.join(path_planner_dir, 'rviz', 'pathplanning.rviz')

    # Conditional cmd_vel remapping (only for sim)
    cmd_vel_remapping = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')] if use_sim_time == 'True' else []

    # Planner server node
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml,
                    {'use_sim_time': use_sim_time == 'True'}],
        remappings=cmd_vel_remapping
    )

    # Controller server node
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml,
                    {'use_sim_time': use_sim_time == 'True'}],
        remappings=cmd_vel_remapping
    )

    # BT Navigator node
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml,
                    {'use_sim_time': use_sim_time == 'True'}]
    )

    # Recoveries server node
    recoveries_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        output='screen',
        parameters=[recovery_yaml,
                    {'use_sim_time': use_sim_time == 'True'}],
        remappings=cmd_vel_remapping
    )

    # Lifecycle manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time == 'True'},
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
        parameters=[{'use_sim_time': use_sim_time == 'True'}],
        output='screen'
    )

    return [
        planner_server_node,
        controller_server_node,
        bt_navigator_node,
        recoveries_server_node,
        lifecycle_manager_node,
        rviz_node
    ]

def generate_launch_description():
    # Declare launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation/Gazebo clock if true'
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        OpaqueFunction(function=launch_setup)
    ])
