import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    map_file_arg = LaunchConfiguration('map_file')

    declare_map_file_argument = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Name of the map file in the config folder'
    )

    # Get package directories
    map_server_dir = get_package_share_directory('map_server')
    map_file_path = PathJoinSubstitution([map_server_dir, 'config', map_file_arg])

    localization_dir = get_package_share_directory('localization_server')
    path_planner_dir = get_package_share_directory('path_planner_server')

    # Determine use_sim_time and AMCL config based on map file
    use_sim_time = PythonExpression([
        "'warehouse_map_real.yaml' not in '", map_file_arg, "'"
    ])

    nav2_yaml = PythonExpression([
        "'", localization_dir, "/config/amcl_config_real.yaml' if 'warehouse_map_real.yaml' in '",
        map_file_arg, "' else '", localization_dir, "/config/amcl_config_sim.yaml'"
    ])

    rviz_config_dir = os.path.join(localization_dir, 'rviz', 'localization.rviz')
    filters_yaml = os.path.join(path_planner_dir, 'config', 'filters.yaml')

    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file_path}]
    )

    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[filters_yaml]
    )

    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[filters_yaml]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )

    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'filter_mask_server',
                                        'costmap_filter_info_server']}]
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
        declare_map_file_argument,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz_node
    ])