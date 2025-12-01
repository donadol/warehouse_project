from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_attach_shelf = get_package_share_directory('attach_shelf')
    rviz_config_file = os.path.join(pkg_attach_shelf, 'rviz', 'config.rviz')

    # Declare launch arguments
    obstacle_arg = DeclareLaunchArgument(
        'obstacle',
        default_value='0.3',
        description='Distance to obstacle in meters at which robot stops'
    )

    degrees_arg = DeclareLaunchArgument(
        'degrees',
        default_value='-90',
        description='Degrees to rotate after stopping (negative = right turn)'
    )

    final_approach_arg = DeclareLaunchArgument(
        'final_approach',
        default_value='false',
        description='Whether to perform final approach and attach to shelf'
    )

    # Get launch configurations
    obstacle = LaunchConfiguration('obstacle')
    degrees = LaunchConfiguration('degrees')
    final_approach = LaunchConfiguration('final_approach')

    # Approach service server node
    approach_service_server_node = Node(
        package='attach_shelf',
        executable='approach_service_server_node',
        output='screen',
        name='approach_service_server'
    )

    # Pre-approach v2 node
    pre_approach_v2_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2_node',
        output='screen',
        name='pre_approach_v2',
        parameters=[{
            'obstacle': obstacle,
            'degrees': degrees,
            'final_approach': final_approach
        }]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Delay pre_approach_v2_node to allow TF tree to stabilize
    delayed_pre_approach = TimerAction(
        period=3.0,  # Wait 3 seconds
        actions=[pre_approach_v2_node]
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        approach_service_server_node,
        rviz_node,
        delayed_pre_approach  # Launch pre_approach_v2 after delay
    ])
