#!/usr/bin/env python3

"""
Warehouse Robot Task - Simple Commander API (Real Robot)

This script implements a complete autonomous warehouse workflow for the real robot:
1. Localize at init_position
2. Navigate to loading_position
3. Attach to shelf (via service)
4. Navigate to shipping_position (avoiding cones)
5. Detach from shelf
6. Return to init_position
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from attach_shelf.srv import GoToLoading

# Waypoint positions [x, y, orientation_z, orientation_w]
POSITIONS = {
    'init': [2.86854, 0.0679181, -0.794687, 0.60702],
    'loading': [1.23159, -4.41577, 0.9845, 0.175383],
    'shipping': [3.84222, -2.32182, -0.142095, 0.989853],
}

# Footprint definitions (octagon shapes)
FOOTPRINT_NORMAL = "[[0.30, 0.0], [0.2121, 0.2121], [0.0, 0.30], [-0.2121, 0.2121], [-0.30, 0.0], [-0.2121, -0.2121], [0.0, -0.30], [0.2121, -0.2121]]"
FOOTPRINT_SHELF = "[[0.55, 0.0], [0.3890, 0.3890], [0.0, 0.55], [-0.3890, 0.3890], [-0.55, 0.0], [-0.3890, -0.3890], [0.0, -0.55], [0.3890, -0.3890]]"


def create_pose(position_name, navigator):
    """Create a PoseStamped from a position in the POSITIONS dictionary."""
    x, y, z, w = POSITIONS[position_name]
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose


def update_footprint(node, footprint_str, with_shelf=True):
    """Update robot footprint and related costmap parameters."""

    # Target costmap nodes
    target_nodes = [
        '/local_costmap/local_costmap',
        '/global_costmap/global_costmap'
    ]

    # Set obstacle ranges based on whether we have shelf
    if with_shelf:
        obstacle_min = 0.55
        raytrace_min = 0.55
    else:
        obstacle_min = 0.0
        raytrace_min = 0.0

    # Create parameters
    parameters = [
        Parameter(name='footprint', value=footprint_str),
        Parameter(name='voxel_layer.obstacle_min_range', value=obstacle_min),
        Parameter(name='voxel_layer.raytrace_min_range', value=raytrace_min)
    ]

    # Convert to ROS parameter messages
    ros_parameters = [Parameter.to_parameter_msg(param) for param in parameters]

    # Update each costmap
    for target in target_nodes:
        param_client = node.create_client(SetParameters, f'{target}/set_parameters')

        if not param_client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(f'Cannot reach {target}/set_parameters')
            continue

        request = SetParameters.Request(parameters=ros_parameters)
        future = param_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if future.done():
            node.get_logger().info(f'Updated footprint on {target}')
        else:
            node.get_logger().warn(f'Timeout updating {target}')

    time.sleep(0.5)  # Brief pause for parameters to take effect


def main():
    """Main function for warehouse robot task."""

    # Initialize ROS2
    rclpy.init()

    # Create a simple node for service calls and publishers
    node = Node('warehouse_task_node')

    # Initialize Simple Commander API
    navigator = BasicNavigator()

    # Create service client for shelf attachment
    attach_client = node.create_client(GoToLoading, '/approach_shelf')

    # Create publishers for elevator control
    elevator_up_pub = node.create_publisher(String, '/elevator_up', 10)
    elevator_down_pub = node.create_publisher(String, '/elevator_down', 10)

    # Create publisher for manual velocity control
    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    node.get_logger().info('Starting warehouse robot task...')

    # STEP 1: Set initial pose for localization
    node.get_logger().info('[Step 1/7] Setting initial pose for localization')

    initial_pose = create_pose('init', navigator)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2 to activate
    node.get_logger().info('Waiting for Nav2 to activate...')
    navigator.waitUntilNav2Active()
    node.get_logger().info('Nav2 is active!')

    time.sleep(2.0)  # Give localization time to converge

    # STEP 2: Navigate to loading position
    node.get_logger().info('[Step 2/7] Navigating to loading position')

    loading_pose = create_pose('loading', navigator)
    navigator.goToPose(loading_pose)

    # Monitor navigation progress
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            node.get_logger().debug(f'ETA to loading position: {eta:.0f} seconds')

    # Check result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        node.get_logger().info('Successfully reached loading position!')
    elif result == TaskResult.CANCELED:
        node.get_logger().error('Navigation to loading position was canceled!')
        return
    elif result == TaskResult.FAILED:
        node.get_logger().error('Navigation to loading position failed!')
        return

    # STEP 3: Attach to shelf
    node.get_logger().info('[Step 3/7] Attaching to shelf')

    # Wait for service to be available
    node.get_logger().info('Waiting for /approach_shelf service...')
    while not attach_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().debug('Service not available, waiting...')

    # Call the service
    request = GoToLoading.Request()
    request.attach_to_shelf = True
    future = attach_client.call_async(request)

    # Wait for service to complete
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)

    if future.result() is not None:
        response = future.result()
        if response.complete:
            node.get_logger().info('Successfully attached to shelf!')

            # Raise the elevator
            node.get_logger().debug('Raising elevator...')
            elevator_up_msg = String()
            elevator_up_msg.data = ''
            for _ in range(3):
                elevator_up_pub.publish(elevator_up_msg)
                time.sleep(1.0)
            node.get_logger().info('Elevator raised!')
            time.sleep(2.0)

            # Update footprint for shelf
            node.get_logger().debug('Updating robot footprint for shelf...')
            update_footprint(node, FOOTPRINT_SHELF, with_shelf=True)
            node.get_logger().info('Robot footprint updated (0.55m octagon)')
        else:
            node.get_logger().error('Failed to attach to shelf')
            return
    else:
        node.get_logger().error('Service call failed!')
        return

    time.sleep(1.0)  # Brief pause after attachment

    # STEP 4: Move backward with curved motion to clear the shelf area
    node.get_logger().info('[Step 4/8] Moving backward from shelf with curved motion')

    # Use curved backward motion to turn while backing out
    distance = 1.75  # meters
    curvature = 0.10  # rad/s (slight curve to turn while backing)
    speed = 0.2  # m/s

    backward_twist = Twist()
    backward_twist.linear.x = -speed
    backward_twist.angular.z = curvature

    # Calculate duration based on distance and speed
    duration = distance / speed
    node.get_logger().info(f'Backing up {distance}m at {speed}m/s with curvature {curvature} for {duration:.1f}s')

    start_time = node.get_clock().now()
    while (node.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
        cmd_vel_pub.publish(backward_twist)
        time.sleep(0.1)

    # Add in-place rotation to finish the turn (about -45 degrees)
    node.get_logger().info('Completing rotation in place...')
    rotate_twist = Twist()
    rotate_twist.angular.z = -0.7854  # -45 degrees in rad/s

    rotate_duration = 3.0  # seconds
    start_time = node.get_clock().now()
    while (node.get_clock().now() - start_time).nanoseconds / 1e9 < rotate_duration:
        cmd_vel_pub.publish(rotate_twist)
        time.sleep(0.1)

    # Stop the robot
    stop_twist = Twist()
    for _ in range(10):  # Send multiple stop commands to ensure robot stops
        cmd_vel_pub.publish(stop_twist)
        time.sleep(0.1)

    node.get_logger().info('Successfully cleared shelf area and turned around')

    # STEP 5: Navigate to shipping position with shelf (avoiding cones)
    node.get_logger().info('[Step 5/8] Navigating to shipping position (avoiding cones)')

    shipping_pose = create_pose('shipping', navigator)
    navigator.goToPose(shipping_pose)

    # Monitor navigation progress
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            node.get_logger().debug(f'ETA to shipping position: {eta:.0f} seconds')

    # Check result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        node.get_logger().info('Successfully reached shipping position!')
    elif result == TaskResult.CANCELED:
        node.get_logger().error('Navigation to shipping position was canceled!')
        return
    elif result == TaskResult.FAILED:
        node.get_logger().error('Navigation to shipping position failed!')
        return

    # STEP 6: Detach from shelf
    node.get_logger().info('[Step 6/8] Detaching from shelf')

    # Lower the elevator
    node.get_logger().debug('Lowering elevator...')
    elevator_down_msg = String()
    elevator_down_msg.data = 'down'
    for _ in range(3):
        elevator_down_pub.publish(elevator_down_msg)
        time.sleep(1.0)
    node.get_logger().info('Elevator lowered!')
    time.sleep(2.0)

    # Back out from under the shelf (straight backward, no curvature)
    distance = 1.5  # meters
    speed = 0.2  # m/s

    backup_twist = Twist()
    backup_twist.linear.x = -speed
    backup_twist.angular.z = 0.0  # No curvature, straight backward

    # Calculate duration based on distance and speed
    duration = distance / speed
    node.get_logger().info(f'Backing up {distance}m at {speed}m/s for {duration:.1f}s')

    start_time = node.get_clock().now()
    while (node.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
        cmd_vel_pub.publish(backup_twist)
        time.sleep(0.1)

    # Stop the robot
    stop_twist = Twist()
    for _ in range(10):  # Send multiple stop commands to ensure robot stops
        cmd_vel_pub.publish(stop_twist)
        time.sleep(0.1)

    node.get_logger().info('Successfully detached from shelf!')

    # Restore normal footprint
    node.get_logger().debug('Restoring normal robot footprint...')
    update_footprint(node, FOOTPRINT_NORMAL, with_shelf=False)
    node.get_logger().info('Robot footprint restored (0.30m octagon)')

    time.sleep(1.0)  # Brief pause after detachment

    # STEP 7: Return to init position
    node.get_logger().info('[Step 7/8] Returning to init position')

    init_pose = create_pose('init', navigator)
    navigator.goToPose(init_pose)

    # Monitor navigation progress
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            node.get_logger().debug(f'ETA to init position: {eta:.0f} seconds')

    # Check result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        node.get_logger().info('Successfully returned to init position!')
        node.get_logger().info('Robot is ready for next task!')
    elif result == TaskResult.CANCELED:
        node.get_logger().error('Navigation to init position was canceled!')
        return
    elif result == TaskResult.FAILED:
        node.get_logger().error('Navigation to init position failed!')
        return

    # TASK COMPLETE
    node.get_logger().info('WAREHOUSE TASK COMPLETED SUCCESSFULLY!')
    node.get_logger().info('Robot is ready to move another shelf.')

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
