#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

def movebase_client():
    rclpy.init()

    # Create a ROS 2 node
    node = rclpy.create_node('movebase_client')

    # Create an action client called "navigate_to_pose" with action definition "NavigateToPose"
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # Wait until the action server has started up and started listening for goals
    if not client.wait_for_server(timeout_sec=5.0):
        print("Action server not available!")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Define the forward distance
    forward_distance = 1.0  # Adjust as needed

    # Create a goal message with the PoseStamped message
    goal_msg = NavigateToPose.Goal()

    # Set the goal target pose
    goal_msg.pose.header.frame_id = 'base_link'  # Set the frame according to your robot's frame
    goal_msg.pose.pose.position.x = forward_distance  # Move forward by the specified distance
    goal_msg.pose.pose.orientation.w = 1.0  # Set a valid orientation (identity quaternion)

    # Send the goal to the action server
    goal_handle_future = client.send_goal_async(goal_msg)

    # Wait for the goal to complete
    while rclpy.ok():
        rclpy.spin_once(node)
        if goal_handle_future.done():
            goal_handle = goal_handle_future.result()
            if goal_handle.accepted:
                print("Goal accepted.")
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(node, result_future)
                result = result_future.result().result
                if result != False:
                    print("Goal execution done!")
                else:
                    print("Goal execution failed!")
                break
            else:
                print("Goal rejected.")
                break

    # Clean up resources
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    movebase_client()
