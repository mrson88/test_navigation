import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from arduinobot_msgs.action import MoveToPoint  # Update with your package name
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
# class MoveToPointActionClient(Node):
# 
#     def __init__(self):
#         super().__init__('move_to_point_action_client')
#         self._action_client = ActionClient(self, MoveToPoint, 'task_server')
# 
#     def send_goal(self, x, y, z):
#         goal_msg = MoveToPoint.Goal()
#         goal_msg.target_point.x = x
#         goal_msg.target_point.y = y
#         goal_msg.target_point.z = z
# 
#         self._action_client.wait_for_server()
#         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)
# 
#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected :(')
#             return
# 
#         self.get_logger().info('Goal accepted :)')
# 
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)
# 
#     def get_result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info(f'Result: {result.success}')
# 
#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         self.get_logger().info(f'Distance to goal: {feedback.distance_to_goal}')

class NavigateToPoseActionClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'task_server')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Current pose: {0}'.format(feedback.current_pose))

def main(args=None):
    rclpy.init(args=args)
#     action_client = MoveToPointActionClient()
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 2.0
    pose.pose.position.y = 3.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0 
    action_client = NavigateToPoseActionClient()
#     action_client.send_goal(1.0, 1.0, 1.0)  # Example target point
    action_client.send_goal(pose)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
