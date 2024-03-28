import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from arduinobot_msgs.action import ArduinobotTask
import time
class YoloActionClient(Node):

    def __init__(self):
        super().__init__('yolo_action_client')
        self.action_client = ActionClient(self, ArduinobotTask, 'task_server')

    def send_goal(self, order):
        goal_msg = ArduinobotTask.Goal()
        goal_msg.task_number = order

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        time.sleep(2)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        
        self.get_logger().info('Result: {0}'.format(result.success))
        return result.success

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.percentage))
        
        
def main(args=None):
    rclpy.init(args=args)

    action_client = YoloActionClient()
#     action_client.send_goal(2)
#     action_client.get_logger().info('Result 1: {0}'.format(action_client.get_result_callback))
    # Example: sending a goal with order 5
    for i in range (15):
        action_client.send_goal(0)
        action_client.send_goal(1)
        action_client.send_goal(2)   
        i+=1
        if i==15:
            i=0

    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

