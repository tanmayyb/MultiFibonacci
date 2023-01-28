import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from action_tutorials_interfaces.action import Fibonacci

class miniFibonacciClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self, 
            Fibonacci, 
            'fibonacci')

        self.goal_handle = None

        print("client created")

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        #self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        #print(_goal_handle)
        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(':{0}'.format(feedback.partial_sequence))

    def cancel_goal(self):
        cancel_future  = self._goal_handle.cancel_goal_async() #requesting cancel
        print("tryna cancel")
        cancel_future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().warning('Goal successfully canceled')
        else:
            self.get_logger().warning('Goal failed to cancel')
        