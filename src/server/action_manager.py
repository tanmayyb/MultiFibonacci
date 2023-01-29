import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Int32
from action_tutorials_interfaces.action import Fibonacci

from rclpy.executors import MultiThreadedExecutor

import time

class multiFiboClient(Node):

    def __init__(self):
        super().__init__('multifibo_action_manager')
        self._action_client = ActionClient(
            self, 
            Fibonacci, 
            'mini_fibonacci')
        self.goal_handle = None
        self.goal_queue = []
        self.goal_queue_last_index = -1
        self.get_logger().info("multifibo action manager created")


        self.req_stop_all_actions = False


        self.get_logger().info("creating goals")
        self.add_goal_to_queue(3)
        self.add_goal_to_queue(4)
        self.add_goal_to_queue(5)
        self.add_goal_to_queue(6)
        self.add_goal_to_queue(7)

        self.gid = 0


        # self.pop_all_goals()

        self.get_logger().warning("!dequeing actions from goal queue!")
        self.execute_all_goals_in_queue()    
        
    
        #make into services

        # self.sub1 = self.create_subscription(
        #     self, 
        #     Int32,
        #     'add_point_to_queue',
        #     self.add_point_,
        #     10)

        # self.sub2 = self.create_subscription(
        #     self, 
        #     Int32,
        #     'delete_sing_point',
        #     )

        # self.sub3 = self.create_subscription(
        #     self,
        #     Int32,
        #     'delete_all_points',
        #     10
        # )



    def add_goal_to_queue(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.goal_queue.append(goal_msg)
        self.goal_queue_last_index=self.goal_queue_last_index+1

    def pop_goal_from_front(self):
        self.goal_queue.pop(0)
        self.goal_queue_last_index=self.goal_queue_last_index-1

    def pop_goal_from_back(self):
        self.goal_queue.pop(self.goal_queue_last_index)
        self.goal_queue_last_index=self.goal_queue_last_index-1

    def pop_all_goals(self):
        self.get_logger().info("popping all goals")
        for i in range(0, self.goal_queue_last_index+1):
            self.goal_queue.pop(0)
        self.goal_queue_last_index = -1
        self.get_logger().info("all goals popped")


    def execute_all_goals_in_queue(self):
        if self.gid<=self.goal_queue_last_index:
            goal = self.goal_queue[self.gid]
            print(self.gid, goal)
            self.get_logger().warning('sending fibonacci goal of order: %d' % (goal.order,))
            self.send_goal(goal)
        else:
            self.get_logger().warning('all goals successfully completed: %d' % (goal.order,))
            self.pop_all_goals()


    def send_goal(self, goal):
        send_goal_future = self._action_client.send_goal_async(
            goal, 
            feedback_callback=self.feedback_callback)

        #result = self._action_client.send_goal(goal)
        # print(result)

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
        self.get_logger().warning('Result: {0}'.format(result.sequence))
        
        self.gid = self.gid + 1
        self.execute_all_goals_in_queue()

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



def main(args=None):
    rclpy.init(args=args)

    _multiFiboClient = multiFiboClient()

    executor = MultiThreadedExecutor()

    rclpy.spin(_multiFiboClient, executor=executor)

    _multiFiboClient.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()