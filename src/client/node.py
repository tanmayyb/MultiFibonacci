import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger
from action_tutorials_interfaces.action import Fibonacci

class multiFiboClient(Node):

    def __init__(self):
        super().__init__('multifibo_client')

        self.send_goals_to_action_manager_publisher = self.create_publisher(
            Int32MultiArray,
            'multifibonacci_action_manager_goals',
            10)

        self.cancel_multifibo_goals_service_client = self.create_client(
            Trigger, 
            'cancel_all_multifibo_goals')
        
        self.get_logger().info("Client Node Created!")

    def prep_and_send_goals_of_order(self, order):
        goals = []
        
        #sequentially increase order of goals
        for i in range(1, order+1):
            goals.append(i)

        self.send_goals(goals)


    def send_goals(self, goals):
        msg = Int32MultiArray()
        msg.data = goals

        self.send_goals_to_action_manager_publisher.publish(msg)
        self.get_logger().info("Sent Goal List of len {0} to MultiFibo Action Manager...".format(len(goals)))


    def cancel_all_goals(self):
        self.get_logger().error("Trying to Cancel MultiFibo Goals")

        while not self.cancel_multifibo_goals_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
                
        # code goes here
        self.req = Trigger.Request()
        future = self.cancel_multifibo_goals_service_client.call_async(self.req)
        future.add_done_callback(self.cancel_all_goals_callback)

    def cancel_all_goals_callback(self, future):
        self.get_logger().info('All Goals Should Now Be Cancelled...')
        


        