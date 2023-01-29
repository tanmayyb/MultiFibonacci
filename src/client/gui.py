from tkinter import *
from threading import *
from window import *
import random, rclpy
from rclpy.executors import MultiThreadedExecutor
from client_node import *


class gui(Thread):

    def __init__(self, window):
        Thread.__init__(self)
        
        self.window = window
        self.setup_window()
        self.draw_gui()

        rclpy.init(args=None)
        self.setup_nodes()
        
        print("gui initialised")


    def setup_window(self):
         # Window setup
        self.window.title("cancel action gui")
        self.window.geometry(SETUP_STRING)

    def draw_gui(self):
        button1 = Button(
            self.window, 
            text = "send action", 
            command=lambda: self.send_action(), 
        )

        button2 = Button(
            self.window,
            text = "cancel all actions",
            command=lambda: self.cancel_goal(), 
        )

        button1.grid(row=0,column=0, sticky='E')
        button2.grid(row=1,column=0, sticky='E')

    def setup_nodes(self):

        self.action_client = multiFiboClient()

        """
        Threading
        """
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.action_client)
        self.executor_thread = Thread(
            target=self.executor.spin)
        self.executor_thread.start()


    def send_action(self):
        num = random.randrange(7, 12, 2)
        print("sending %d to server", num)
        self.action_client.send_goal(num)

    def cancel_goal(self):
        goal_id = self.action_client.cancel_goal()

        
