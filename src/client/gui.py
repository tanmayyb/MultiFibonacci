from tkinter import *
from threading import *
import random

import rclpy
from rclpy.executors import MultiThreadedExecutor

from window import *
from node import *


class gui(Thread):

    def __init__(self, window):
        Thread.__init__(self)
        
        self.window = window
        self.setup_window()
        self.draw_gui()

        rclpy.init(args=None)
        self.setup_nodes()

        print("[INFO] [gui]: Gui Ready...")


    def setup_window(self):
         # Window setup
        self.window.title("multifibonacci control gui")
        self.window.geometry(SETUP_STRING)

    def draw_gui(self):
        self.draw_labels()
        self.draw_text_fields()
        self.draw_buttons()
    
    def draw_labels(self):
        self.label1 = Label(
            self.window,
            text="# of goals:")

        self.label1.grid(
            row=1,
            column=1, 
            columnspan=2,
            sticky='W')


    def draw_buttons(self):
        self.button1 = Button(
            self.window, 
            text = "Execute Multifibo", 
            command=lambda: self.send_action(self.text_field1_input.get()), 
        )

        self.button2 = Button(
            self.window,
            text = "Cancel Multifibo",
            command=lambda: self.cancel_all_goals(), 
        )

        self.button1.grid(row=2,column=3, sticky='W')
        self.button2.grid(row=3,column=3, sticky='W')

    def draw_text_fields(self):
        default_text_field1_input = 5
        self.text_field1_input = StringVar()
        self.text_field1_input.set(default_text_field1_input)
        self.text_field1 = Entry(
            self.window,
            width=5,
            textvariable=self.text_field1_input)

        self.text_field1.grid(row=1,column=3, sticky='W')
        


    def setup_nodes(self):
        self.executor = MultiThreadedExecutor()

        
        self.multifibo_client = multiFiboClient()
        self.executor.add_node(self.multifibo_client)
        
        
        self.executor_thread = Thread(
            target=self.executor.spin)
        self.executor_thread.start()


    def send_action(self, order):
        self.multifibo_client.prep_and_send_goals_of_order(int(order))

    def cancel_all_goals(self):
        self.multifibo_client.cancel_all_goals()

        
