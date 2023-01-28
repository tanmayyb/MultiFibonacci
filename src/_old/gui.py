from tkinter import *
from turtle import color
from act_client import *
import rclpy, random
from rclpy.executors import MultiThreadedExecutor
from threading import *


def setup_nodes(executor, action_client):

    """
    Threading
    """
    executor.add_node(action_client)
    executor_thread = Thread(
        target= executor.spin, 
        daemon=True)
    executor_thread.start()

    return executor, executor_thread, action_client

def send_action(action_client):
    num = random.randrange(5, 10, 2)
    print("sending %d to server", num)
    action_client.send_goal(num)


def cancel_goal(action_client):
    goal_id = action_client.cancel_goal();


def main(args=None):
    rclpy.init()

    action_client = miniFibonacciClient()

    executor = MultiThreadedExecutor()
    executor, executor_thread, action_client  = setup_nodes(
        executor, 
        action_client)

    root = Tk()
    root.geometry("200x200")

    button1 = Button(
        root, 
        text = "send action", 
        command=lambda: send_action(action_client), 
    )

    button2 = Button(
        root,
        text = "cancel all actions",
        command=lambda: cancel_goal(action_client), 
    )

    button1.grid(row=0,column=0, sticky='E')
    button2.grid(row=1,column=0, sticky='E')
    

    root.mainloop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
