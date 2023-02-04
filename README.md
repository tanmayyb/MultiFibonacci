# MultiFibonacci

This repo demonstrates a ROS2-Python Action Manager GUI to chain/sequence/control ROS2 actions. This `Multifibonacci Action Manager` will sequence smaller `Fibonacci` actions which are a part of `example_interfaces` in ROS2. This is useful when you want your robot to do complex behaviour like walk in a circle. Patrol an area. Etc. 
</br>
In this repo a simple msg type(Int32MultiArray) is used but for more complex actions management custom msg types can be used - and the manager can be programmed accordingly. The `Action Manager` design could potentially be translated into a ROS2 action. But that is trickier to program due to asynchronous context handling. 


## Usage Intruction

### for client side
Open `src/client` in terminal and run this:
```
python3 app.py
```

### for server side
Open `src/server` in terminal and run this:

```
python3 multifibo_action_manager.py
```
and, this:
```
python3 minifibo_server.py
```
</br>
You should be able to chain the `Fibonacci` actions and cancel them graphically.