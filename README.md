# MultiFibonacci

![multifibonacci_architecture](https://user-images.githubusercontent.com/72982560/216751836-d4d1c493-03b6-42e0-a81a-2875c7e6dbad.jpg)


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
You should be able to chain the `Fibonacci` actions and cancel them graphically. As shown below: </br>

![ss_multifibo](https://user-images.githubusercontent.com/72982560/216749727-6e8a5c1e-8cfa-43fd-9b56-f883d855cc1d.png)
