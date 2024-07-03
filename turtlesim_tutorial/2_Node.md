# Node
> Each node in ROS should be responsible for a single, modular purpose,
> e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder.
> Each node can send and receive data from other nodes via topics, services, actions, or parameters.

## Task

>[!NOTE]
> Always remember to source ROS2 in every terminal you open.

### 1. The command `ros2 run` launches an executable from package.
```
ros2 run <package_name> <executable_name>
```
* 1. To run turtlesim, open a new terminal, and enter the following command.
 
```
ros2 run turtlesim turtlesim_node
```

The turtlesim window will open, as you saw in the previous tutorial.

Here, `turtlesim` is the package name, and `turtlesim_node` is the executable name.

But we still dont know the node name. Open another terminal and enter the command

```
ros2 node list
```

picture!!!

terminal will return the node name `/turtlesim`
