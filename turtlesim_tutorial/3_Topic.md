# Topic

enter the command 
```
rqt_graph
```
This command visualizes the changing nodes and topics, as well as the connections between them.

picture!!!

The `/teleop_turtle` node is publishing data (the keystrokes you enter to move the turtle around)
to the `/turtle1/cmd_vel` topic, and the  `/turtlesim` node is subscribed to that topic to receive the data.


