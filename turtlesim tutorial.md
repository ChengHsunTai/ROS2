# Turtlesim
> Turtlesim is a basic simulator designed for beginners learning ROS2. It illustrate what the ROS2 does at the most basic level of ROS2 operation,
> and give you an idea of what you will do with a real robot or robot simulation.

You will learn how to excecute ROS2_humble_Turtlesim in linux/Ubunto 22.04 by following the instructions below.

## 1. Sourcing your setup files
As always, remember to source setup files for every new terminal.

`source /opt/ros/humble/setup.bash`

>[!TIP]
>To avoid sourcing the setup every time, add the code above to your bashrc.
>
>* Open a new terminal and edit bashrc
>
>`nano ~/.bashrc`
>
>* add the code below to the end of your ~/.bashrc file.
>
>`source /opt/ros/humble/setup.bash`
>
>![7972e1c7](https://github.com/ChengHsunTai/ROS2/assets/137912642/3c5997d0-28f8-46fd-a9fe-18472bb0aed4)
>
> save the file and exit (`ctnl+x` and press `y` to save the file, press `enter` to exit the editor)
> 
>* reload the ~/.bashrc
>
>`source ~/.bashrc`

## 2. Install Turtlesim
* install the turtlesim package for your ROS2 distro:
```
sudo apt update
sudo apt install ros-humble-turtlesim
```
* Check the package is installed
```
ros2 pkg executables turtlesim
```
It'll return a list of turtlesim's executables:

![d9b1ceaa](https://github.com/ChengHsunTai/ROS2/assets/137912642/4e764866-a70a-436f-8e95-de2b4de3320a)

## 3. Start Turtlesim
* enter the command below in the terminal

`ros2 run turtlesim turtlesim_node`

the simulator windows should appear with a random turtle

![4ebda826](https://github.com/ChengHsunTai/ROS2/assets/137912642/434c30a1-db39-4c1d-88b5-1bb96c4cb9d3)


and you'll see the default turtle's name and coordinates in the terminal:

![15109b9b](https://github.com/ChengHsunTai/ROS2/assets/137912642/99c9182b-5c53-410c-aaa4-f9dbced5fdbc)

## 4. Use Turtlesim
* Open a new terminal, and source the ROS2 again (if you do not edit ~/.bashrc)

Create a new node as a first node to control turtle

`ros2 run turtlesim turtle_teleop_key`

And it'll return :

![螢幕擷取畫面 2024-06-28 084449](https://github.com/ChengHsunTai/ROS2/assets/137912642/23f24fa5-ccb0-4a55-949e-ef84d0c16fda)

* So far, you should have three windows open: a turtlesim window, a terminal running `turtlesim_node`, and a terminal running `turtle_teleop_key`.
you can use the arrow keys to control the turtle in the terminal running `turtle_teleop_key`, and use G|B|V|C|D|E|R|T to control turtle's absalute direction.
the movement trajectory appears as shown below:
 
![螢幕擷取畫面 2024-06-28 084516](https://github.com/ChengHsunTai/ROS2/assets/137912642/3062e833-86d2-49f5-b37a-59e54b7a4fae)

* if your turtle move out of the window's range, the terminal running `turtlesim_node` will return `out of range` or `oh no! I hit the wall!`

![unnamed](https://github.com/ChengHsunTai/ROS2/assets/137912642/381ad121-6ccc-4c4d-80c2-59b32bd0f5af)
![unnamed (1)](https://github.com/ChengHsunTai/ROS2/assets/137912642/3e26105d-00ae-4aaf-bec0-d0467cf043fc)

>[!TIP]
> you can see the nodes and their associated topics, sevices, and ations by using `list` subcommands below:
>```
>ros2 node list
>ros2 topic list
>ros2 service list
>ros2 action list
>```
>You'll learn more about these concept in the comming tutorials.

## 5. Install rqt
* Open a new terminal to install the rqt
```
sudo apt update

sudo apt install '~nros-humble-rqt*'
```


* run the rqt
```
rqt
```
the rqt window will appear

![unnamed (2)](https://github.com/ChengHsunTai/ROS2/assets/137912642/0cc44bf2-43ea-478f-90e3-9d60789a6785)

* When running the rqt for the first time,it's normal being blank. Select `Plugins > Services > Service Caller`from the menu bar at the top.

![unnamed (3)](https://github.com/ChengHsunTai/ROS2/assets/137912642/e9410e0a-3b40-4b66-9005-00c8c686887b)

* click on the service dropdown list to see the `turtlesim`'s services, and select `/spawn` service.

### 5.1 Let's use the rqt to call the `spawn` service. this service will create a new turtle in the turtlesim window.

give the new turtle a unique name like `turtle2` by double-clicking the column `Expression`. Next, enter some valid coordinates at which to spawn the new turtle, like `x = 1.0` and `y = 1.0`.

![unnamed (4)](https://github.com/ChengHsunTai/ROS2/assets/137912642/873f26ea-e5e3-4c7b-93df-8be14f50af2b)

>[!NOTE]
>If you try to spawn a new turtle with the same name as an existing turtle, like the default turtle1, you will get an error message in the terminal running `turtlesim_node`:
>
>![unnamed (5)](https://github.com/ChengHsunTai/ROS2/assets/137912642/fee8a024-bb89-4cfb-b0ce-a8cc426ed3c1)


* To spawn `turtle2`, you need to call the service by clicking `Call` button.

![unnamed (6)](https://github.com/ChengHsunTai/ROS2/assets/137912642/f0e2cce9-c68f-4cab-9913-e214a8f3db08)

if the service call is successfully, you should see a new turtle spawn at the coordinate you input for x and y.

![unnamed (7)](https://github.com/ChengHsunTai/ROS2/assets/137912642/d4df4337-764b-4431-8b14-4b815f67ee3c)

If you refresh the service list in rqt, you will also see that now there are services related to the new turtle, `/turtle2/...`, in addition to `/turtle1/...`.

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/ff4b3b47-763c-4e58-8ee2-e421e937fbad)

### 5.2 set_pen Service
> now let's give a unique pen for `turtle1` using the `set_pen` service

* Select the `/turtle1/set_pen` service from Service dropdown, and edit the properties including `r`, `g`, `b`, `width`.

![unnamed (8)](https://github.com/ChengHsunTai/ROS2/assets/137912642/8c810de1-38ea-48df-a3d1-a2b3e49fa137)

* move `turtle1` from terminal running `turtle_teleop_key`, you'll see the path turtle1 made change.

![unnamed (9)](https://github.com/ChengHsunTai/ROS2/assets/137912642/2594a2e2-aaeb-44f9-ad20-ddfed0f4735b)

## 6. Remapping
> You’ve probably also noticed that there’s no way to move `turtle2`. That’s because there is no teleop node for `turtle2`.
> You need a second teleop node in order to control `turtle2`. However, if you try to run the same command as before, you will notice that this one also controls turtle1. The way to change this behavior is by remapping the `cmd_vel` topic.

* In the new terminal, source ROS2 if needed, and run

`ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel`

you can move `turtle2`, when this terminal is active, and `turtle1` when the other terminal running turtle_teleop_key is active.

![unnamed (10)](https://github.com/ChengHsunTai/ROS2/assets/137912642/05d440b8-1bad-41a1-8473-e21a12382973)





