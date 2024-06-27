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
>Open a new terminal and edit bashrc
>
>`nano ~/.bashrc`
>
>add the code below to the end of your ~/.bashrc file.
>
>`source /opt/ros/humble/setup.bash`
>
>save the file and exit (`ctnl+x` and press `y` to save the file, press `enter` to exit the editor)
> ![7972e1c7](https://github.com/ChengHsunTai/ROS2/assets/137912642/3c5997d0-28f8-46fd-a9fe-18472bb0aed4)
>
>reload the ~/.bashrc
>
>`source ~/.bashrc`

## 2. Install Turtlesim
install the turtlesim package for your ROS2 distro:
```
sudo apt update
sudo apt install ros-humble-turtlesim
```
Check the package is installed
```
ros2 pkg executables turtlesim
```
![d9b1ceaa](https://github.com/ChengHsunTai/ROS2/assets/137912642/4e764866-a70a-436f-8e95-de2b4de3320a)


## 3. Start Turtlesim
enter the command below in the terminal

`ros2 run turtlesim turtlesim_node`

the simulator windows should appear with a random turtle

![4ebda826](https://github.com/ChengHsunTai/ROS2/assets/137912642/434c30a1-db39-4c1d-88b5-1bb96c4cb9d3)


and you'll see the default turtle's name and coordinates in the terminal:

![15109b9b](https://github.com/ChengHsunTai/ROS2/assets/137912642/99c9182b-5c53-410c-aaa4-f9dbced5fdbc)

