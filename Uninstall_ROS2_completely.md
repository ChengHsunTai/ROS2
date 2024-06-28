# How to uninstall ROS2 
> In this section, you will learn how to uninstall ROS2 completely in linux/Ubuntu 22.04

## check all distrbution of ROS2 you have installed
* 1. open a terminal and enter the command below:
```
cd /opt/ros
ls
```
For example below, you'll see one distribution `humble` in `ros` directory.

![unnamed (11)](https://github.com/ChengHsunTai/ROS2/assets/137912642/6c1149ec-ad69-4dbd-8b1d-f68446bd62a6)

after checking for distributions of ROS2, we turn back to remove ROS2

`cd `

## Remove ROS2
* 1. enter the command to remove ROS2:
```
sudo apt remove ros-*
sudo apt autoremove
```
* 2. check all packages have been removed, enter the command again

`sudo apt autoremove`

the terminal should return:

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/a50587e9-6c12-49e7-81d9-4e43978ed7b2)

* 3. remove the ROS2 source that you manually added during the installation

enter the command below:

`sudo apt update`

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/74946aa9-9e42-4e1a-a5ca-cc805cbc34c2)

after updating, ros2 package still exist

* 4. go into the directory `/etc/apt/sources.list.d/`

```
cd /etc/apt/sources.list.d/
ls
```
you'll see `ros.list` in that directory:

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/8e421179-5e07-4b5f-9d39-5d67794cff86)

* 5. remove `ros2.list`

`sudo rm ros2.list`

* 6. turn to home directory, and update again:
```
cd
sudo apt update
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/253538b7-97bb-4ca6-be88-81627a1c7c8c)

the ros package is gone!

## reboot
At last, remember to reboot your computer after removing multiple packages.

`sudo reboot`
