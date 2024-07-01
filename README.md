# ROS2
## introduction of ROS2
ROS2, Robot Operating system, is an open-source framework designed to facilitate the development of robotic systems. It provides a flexible and distributed architecture for building software that controls robots and other robotic devices.

Here's a lists of distribution of ROS2

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/f9375b7f-f1f0-4e68-88b7-1bf30f5252ee)

Although the latest ROS 2 LTS distribution is ROS 2 Jazzy, it only supports Ubuntu 24.04, which is too new for certain softwares. Therefore, I have decided to install the previous LTS distribution: ROS 2 Humble.

## Installing ROS2 Humble
> Here is a step-by-step guide to install ROS2 Humble on Linux and Ubuntu 22.04.

### 1. add ROS2 apt repository
> You need to add the ROS2 apt repository in your system

> [!NOTE]
> apt(Advanced Packaging Tool): Used for installing, updating, or removing software.  
> 
> repository :a storage location

* 1. ensure that the *Ubuntu Universe repository* is enable
> Ubuntu Universe repository is community maintained and it provides free and open-source software.
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/19725b20-c7d6-4f0c-82f4-5fdd797643e1)


* 2. Add ROS2 GPG key with apt
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Download the key from the ROS2 website and save it to the path `/usr/share/keyrings/ros-archive-keyring.gpg`.

* 3. then add the repository to your sources list.
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Set up the ROS2 software repository on Ubuntu to inform the system where to find ROS 2 packages and updates.

### 2. Install ROS2 Packages

* 1. Update your apt repository caches after setting up the repositories.
```
sudo apt update
```

* 2. ensure your system is up to date before installing new packages.
```
sudo apt upgrade
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/b3812121-e724-45c0-8da0-e4b2f7456f88)


* 3. Desktop install
> ROS, RViz, demos, tutorials

```
sudo apt install ros-humble-desktop
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/1e5f924f-c31d-4558-b12f-90b44e754f49)

enter `Y` (It will take some time to install.)

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/4d5eb938-9ba6-4b74-bcdb-22487a85a447)

* 4. ROS-Base Install
>  Communication libraries, message packages, command line tools. No GUI tools

```
sudo apt install ros-humble-ros-base
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/66c05894-ed42-480f-838c-0fd0856195bd)

* 5. Development tools
> Compilers and other tools to build ROS packages

```
sudo apt install ros-dev-tools
```
![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/12ea9eba-63dc-477e-8614-8e67563a1f5f)

### 3. environment setup
>Sourcing the setup script

* 1. Set up your environment by sourcing the following file.
 
```
source /opt/ros/humble/setup.bash
```

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

### 4. Try some demo to check whether installation successful or not

* 1. open one terminal, source the setup file and run a `talker`(C++):
```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```
* 2. Open another terminal, and source the setup file and then run `lestener`(python):
```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

* 3. And the result will be shown like below

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/4d044aed-0055-4d98-bf8e-1d6710a3bdd8)

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/434e7c5d-0e9e-4e80-b1e4-7d6547dc3bb5)

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










