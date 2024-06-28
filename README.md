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
picture!!!

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

picture!!!

* 3. Desktop install
> ROS, RViz, demos, tutorials

```
sudo apt install ros-humble-desktop
```

picture!!!

enter `Y` (It will take some time to install.)

picture!!!

* 4. ROS-Base Install
>  Communication libraries, message packages, command line tools. No GUI tools

```
sudo apt install ros-humble-ros-base
```

* 5. Development tools
> Compilers and other tools to build ROS packages

```
sudo apt install ros-dev-tools
```

## 3. environment setup
>Sourcing the setup script

* 1. Set up your environment by sourcing the following file.
 
```
source /opt/ros/humble/setup.bash
```

## 4. Try some demo to check whether installation successful or not

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

![image](6ec77760.png)
![image](9a06ab5b.png)









