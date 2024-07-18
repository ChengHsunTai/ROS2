# Watch dog
When both parties are exchanging data, ensuring both are online is crucial to detect disconnections or crashes, preventing message transmission failures and losses. This maintains data integrity and coordination, reducing risks.

The watchdog mechanism involves setting up two watchdogs on each communication device, which communicate by sending alternating 1 and 0 messages. If one watchdog stops sending alternating messages, the other issues a warning. This project uses ROS2 topics and nodes, where each node subscribes to the other's messages, as illustrated below.

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/113e1e32-3c92-48c4-ab8d-c4ea99120c17)

## 步驟
### 1. creating a new package

in the workspace root `ros2_ws`, enter the command:
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 guard_dog
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/56493ceb-2c6b-450c-8f34-2a4251e975b1)


### 2. write the first guard dog  `node_a`


navigate into `ros2_ws/src/guard_dog/src`, and enter the command below to download the publisher node:

```
wget -O publisher_member_function.cpp https://github.com/ChengHsunTai/ROS2/blob/edca1389d3c3ff43e434c0bea40336e696d3e068/project/%E7%9C%8B%E9%96%80%E7%8B%97/member_a.cpp
```
* 1. add dependencies
 
open `package.xml` in the `ros2_ws/src/guard_dog` directorty.
Add a new line after the `ament_cmake` buildtool dependency and paste the following dependencies corresponding to your node's include statements:

```
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/0c734d73-e374-42f1-8c01-0952aceb3d27)

This declares the package needs rclcpp and std_msgs when its code is built and executed.

remember to save the file.

* 2. CMakeList.txt

add lines after `find_package(ament_cmake REQUIRED)`:

```
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```
After that, add the executable and name it `member_a` so you can run your node using `ros2 run`:

```
add_executable(member_a src/node_a.cpp)
ament_target_dependencies(member_a rclcpp std_msgs)
```

Finally, add the `install(TARGETS...)` section so `ros2 run` can find your executable:

```
install(TARGETS
  member_a
  DESTINATION lib/${PROJECT_NAME})
```

### 3. write the subscriber node

navigate into `ros2_ws/src/guard_dog/src`, and enter the command:

```
wget -O subscriber_member_function.cpp https://github.com/ChengHsunTai/ROS2/blob/edca1389d3c3ff43e434c0bea40336e696d3e068/project/%E7%9C%8B%E9%96%80%E7%8B%97/member_b.cpp
```

Reopen `CMakeLists.txt` and add the executable and target for the subscriber node below the publisher’s entries.

```
add_executable(member_b src/node_b.cpp)
ament_target_dependencies(member_b rclcpp std_msgs)

install(TARGETS
  member_a
  member_b
  DESTINATION lib/${PROJECT_NAME})
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/1df5ada4-9a6c-4039-bc4d-ef4ee645d889)

### 4. build and run

In the root of your workspace, `ros2_ws`, build your new package:

```
colcon build --packages-select guard_dog
```

Open a new terminal, navigate to `ros2_ws`, source the setup file:

```
. install/setup.bash
```

run the talker node

```
ros2 run guard_dog member_a
```

Open another terminal, source the setup files from inside `ros2_ws` again, and then start the listener node:

```
ros2 run guard_dog member_b
```


## result

### 1. When member_b stops receiving messages from member_a (left terminal is member_b, right terminal is member_a)
![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/1fdbf4a8-2a1f-437a-9825-9fecd826a1bd)

### 2. When member_a stops receiving messages from member_b

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/4155d0a2-76fa-4e24-8a43-f15ec5d8b760)

### 3. To simulate member_b crashing, it continuously outputs 1. Meanwhile, member_a continuously outputs a warning.

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/785d3633-74b0-4c7a-922e-5190cdb5143f)



