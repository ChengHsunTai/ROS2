# 看門狗
當雙方在進行數據交握時，確保彼此都在線上是至關重要的。這樣可以及時發現任何一方出現斷訊或當機的情況，從而防止訊息傳輸失敗或造成更大的損失。透過這種方式，不僅能保障數據傳輸的完整性，還能確保雙方在通信過程中的協同一致，從而有效地減少風險和損失。

看門狗的機制為，設置兩隻看門狗分別在兩方通訊設備上，牠們通過發送1、0變換的訊息進行交流。如果其中一隻看門狗停止發送變換消息或發送沒有變換的訊息，另一隻看門狗將發出警告。為了實現這一點，我使用了在ROS2 中的topic以及node來實現本次專案。每個node都會透過topic訂閱另一個節點的訊息，如下圖。

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
wget -O publisher_member_function.cpp https://github.com/ChengHsunTai/ROS2/blob/89e3276ff9a9cd190afc218791eeb48a262dbfbf/project/node_a.cpp
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
wget -O subscriber_member_function.cpp https://github.com/ChengHsunTai/ROS2/blob/89e3276ff9a9cd190afc218791eeb48a262dbfbf/project/subscriber_member_function.cpp
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

### 5. result

* 1. when two dogs are receiving the message successfully
![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/85cd2ef8-3bbb-43b4-8237-774c7ef58fe9)

* 2. stop running the `member_b` node
 
![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/9f89f897-c9be-403b-aeb8-76faea4907cd)

* 3. run the `member_b` again
 
![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/bc031d4a-a67e-41e6-9739-fdd915b7000b)

* 4. stop running the `member_a`
 
![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/410254af-cf08-4f82-ad6c-6ea7b2f5218c)

* 5. use `rqt_graph` to see the relationship between two nodes:

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/113e1e32-3c92-48c4-ab8d-c4ea99120c17)

As you can see, there's two topics connect the nodes.


## 成果

### 1. 當 member_b 接收不到member_a 的訊息時(左邊終端機為member_b，右邊為member_a)
![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/1fdbf4a8-2a1f-437a-9825-9fecd826a1bd)

### 2. 當 member_a 接收不到member_b 的訊息時

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/4155d0a2-76fa-4e24-8a43-f15ec5d8b760)

### 3. 模擬 member_b 當機情況，持續輸出 1。member_a 持續輸出警訊。

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/785d3633-74b0-4c7a-922e-5190cdb5143f)



