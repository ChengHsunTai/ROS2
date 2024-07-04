# Create my first project

## creating a Workspace
> In ROS2, a project is generally veiwed as a unit and packaged into a folder, which is referred to as a Workspace `ws`. Creating a workspace and learn how to set up an overlay for development and testing.

### 1. create a directory named `ros2_ws`

enter the command:

```
mkdir test
cd ~/test
mkdir src
```
the three commands above can be replaced by one command:
```
mkdir -p ~/ros2_ws/src
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/5c7d0ca6-6738-4c58-baba-d524c1d901db)

then navagate into the src directory.
```
cd ~/ros2_ws/src
```

### 2. clone a sample repo
 
in the `ros2_ws/src` directory, run the command:

```
git clone https://github.com/ros/ros_tutorials.git -b humble
```
Now, `ros_tutorials` is cloned in your workspace. 

So far,  you have populated your workspace with a sample package, but it isn’t a fully-functional workspace yet. You need to resolve the dependencies first and then build the workspace.

### 3. resolve dependencies

Before building the workspace, you need to resolve the package dependencies.

From the root of your workspace `ros2_ws`, run the following command:
```
rosdep install -i --from-path src --rosdistro humble -y
```
if success, the terminal return:

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/7c71edf4-8ef1-46fe-80cf-55d51e706a6d)

the terminal may return that you should install rosdep. follow the installation instruction:

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/b29f0fbf-ed71-4789-87d2-8652e08663a8)

### 4. build the workspace with colcon
 
From the root of your workspace `ros_ws`, you can now build your package:
```
colcon build
```
the console will return the following massage

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/3f4e90b9-0cf7-4692-868d-981903ae4916)


After build is finished, you'll see that colcon has create new directories in the workspace root `~/ros2_ws`:

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/6e16bdba-d6b2-496f-97b6-76d6536e2a18)


The `install` directory is where your workspace’s setup files are, which you can use to source your overlay.

### 5. source the overlay
 
In the new terminal, source your main ROS 2 environment as the “underlay”, so you can build the overlay “on top of” it:

```
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/local_setup.bash
```
>[!NOTE]
>Sourcing the local_setup of the overlay will only add the packages available in the overlay to your environment. setup sources the overlay as well as the underlay it was created in, allowing you to utilize both workspaces.
>
>So, sourcing your main ROS 2 installation’s setup and then the ros2_ws overlay’s local_setup, like you just did, is the same as just sourcing ros2_ws’s setup, because that includes the environment of its underlay.

### 6. run `turtlesim`
 
```
ros2 run turtlesim turtlesim_node
```

To ensure that this is the overlay turtlesim running, and not your main installation’s turtlesim, I modify the turtlesim in the overlay

the effect:
* You can modify and rebuild packages in the overlay separately from the underlay.
* The overlay takes precedence over the underlay.

modify the `turtle_frame.cpp` file, which located `~/ros2_ws/src/ros_tutorials/turtlesim/src`.

Find the fuction `setWindowTitle("TurtleSim");`, and change the value to `setWindowTitle("MyTurtleSim2024-0703");`.

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/484e9287-1cdf-42fb-8c03-4141b13c84da)

Then `colcon build` again.

In the second terminal(remember to source the overlay), run turtlesim again:
```
ros2 run turtlesim turtlesim_node
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/a25717d2-c5a2-4f9b-890b-73f3fe695fe3)

As you can see, the title bar of turtlesim's window chage to `MyTurtleSim2024-0703`.

## create a package
> A package is an organizational unit for your ROS 2 code. Package creation in ROS 2 uses ament as its build system and colcon as its build tool.

Here, I use CMake to build a package.

### 1. create a package

(in `~/ros2_ws/src` folder )The command syntax for creating a new package in ROS 2 is:

```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/a25dc15f-6a95-4372-8eca-8b578b62ce51)

You will now have a new folder within your workspace’s `src` directory called `my_package`.

### 2. build a package

colcon build the packages in the root of the workspace

```
colcon build
```
you can only build the package `my_package` by the command below:

```
colcon build --packages-select my_package
```

### 3. source the setup file

To use your new package and executable, first open a new terminal and source your main ROS 2 installation.
Then, from inside the `ros2_ws` directory, run the following command to source your workspace:

```
source install/local_setup.bash
```
Now that your workspace has been added to your path, you will be able to use your new package’s executables.

### 4. run the package

```
ros2 run my_package my_node
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/d151cfdf-b4c0-4236-b29f-0d4a455b0c9d)

### 5. customize `package.xml`

to be conti

## Writing a simple publisher and subscriber (C++)

### 1. create a new package

in the workspace root `ros2_ws`, enter the command:
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_pubsub
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/c6a618fa-82cd-4512-a381-36774d2edc2f)

### 2. write the publisher node

navigate into `ros2_ws/src/cpp_pubsub/src`, and enter the command below to download the publisher node:

```
wget -O publisher_member_function.cpp https://github.com/ChengHsunTai/ROS2/blob/89e3276ff9a9cd190afc218791eeb48a262dbfbf/project/publisher_member_function.cpp
```

* 1. examine the code

| Command | Description |
| ---- | --- |
|`#include <chrono>`|Provides facilities for working with time durations and time points.|
|`#include <functional>`| provides bind() function to bind functions with parameters.|
|`#include <memory>`| managing dynamic memory allocation, specifically smart pointers like `std::shared_ptr`.|

***
| Command | Description |
| ---- | --- |
|`#include "rclcpp/rclcpp.hpp"`|Includes the main header file for the ROS 2 C++ client library|
|`#include "std_msgs/msg/string.hpp"`| This is a standard message type in ROS 2 for sending string data.Used with `rclcpp::Subscription<std_msgs::msg::String>` to create a subscriber that listens for messages of type `std_msgs::msg::String`.|
|`using namespace std::chrono_literals;`| to define the time duration like : 1000ms|

***
/* This example creates a subclass of Node and uses std::bind() to register a member function as a callback from the timer. */

```
class MinimalPublisher : public rclcpp::Node 
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

```

* `class MinimalPublisher : public rclcpp::Node`: `MinimalPublisher` inherits from `rclcpp::Node`, making it a ROS 2 node
* `MinimalPublisher()`: constructor
* `: Node("minimal_publisher")`Calls the base class constructor to initialize the node with the name "minimal_publisher".
* `count_(0)`Initializes the `count_` variable to 0.
* `publisher_` initialization:
*`this->create_publisher<std_msgs::msg::String>("topic", 10)`: Creates a publisher that publishes messages of type `std_msgs::msg::String` on the topic named "topic" with a queue size of 10.
* `timer_` initialization: Creates a timer that calls the timer_callback function every 500 milliseconds

```
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); 
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```
| Command | Description |
| ---- | --- |
|`rclcpp::init(argc, argv);`|Initializes ROS 2|
|`rclcpp::spin(std::make_shared<MinimalPublisher>());`| Spins (runs) the MinimalSubscriber node|


* 2. add dependencies
 
open `package.xml` in the `ros2_ws/src/cpp_pubsub` directorty.
Add a new line after the `ament_cmake` buildtool dependency and paste the following dependencies corresponding to your node's include statements:

```
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/0c734d73-e374-42f1-8c01-0952aceb3d27)

This declares the package needs rclcpp and std_msgs when its code is built and executed.

remember to save the file.

* 3. CMakeList.txt

add lines after `find_package(ament_cmake REQUIRED)`:

```
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```
After that, add the executable and name it `talker` so you can run your node using `ros2 run`:

```
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
```

Finally, add the `install(TARGETS...)` section so `ros2 run` can find your executable:

```
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/fea59843-c352-482c-9245-d0504f298230)

### 3. write the subscriber node

navigate into `ros2_ws/src/cpp_pubsub/src`, and enter the command:

```
wget -O subscriber_member_function.cpp https://github.com/ChengHsunTai/ROS2/blob/89e3276ff9a9cd190afc218791eeb48a262dbfbf/project/subscriber_member_function.cpp
```

Reopen `CMakeLists.txt` and add the executable and target for the subscriber node below the publisher’s entries.

```
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

### 4. build and run

In the root of your workspace, `ros2_ws`, build your new package:

```
colcon build --packages-select cpp_pubsub
```

Open a new terminal, navigate to `ros2_ws`, source the setup file:

```
. install/setup.bash
```

run the talker node

```
ros2 run cpp_pubsub talker
```

Open another terminal, source the setup files from inside `ros2_ws` again, and then start the listener node:

```
ros2 run cpp_pubsub listener
```
### 5. The result:

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/244ee97b-2b0a-4d82-a48e-5d8b499d7f80)

* use `rqt_graph` to show the relationship between two node

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/9680bd36-8f34-4a49-a7fb-0bfd67c1b9c6)

As you can see, the topic named `topic` connect two node, and the message will be sent from publisher to subscriber.


## A guard dog
>In this section, I'll demonstrate two guard dogs communicating with each other by sending messages. If one dog stops sending messages, the other will continuously bark a warning. To accomplish this, I use two topics to connect the two nodes. Each node subscribes to the other's messages and issues a warning when messages are not received.

### 1. creating a new package

in the workspace root `ros2_ws`, enter the command:
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 guard_dog
```

picture!!!

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

picture!!!

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

