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

picture!!!

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

picture!!!

### 5. customize `package.xml`

to be conti

## Writing a simple publisher and subscriber (C++)





## C++ ROS2 package
### 1. use `colcon` to create C++ ROS2 package
 ```
ros2 pkg create --build-type ament_cmake <package_name>
```
 here name the package `test1`
 
 ![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/c3d02732-493a-4654-b104-e773ff2a3312)

### 2. type the command below to see the struture of the package you create
> if command `tree` not found. type the command `sudo apt install tree`, and try again.
```
tree -D  <package_name>
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/5b5f445b-33cc-432f-ab30-5a62f659f500)

The `include` and `src` directories are used to store the main source and header files of the program.

### 3. ceate a node

```
cd ~/test1/src
ros2 pkg create --build-type ament_cmake --node-name hello_world beginner_tutorials_cpp
```

picture!!!

in the `src` folder, conmmand `--node-name` to create a initial node, and it will also add the executable 
for you in the `CMakeLists.txt`. After creating the node, you'll see the folder named `beginner_tutorials_cpp` 
in the `src`.

you can see the structure of `beginner_tutorials_cpp` by using the command
```
tree beginner_tutorials_cpp
```

picture!!!

In this case, `hello_world__cpp` is the node. I won't using class in this project, so the forlder `beginner_tutorials_cpp`in the `include` folder is empty. Otherwise, it'll include a `hello_world.hpp`.  
