# Create your first project

## Workspace
> In ROS2, a project is generally veiwed as a unit and packaged into a folder, which is referred to as a Workspace `ws`.
![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/5c7d0ca6-6738-4c58-baba-d524c1d901db)


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

### 3. ceeate a node

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
