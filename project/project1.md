# Create your first project

## Workspace
> In ROS2, a project is generally veiwed as a unit and packaged into a folder, which is referred to as a Workspace `ws`.
![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/5c7d0ca6-6738-4c58-baba-d524c1d901db)


## use `colcon` to create C++ ROS2 package
 ```
ros2 pkg create --build-type ament_cmake <package_name>
```
 here name the package `test1`
 
 ![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/c3d02732-493a-4654-b104-e773ff2a3312)

## you cna take a look the structure of the package you create by the command below
> if command `tree` not found. type the command `sudo apt install tree`, and try again.
```
tree -D  <package_name>
```

![image](https://github.com/ChengHsunTai/ROS2/assets/137912642/5b5f445b-33cc-432f-ab30-5a62f659f500)

The `include` and `src` directories are used to store the main source and header files of the program.

