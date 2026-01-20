# Creating a ROS2 package
First, make sure you are in the devcontainer with the ROS2 environment active.

To create a Python package, in the `UROV/src` directory, run the following command

```
ros2 pkg create --build-type ament_python --license Apache-2.0 PACKAGE_NAME
```

To create a C++ package, in the `UROV/src` directory, run the following command

```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 PACKAGE_NAME`
```
