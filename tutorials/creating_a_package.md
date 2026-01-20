# Creating a ROS2 package
First, make sure you are in the devcontainer with the ROS2 environment active.

To create a Python package, in the `UROV/src` directory, run the following command to create a Python package
```
ros2 pkg create --build-type ament_python --license Apache-2.0 PACKAGE_NAME
```
To create a C++ package, in the `UROV/src` directory, run the following command
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 PACKAGE_NAME`
```
Now, we should have a new folder in `UROV/src` with the name of our package. From here, just write the code for your package. It might also be worth modifying the `package.xml` file and fill out the `maintainer` and `description` fields. Once you are ready to build it, go to the `UROV` directory (the root for this project) and run the below command to rebuild all packages.
```
colcon build
```
Optionally, you can run this to build only a select package
```
colcon build --packages-select PACKAGE_NAME
```
From here, source the setup file with one of the two following
```
source install/setup.bash
```
or 
```
source install/local_setup.bash
```
Lastly, run your package with
```
ros2 run PACKAGE_NAME NODE_NAME
```
