# NURC UROV

### Requirements:
- VSCode
- Docker

### Setup:
1. In VSCode, install the "Remote Development" extension. This will allow you to run the development environment
2. Click on the little arrows symbol in the bottom left corner of VSCode.
3. Click "Reopen in Container"
You should now be in the docker container with ROS 2 installed.

### Temporary roadmap:
1. Establish communication between laptop on surface and raspberry pi
2. Create thruster node - subscribe to movement commands, calculate individual motor speeds, publish PWM values to ESCs
3. Create vehicle state node - track depth, orientation, velocity

This is very subject to change as we start

### Additional resources
**Link to ROS 2 documentation**

https://docs.ros.org/en/jazzy/index.html

**Link to Northwestern's Master of Science in Robotics (MSR) website**

https://nu-msr.github.io

### Notes
Currently using ROS2 Jazzy, but might have to roll back to ROS2 Humble if needed

