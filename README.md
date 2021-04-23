# Exoskeleton --foxy

This package is built by `ROS2` foxy version, which aim to have the real-time control version.

|                     Version: NEUX 1.0.0                      |                  Environment: Ubuntu 20.04                   |      |      |
| :----------------------------------------------------------: | :----------------------------------------------------------: | ---- | ---- |
| ![](https://img.shields.io/github/v/release/MingshanHe/Graduate_Design?style=flat-square) | ![](https://img.shields.io/travis/MingshanHe/robotictool?style=flat-square) |      |      |



## exoskeleton_description

This package is mainly about the `rviz2` for showing the state of the exoskeleton robot and visualization. The command is mainly like following and run it under the root folder in the package.

```bash
$ ros2 launch exoskeleton_description view_robot_launch.py
```

And this will run the `rviz2` and visual the robot under it.
## exoskeleton_control

This package is mainly about the **Control Algorithm** which could be used in exoskeleton robot. I have mainly designed the Admittance and Impedance control. There are have two mode for this Project: Software simulation and Hardware simulation.

## sensor_module

This package is mainly about the Sensor to get the data of the Control Algorithm Inputs.

## joint_state_publisher

This package is developed by official and can be used to control the joint by position mode.