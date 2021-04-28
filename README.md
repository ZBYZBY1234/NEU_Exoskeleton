# Exoskeleton --foxy

This package is built by `ROS2` foxy version, which aim to have the real-time control version.

|                     Version: NEUX 1.0.0                      |                      Build: Environment                      |                        Size: Packages                        |                   Language: Python && C++                    |
| :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: |
| ![](https://img.shields.io/github/v/release/MingshanHe/Graduate_Design?style=flat-square) | ![](https://img.shields.io/badge/build-passing-${color}.svg?style=flat-square) | ![](https://img.shields.io/github/repo-size/MingshanHe/Graduate_Design?style=flat-square) | ![](https://img.shields.io/badge/language-Python-green.svg?style=flat-square)  ![](https://img.shields.io/badge/language-C++-green.svg?style=flat-square) |

## Run Environment

* Operating System: Ubuntu 20.04     <code><img height="40" src="https://raw.githubusercontent.com/github/explore/80688e429a7d4ef2fca1e82350fe8e3517d3494d/topics/ubuntu/ubuntu.png" alt="ubuntu"></code>
* Robot Operate System: Foxy             <code><img height=40 src="Image/ros.jpeg" alt="ros"></code>

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

* IMU (Inertial Measurement Unit)
* Piezoelectric Module

## joint_state_publisher

This package is developed by official and can be used to control the joint by position mode.

## security_module

This package is mainly for programming the security module for this exoskeleton. And there are some tips for record data as csv file to do some treatment.