# NEU Exoskeleton (Northeastern University Exoskeleton)

<img src="/home/hemingshan/exo_ws/src/Image/Badge.jpeg" style="zoom:25%;" />

Developed by Human and Robot Collaboration Laboratory, Faculty of Robot Science and Engineering.

This package is built by `ROS2` foxy version, which aim to have the real-time control version.

|                     Version: NEUX 1.0.0                      |                      Build: Environment                      |                        Size: Packages                        |                   Language: Python && C++                    |
| :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: |
| ![](https://img.shields.io/github/v/release/MingshanHe/Graduate_Design?style=flat-square) | ![](https://img.shields.io/badge/build-passing-${color}.svg?style=flat-square) | ![](https://img.shields.io/github/repo-size/MingshanHe/Graduate_Design?style=flat-square) | ![](https://img.shields.io/badge/language-Python-green.svg?style=flat-square)  ![](https://img.shields.io/badge/language-C++-green.svg?style=flat-square) |

## Run Environment

* Operating System: Ubuntu 20.04     <code><img height="40" src="https://raw.githubusercontent.com/github/explore/80688e429a7d4ef2fca1e82350fe8e3517d3494d/topics/ubuntu/ubuntu.png" alt="ubuntu"></code>
* Robot Operate System: Foxy             <code><img height=40 src="Image/ros.jpeg" alt="ros"></code>

## Consist Packages of Project

### exoskeleton_description

This package is mainly about the `rviz2` for showing the state of the exoskeleton robot and visualization. The command is mainly like following and run it under the root folder in the package.

```bash
$ ros2 launch exoskeleton_description view_robot_launch.py
```

And this will run the `rviz2` and visual the robot under it.
### exoskeleton_control

This package is mainly about the **Control Algorithm** which could be used in exoskeleton robot. I have mainly designed the Admittance and Impedance control. There are have two mode for this Project: Software simulation and Hardware simulation.

### sensor_module

This package is mainly about the Sensor to get the data of the Control Algorithm Inputs.

* IMU (Inertial Measurement Unit)
* Piezoelectric Module

### joint_state_publisher

This package is developed by official and can be used to control the joint by position mode.

### security_module

This package is mainly for programming the security module for this exoskeleton. And there are some tips for record data as csv file to do some treatment.

### gait_recognition

This package is mainly for recognize the gait by real-time. So It's mainly for the data filter and Neural Network which is used to make some recognition.

## Run

### Hard Ware Mode

This mode will need the IMU and Piezoelectric Sensors for get the human body state. So at first, I need to run the launch file for running the sensors.

```bash
$ ros2 launch sensor_module exoskeleton_sensor_launch.py
$ ros2 launch sensor_module human_sensor_launch.py
```

The first command is mainly for running the IMU Sensor which is at the link of the exoskeleton, the second command is mainly for running the IMU and Piezoelectric Sensors which is at the link of the human body.

Next, it need to run the Motor Driver to drive the motor to turn. The command is

```bash
$ ros2 run exoskeleton_control Angle_Driver
```

Lastly, the control algorithm is used Admittance Control and I have programmed it, can run it as

```bash
$ ros2 run exoskeleton_control Admittance_control_Hardware
```

For Data Visualization, the command can record the data which you needed.

```bash
$ ros2 run security_module Joint_Record
```

And there are some codes for visualizing the data of Sensor and Angle Driver



## CSV File Description (security_module/csv_file)

This will give a description for the file.

* 1.csv
* 2(input and record).csv: This file is mainly create by the Joint_input which is in the security_module and give the motor as the range of 30 degree and to check the error and time.
* 3.csv: This file is mainly create by the Joint_Record_25 which is record the data in the mode of free time.