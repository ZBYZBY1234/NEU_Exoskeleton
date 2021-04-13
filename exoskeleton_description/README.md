## Rviz2 -> Joint State Real-Time Monitor

Running Rviz and showing the Robot Model in it.

Command:

```bash
$ ros2 launch exoskeleton_description view_robot_launch.py 
```

Then, running the joint_state_publisher node for import the urdf and getting the robot state based on it.

Command:

```bash
$ ros2 run joint_state_publisher joint_state_publisher
```

Finally, We can publish some message for the topic to make the robot have some changes.

Command:

```bash
ros2 topic pub -1 /topic std_msgs/msg/Float64MultiArray data:\ [-0.5,0.1,0.1,0.1,0.2,0.2,0.2,0.2]\ 
```
