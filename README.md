# NEU Exoskeleton (Northeastern University Exoskeleton)

 Developed by Human and Robot Collaboration Laboratory, Faculty of Robot Science and Engineering.

## Environment

* OS: Ubuntu 16.04
* ROS: Kinetic(Running Passed)

## Package List

This is a project for the powered lower exoskeleon.

* **exoskeleton_description**

This package is mainly about the description of powered lower exoskeleton and 
it is converted by SolidWorks2020 SW2URDF plugin.

* **powered_exoskeleton**

This package is mainly used for controlling the motors and the ouput command 
will connect the PC with motors by USB to CAN network, which need the command in
CAN control.

* **exoskeleton_control**

This package is based on the package of ros_control, which is widely used to 
control robot joint and others.

* **exoskeleton_modeling**

This package is developed by python launguage, and realized the algorithm about the 
screw method and dynamics.

* **exoskeleton_param**

This package is mainly to get the parameters for the control algorithms.

* **exoskeleton_matlab**

This package is mainly contributed in matlab environment and test the control algorithm and other modeling code or method.