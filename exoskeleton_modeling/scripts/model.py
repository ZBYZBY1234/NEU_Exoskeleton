#!/usr/bin/env python
# coding=UTF-8
# /*
#  * @Author: MingshanHe
#  * @Email: hemingshan_1999@163.com
#  * @Date: 2021-01-13 06:51:13
#  * @Last Modified by:   MingshanHe
#  * @Last Modified time: 2021-01-13 06:51:13
#  * @Description: The main function for the modeling the robot
#  */

from packages import screw_method as sm
from packages import inverse_dynamics as inv_dy
import numpy as np
import rospy
if __name__ == '__main__':
    try:
        theta_list   = np.array([0, 0, 0, 0])
        dtheta_list  = np.array([0, 0, 0, 0])
        ddtheta_list = np.array([0, 0, 0, 0])
        print("T_end: ")
        sm.screw_method(theta_list)
        print("F_joint: ")
        inv_dy.inverse_dynamics(theta_list, dtheta_list, ddtheta_list)
    except rospy.ROSInterruptException:
        pass
