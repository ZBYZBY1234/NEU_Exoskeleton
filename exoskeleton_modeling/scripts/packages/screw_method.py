#!/usr/bin/env python
# coding=UTF-8
# /*
#  * @Author: MingshanHe
#  * @Email: hemingshan_1999@163.com
#  * @Date: 2021-01-13 06:24:57
#  * @Last Modified by:   MingshanHe
#  * @Last Modified time: 2021-01-13 06:24:57
#  * @Description: Modelling Method
#  */

import numpy as np
import modern_robotics as mr
import rospy

# revolute axis in the world frame
omega_hip   = np.array([0, 0, 1])
omega_thigh = np.array([0, 1, 0]) 
omega_knee  = np.array([0, 1, 0])
omega_ankle = np.array([0, 1, 0])
# P: m, discripted in the base_link frame
p_hip   = np.array([ 0.22430,  0.20800, -0.28300])
p_thigh = np.array([ 0.00000, -0.01200, -0.09700])
p_knee  = np.array([ 0.23966,  0.00000, -0.28189])
p_ankle = np.array([-0.15989, -0.02500, -0.31135])
# v: m/rad
v_hip   = np.cross(-omega_hip  , p_hip)
v_thigh = np.cross(-omega_thigh, p_thigh)
v_knee  = np.cross(-omega_knee , p_knee)
v_ankle = np.cross(-omega_ankle, p_ankle)
# S: screw value
S_hip   = np.append(omega_hip   ,v_hip)
S_thigh = np.append(omega_thigh ,v_thigh)
S_knee  = np.append(omega_knee  ,v_knee)
S_ankle = np.append(omega_ankle ,v_ankle)
# Vec to se3
omega_hip_se3   = mr.VecTose3(S_hip)
omega_thigh_se3 = mr.VecTose3(S_thigh)
omega_knee_se3  = mr.VecTose3(S_knee)
omega_ankle_se3 = mr.VecTose3(S_ankle)
'''
*** Function: get_S
*** Input:    joint_name
*** Return:   the S matrix by screw
'''
def get_S(joint_name):
    if joint_name=='hip':
        S = np.append(omega_hip   ,v_hip)
        return S
    elif joint_name=='thigh':
        S = np.append(omega_thigh ,v_thigh)
        return S
    elif joint_name=='knee':
        S = np.append(omega_knee  ,v_knee)
        return S
    elif joint_name=='ankle':
        S = np.append(omega_ankle ,v_ankle)
        return S
    else:
        print("Error Input the Joint Name.")
        pass
'''
*** Function: get_T by Rodrigues_formula
*** Input:    joint_name
*** Return:   the matrix from parent joint to the joint
'''
def get_T(joint_name,theta):
    if joint_name=='hip':
        T   = mr.Rodrigues_formula(omega_hip_se3,theta)
        return T
    elif joint_name=='thigh':
        T = mr.Rodrigues_formula(omega_thigh_se3,theta)
        return T
    elif joint_name=='knee':
        T  = mr.Rodrigues_formula(omega_knee_se3,theta)
        return T
    elif joint_name=='ankle':
        T = mr.Rodrigues_formula(omega_ankle_se3,theta)
        return T
    elif joint_name=='base':
        T = np.eye(4)
        return T
    else:
        print("Error Input the Joint Name.")
        pass

'''
*** Function: screw_method by screw
*** Input:    theta_list
*** Return:   Get the T_end Matrix by the screw modeling method
'''
def screw_method(theta_list):
    # theta: rad
    theta_hip   = theta_list[0]
    theta_thigh = theta_list[1]
    theta_knee  =  theta_list[2]
    theta_ankle = theta_list[3]
    # Rodrigues_formula
    T_hip   = mr.Rodrigues_formula(omega_hip_se3,theta_hip)
    T_thigh = mr.Rodrigues_formula(omega_thigh_se3,theta_thigh)
    T_knee  = mr.Rodrigues_formula(omega_knee_se3,theta_knee)
    T_ankle = mr.Rodrigues_formula(omega_ankle_se3,theta_ankle)

    # M: original attitude matrix
    M_R = np.array([[1,  0, 0],
                    [0,  0, 1],
                    [0, -1, 0]])
    M_p = p_hip + p_thigh + p_knee + p_ankle
    M = mr.RpToTrans(M_R, M_p)

    # %T:末端矩阵
    T = np.dot(T_hip,np.dot(T_thigh,np.dot(T_knee,np.dot(T_ankle,M))))
    print(T)
    return T