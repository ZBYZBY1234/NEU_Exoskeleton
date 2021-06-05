#!/usr/bin/env python
# coding=UTF-8
# /*
#  * @Author: MingshanHe
#  * @Email: hemingshan_1999@163.com
#  * @Date: 2021-01-13 07:00:07
#  * @Last Modified by:   MingshanHe
#  * @Last Modified time: 2021-01-13 07:00:07
#  * @Description: This function uses forward-backward Newton-Euler
#  * iterations to solve the equation.
#  */
import numpy as np
import modern_robotics as mr
from packages import screw_method as sm
import rospy
'''
*** Function: inverse_dynamics
*** Input:    thetalist dthetalist ddthetalist
*** Return:   Get the Tau Matrix about every joint
'''
def inverse_dynamics(thetalist, dthetalist, ddthetalist):
    
    # theta: rad
    theta_hip   = thetalist[0]
    theta_thigh = thetalist[1]
    theta_knee  = thetalist[2]
    theta_ankle = thetalist[3]

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

    # Rodrigues_formula
    T_hip   = mr.Rodrigues_formula(omega_hip_se3,theta_hip)
    T_thigh = mr.Rodrigues_formula(omega_thigh_se3,theta_thigh)
    T_knee  = mr.Rodrigues_formula(omega_knee_se3,theta_knee)
    T_ankle = mr.Rodrigues_formula(omega_ankle_se3,theta_ankle)

    # Mass: kg
    hip_m   = 0.6272795491670899
    thigh_m = 1.9901518674612200
    knee_m  = 1.6727541009136200
    ankle_m = 1.1903645092283100

    # Moments of Inertia: kg*m^2
    hip_I_xx   = 0.000803551783618585
    hip_I_yy   = 0.001227800194699890
    hip_I_zz   = 0.000525693313567132
    thigh_I_xx = 0.008268562266969650 
    thigh_I_yy = 0.006907974105005490
    thigh_I_zz = 0.013386245019397700
    knee_I_xx  = 0.008835266568157909
    knee_I_yy  = 0.003617034724873650
    knee_I_zz  = 0.011469515175811100
    ankle_I_xx = 0.001530918183680350
    ankle_I_yy = 0.006480120450897900
    ankle_I_zz = 0.005987797998077430
    
    # Products of Inertia: kg*m^2
    hip_I_xy   = -2.98500736137851e-05
    hip_I_xz   = 2.29207102245784e-07
    hip_I_yz   = -5.17981149005877e-08
    thigh_I_xy = -0.0003572757112064
    thigh_I_xz = -0.000304599567683225
    thigh_I_yz = -0.00414176527978269
    knee_I_xy  = -0.000337215822027566
    knee_I_xz  = 0.000173170626109102
    knee_I_yz  = 0.00363950973911315
    ankle_I_xy = -7.19796708189506e-06
    ankle_I_xz = 6.995899556434e-05
    ankle_I_yz = -6.61160505400838e-06
    # inertia matrix: M or I_b
    hip_I   = np.array([[hip_I_xx, hip_I_xy, hip_I_xz],
                        [hip_I_xy, hip_I_yy, hip_I_yz],
                        [hip_I_xz, hip_I_yz, hip_I_zz]])
    thigh_I = np.array([[thigh_I_xx, thigh_I_xy, thigh_I_xz],
                        [thigh_I_xy, thigh_I_yy, thigh_I_yz],
                        [thigh_I_xz, thigh_I_yz, thigh_I_zz]])
    knee_I  = np.array([[knee_I_xx, knee_I_xy, knee_I_xz],
                        [knee_I_xy, knee_I_yy, knee_I_yz],
                        [knee_I_xz, knee_I_yz, knee_I_zz]])
    ankle_I = np.array([[ankle_I_xx, ankle_I_xy, ankle_I_xz],
                        [ankle_I_xy, ankle_I_yy, ankle_I_yz],
                        [ankle_I_xz, ankle_I_yz, ankle_I_zz]])
    # spatial inertia matrix: G
    hip_G   = np.diag([hip_I_xx, hip_I_yy, hip_I_zz, hip_m, hip_m, hip_m])
    thigh_G = np.diag([thigh_I_xx, thigh_I_yy, thigh_I_zz, thigh_m, thigh_m, thigh_m])
    knee_G  = np.diag([knee_I_xx, knee_I_yy, knee_I_zz, knee_m, knee_m, knee_m])
    ankle_G = np.diag([ankle_I_xx, ankle_I_yy, ankle_I_zz, ankle_m, ankle_m, ankle_m])
    Glist = np.array([hip_G, thigh_G, knee_G, ankle_G])
    # gravity coefficient
    g = np.array([0, 0, -9.8])
    # transfer matrix from the parent joint
    T_base  = sm.get_T('base',0)
    T_hip   = sm.get_T('hip',theta_hip)
    T_thigh = sm.get_T('thigh', theta_thigh)
    T_knee  = sm.get_T('knee', theta_knee)
    T_ankle = sm.get_T('ankle', theta_ankle)
    Mlist = np.array([T_base, T_hip, T_thigh, T_knee, T_ankle])
    # S matrix from the current joint
    S_hip   = sm.get_S('hip')
    S_thigh = sm.get_S('thigh')
    S_knee  = sm.get_S('knee')
    S_ankle = sm.get_S('ankle')
    Slist   = np.array([S_hip, S_thigh, S_knee, S_ankle]).T
    # out force
    Ftip    = np.array([0, 0, 0, 0, 0, 0])

    taulist = mr.InverseDynamics(thetalist, dthetalist, ddthetalist, g,Ftip, Mlist, Glist, Slist)
    Tau = taulist
    print(Tau)
    return Tau
