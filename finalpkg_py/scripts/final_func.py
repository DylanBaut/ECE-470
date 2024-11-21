#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from final_header import *  

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""


def Get_MS():
    # =================== Code from Lab 5 ====================#
    # Fill in the correct values for w1~6 and v1~6, as well as the M matrix
    M = np.eye(4)
    S = np.zeros((6, 6))

    # ==============================================================#
    return M, S


"""
Function that calculates encoder numbers for each motor
"""


def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    # Initialize the return_value
    return_value = [None, None, None, None, None, None]

    print("Foward kinematics calculated:\n")

    # =================== Your code starts here ====================#
    theta = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
    T = np.eye(4)

    M, S = Get_MS()

    # ==============================================================#

    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5 * PI)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""


def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    # =================== Your code starts here ====================#
    l1 = 0.152
    l2 = 0.12
    l3 = 0.244
    l4 = 0.093
    l5 = 0.213
    l6 = 0.083
    l7 = 0.083
    l8 = 0.082
    l9 = 0.053
    l10 = 0.05
    xgoal = xWgrip + 0.150
    ygoal = yWgrip - 0.150
    zgoal = zWgrip - 0.010
    yaw_W_Rads = np.deg2rad(yaw_WgripDegree)

    xcen = xgoal - l9 * np.cos(yaw_W_Rads)
    ycen = ygoal - l9 * np.sin(yaw_W_Rads)
    zcen = zgoal
    # atheta = np.arccos((l2**2-(l3+l5+l7)**2-(xcen**2+ycen**2))/(-2*(l3+l5+l7)*(np.sqrt(xcen**2+ycen**2))))
    # c1 = np.sqrt((l7**2)+(.027+l6)**2)
    offset = l2 - l4 + l6
    c_theta1 = np.sqrt(xcen**2 + ycen**2)
    atheta = np.arcsin(offset / c_theta1)
    theta1 = np.arctan2(ycen, xcen) - atheta
    # x3 = .137*np.sin(.6405-theta1)-xcen
    to_virtual = np.sqrt(xcen**2 + ycen**2 - offset**2) - l7
    x3 = np.cos(theta1) * to_virtual
    y3 = np.sin(theta1) * to_virtual
    z3 = zcen + l8 + l10
    # print(x3,y3,z3)
    th2C = np.arctan2(z3 - l1, (x3**2 + y3**2))
    dist_to_eff = np.sqrt(x3**2 + y3**2 + (z3 - l1) ** 2)
    smalltheta2 = np.arccos((l3**2 + dist_to_eff**2 - l5**2) / (2 * l3 * dist_to_eff))

    theta2 = -(th2C + smalltheta2)
    theta3 = np.pi - np.arccos((dist_to_eff**2 - l3**2 - l5**2) / (-2 * l3 * l5))
    theta4 = -(theta2 + theta3)
    theta5 = -np.pi / 2
    theta6 = np.pi / 2 + theta1 - yaw_W_Rads

    print(theta1, "\n", theta2, "\n", theta3, "\n", theta4, "\n", theta5, "\n", theta6)

    # ==============================================================#
    return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
