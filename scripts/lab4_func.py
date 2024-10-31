#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	# M = np.eye(4)
	# S = np.zeros((6,6))
	q_1 = np.array([[-150],[150],[10]])*.001
	w_1 = np.array([[0],[0],[1]])
	v_1 = np.cross(-w_1.flatten(),q_1.flatten())
	w_1 = [[0,-w_1.flatten()[2],w_1.flatten()[1]],
		   [w_1.flatten()[2],0,-w_1.flatten()[0]],
		   [-w_1.flatten()[1],w_1.flatten()[0],0  ]]
	
	# q_2 = np.array([[150],[270],[170]])
	q_2 = np.array([[-150],[150+120],[162]])*.001
	w_2 = np.array([[0],[1],[0]])
	v_2 = np.cross(-w_2.flatten(),q_2.flatten())
	w_2 = [[0,-w_2.flatten()[2],w_2.flatten()[1]],
		   [w_2.flatten()[2],0,-w_2.flatten()[0]],
		   [-w_2.flatten()[1],w_2.flatten()[0],0  ]]


	# q_3 = np.array([[120],[270],[170]])
	q_3 = np.array([[244-150],[270],[162]])*.001
	w_3 = np.array([[0],[1],[0]])
	v_3 = np.cross(-w_3.flatten(),q_3.flatten())
	w_3 = [[0,-w_3.flatten()[2],w_3.flatten()[1]],
		   [w_3.flatten()[2],0,-w_3.flatten()[0]],
		   [-w_3.flatten()[1],w_3.flatten()[0],0  ]]
	


	# q_4 = np.array([[320],[170],[170]])
	q_4 = np.array([[244-150+213],[270-93],[162]])*.001
	w_4 = np.array([[0],[1],[0]])
	v_4 = np.cross(-w_4.flatten(),q_4.flatten())
	w_4 = [[0,-w_4.flatten()[2],w_4.flatten()[1]],
		   [w_4.flatten()[2],0,-w_4.flatten()[0]],
		   [-w_4.flatten()[1],w_4.flatten()[0],0  ]]


	# q_5 = np.array([[320],[250],[170]])
	q_5 = np.array([[244-150+213],[270-93+83],[162]])*.001
	w_5 = np.array([[1],[0],[0]])
	v_5 = np.cross(-w_5.flatten(),q_5.flatten())
	w_5 = [[0,-w_5.flatten()[2],w_5.flatten()[1]],
		   [w_5.flatten()[2],0,-w_5.flatten()[0]],
		   [-w_5.flatten()[1],w_5.flatten()[0],0  ]]

	# q_6 = np.array([[410],[170],[170]])
	q_6 = np.array([[244-150+213+83],[270-93+83],[162]])*.001
	w_6 = np.array([[0],[1],[0]])
	v_6 = np.cross(-w_6.flatten(),q_6.flatten())
	w_6 = [[0,-w_6.flatten()[2],w_6.flatten()[1]],
		   [w_6.flatten()[2],0,-w_6.flatten()[0]],
		   [-w_6.flatten()[1],w_6.flatten()[0],0  ]]

	# print(w_1,v_1.T)
	# print(np.vstack((np.hstack((w_1,v_1.reshape(3,1))),[0,0,0,0])))
	s_1 = np.vstack((np.hstack((w_1,v_1.reshape(3,1))),[0,0,0,0]))
	s_2 = np.vstack((np.hstack((w_2,v_2.reshape(3,1))),[0,0,0,0]))
	s_3 = np.vstack((np.hstack((w_3,v_3.reshape(3,1))),[0,0,0,0]))
	s_4 = np.vstack((np.hstack((w_4,v_4.reshape(3,1))),[0,0,0,0]))
	s_5 = np.vstack((np.hstack((w_5,v_5.reshape(3,1))),[0,0,0,0]))
	s_6 = np.vstack((np.hstack((w_6,v_6.reshape(3,1))),[0,0,0,0]))
	S = [s_1,s_2,s_3,s_4,s_5,s_6]
	# print(s_1)
	M = np.array([[0, -1, 0,q_6.flatten()[0]], [0, 0, -1, q_6.flatten()[1]+.082+.059], [1, 0 ,0 , q_6.flatten()[2]+.0535],[0,0,0,1]])



	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):



	# Initialize the return_value

	return_value = [None, None, None, None, None, None]



	# =========== Implement joint angle to encoder expressions here ===========

	print("Foward kinematics calculated:\n")



	# =================== Your code starts here ====================#



	M,S = Get_MS()

	# print(S[0])

	T = expm(theta1*S[0])@expm(theta2*S[1])@expm(theta3*S[2])@expm(theta4*S[3])@expm(theta5*S[4])@expm(theta6*S[5])@M


	print(str(T) + "\n")



	return_value[0] = theta1 + PI

	return_value[1] = theta2

	return_value[2] = theta3

	return_value[3] = theta4 - (0.5*PI)

	return_value[4] = theta5

	return_value[5] = theta6


	print(return_value)
	return return_value



"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	l1 = .152
	l2 = .12
	l6 = .083
	l3 = .244
	l5 = .213
	l7 = .083
	xgrip = xWgrip+.150 
	ygrip = yWgrip-.150
	zgrip = zWgrip-.010
	yaw_W_Rads = yaw_WgripDegree*(PI/180)
	xcen = xgrip - np.cos(yaw_W_Rads)*.0535
	ycen = ygrip - np.sin(yaw_W_Rads)*.0535
	zcen = zgrip
	# atheta = np.arccos((l2**2-(l3+l5+l7)**2-(xcen**2+ycen**2))/(-2*(l3+l5+l7)*(np.sqrt(xcen**2+ycen**2)))) 
	# c1 = np.sqrt((l7**2)+(.027+l6)**2)
	xtheta = 2*np.pi- np.arctan((.027+l6)/(l7))
	ktheta = np.pi - np.arctan((l7)/(.027+l6))
	atheta = 2*np.pi - ktheta-xtheta

	theta1 = np.arctan2(ycen,xcen)-atheta

	x3 = .137*np.sin(.6405-theta1)-xcen
	y3 = .137*np.cos(.6405-theta1)-ycen
	z3 = .059+ .082
	th2C = np.sqrt((z3-l1)**2+(x3**2+y3**2))

	tsmall = (l5**2-th2C**2-l3**2)/(2*th2C*l3)

	smalltheta2 = np.arccos(tsmall)
	theta2 = (np.arctan((z3-l1)/(np.sqrt(x3**2+y3**2)))+smalltheta2)
	theta3 = np.pi-np.arccos((th2C**2-l3**2-l5**2)/(2*l5*l3))
	theta4 = np.pi-smalltheta2+theta3-(np.pi/2)
	theta5 = -np.pi/2
	theta6 = np.pi/2+theta1-yaw_W_Rads 

	print(theta1,"\n",theta2,"\n",theta3,"\n",theta4,"\n",theta5,"\n",theta6)
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
