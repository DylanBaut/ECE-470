#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	# q_1 = np.array([[150],[150],[40]])
	q_1 = np.array([[-150],[150],[10]])
	w_1 = np.array([[0],[0],[1]])		
	v_1 = np.cross(-w_1.flatten(),q_1.flatten())
	w_1 = [[0,-w_1.flatten()[2],w_1.flatten()[1]],
		   [w_1.flatten()[2],0,-w_1.flatten()[0]],
		   [-w_1.flatten()[1],w_1.flatten()[0],0  ]]
	
	# q_2 = np.array([[150],[270],[170]])
	q_2 = np.array([[-150],[150+120],[162]])
	w_2 = np.array([[0],[1],[0]])
	v_2 = np.cross(-w_2.flatten(),q_2.flatten())
	w_2 = [[0,-w_2.flatten()[2],w_2.flatten()[1]],
		   [w_2.flatten()[2],0,-w_2.flatten()[0]],
		   [-w_2.flatten()[1],w_2.flatten()[0],0  ]]


	# q_3 = np.array([[120],[270],[170]])
	q_3 = np.array([[244-150],[270],[162]])
	w_3 = np.array([[0],[1],[0]])
	v_3 = np.cross(-w_3.flatten(),q_3.flatten())
	w_3 = [[0,-w_3.flatten()[2],w_3.flatten()[1]],
		   [w_3.flatten()[2],0,-w_3.flatten()[0]],
		   [-w_3.flatten()[1],w_3.flatten()[0],0  ]]
	


	# q_4 = np.array([[320],[170],[170]])
	q_4 = np.array([[244-150+213],[270-93],[162]])
	w_4 = np.array([[0],[1],[0]])
	v_4 = np.cross(-w_4.flatten(),q_4.flatten())
	w_4 = [[0,-w_4.flatten()[2],w_4.flatten()[1]],
		   [w_4.flatten()[2],0,-w_4.flatten()[0]],
		   [-w_4.flatten()[1],w_4.flatten()[0],0  ]]


	# q_5 = np.array([[320],[250],[170]])
	q_5 = np.array([[244-150+213],[270-93+83],[162]])
	w_5 = np.array([[1],[0],[0]])
	v_5 = np.cross(-w_5.flatten(),q_5.flatten())
	w_5 = [[0,-w_5.flatten()[2],w_5.flatten()[1]],
		   [w_5.flatten()[2],0,-w_5.flatten()[0]],
		   [-w_5.flatten()[1],w_5.flatten()[0],0  ]]

	# q_6 = np.array([[410],[170],[170]])
	q_6 = np.array([[244-150+213+83],[270-93+83],[162]])
	w_6 = np.array([[0],[1],[0]])
	v_6 = np.cross(-w_6.flatten(),q_6.flatten())
	w_6 = [[0,-w_6.flatten()[2],w_6.flatten()[1]],
		   [w_6.flatten()[2],0,-w_6.flatten()[0]],
		   [-w_6.flatten()[1],w_6.flatten()[0],0  ]]

	# print(w_1,v_1.T)
	print(np.vstack((np.hstack((w_1,v_1.reshape(3,1))),[0,0,0,0])))
	s_1 = np.vstack((np.hstack((w_1,v_1.reshape(3,1))),[0,0,0,0]))
	s_2 = np.vstack((np.hstack((w_2,v_2.reshape(3,1))),[0,0,0,0]))
	s_3 = np.vstack((np.hstack((w_3,v_3.reshape(3,1))),[0,0,0,0]))
	s_4 = np.vstack((np.hstack((w_4,v_4.reshape(3,1))),[0,0,0,0]))
	s_5 = np.vstack((np.hstack((w_5,v_5.reshape(3,1))),[0,0,0,0]))
	s_6 = np.vstack((np.hstack((w_6,v_6.reshape(3,1))),[0,0,0,0]))
	S = [s_1,s_2,s_3,s_4,s_5,s_6]
	# print(s_1)
	M = np.array([[0, -1, 0,q_6.flatten()[0]], [0, 0, -1, q_6.flatten()[1]+82+59], [1, 0 ,0 , q_6.flatten()[2]+53.5],[0,0,0,1]])
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
	







	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
