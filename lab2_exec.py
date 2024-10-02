#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 1
# Q11 = [120*pi/180.0, -56*pi/180.0, 124*pi/180.0, -158*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q12 = [120*pi/180.0, -64*pi/180.0, 123*pi/180.0, -148*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q13 = [120*pi/180.0, -72*pi/180.0, 120*pi/180.0, -137*pi/180.0, -90*pi/180.0, 0*pi/180.0]
#tower 1
Q10 = [174.65*pi/180.0, -102.68*pi/180.0, 124.10*pi/180.0, -111.25*pi/180.0, -89.82*pi/180.0, 54.40*pi/180.0]
Q11 = [174.50*pi/180.0, -87.18*pi/180.0, 139.18*pi/180.0, -141.82*pi/180.0, -89.91*pi/180.0, 54.42*pi/180.0]
Q12 = [174.47*pi/180.0, -76.46*pi/180.0, 142.56*pi/180.0, -155.93*pi/180.0, -89.97*pi/180.0, 54.45*pi/180.0]
Q13 = [174.69*pi/180.0, -63.74*pi/180.0, 143.86*pi/180.0, -169.94*pi/180.0, -90.03*pi/180.0, 54.68*pi/180.0]

#tower 2
Q20 = [194.04*pi/180.0, -88.80*pi/180.0, 119.68*pi/180.0, -120.67*pi/180.0, -89.74*pi/180.0, 73.85*pi/180.0]
Q21 = [193.90*pi/180.0, -78.88*pi/180.0, 128.63*pi/180.0, -139.55*pi/180.0, -89.81*pi/180.0, 73.81*pi/180.0]
Q22 = [193.40*pi/180.0, -69.38*pi/180.0, 131.02*pi/180.0, -151.42*pi/180.0, -89.86*pi/180.0, 73.36*pi/180.0]
Q23 = [193.86*pi/180.0, -61.11*pi/180.0, 132.76*pi/180.0, -161.44*pi/180.0, -89.90*pi/180.0, 73.87*pi/180.0]

Q30 = [207.69*pi/180.0, -78.95*pi/180.0, 104.77*pi/180.0, -115.59*pi/180.0, -89.65*pi/180.0, 87.46*pi/180.0]
Q31 = [207.19*pi/180.0, -68.84*pi/180.0, 114.64*pi/180.0, -135.57*pi/180.0, -89.71*pi/180.0, 87.10*pi/180.0]
Q32 = [207.17*pi/180.0, -62.69*pi/180.0, 116.85*pi/180.0, -143.93*pi/180.0, -89.74*pi/180.0, 87.13*pi/180.0]
Q33 = [207.16*pi/180.0, -54.94*pi/180.0, 117.96*pi/180.0, -152.78*pi/180.0, -89.78*pi/180.0, 87.15*pi/180.0]



thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
gripper_in = 1

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q10, Q11, Q12, Q13], \
      [Q20, Q21, Q22, Q23], \
      [Q30, Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def gripper_callback(msg):
    global gripper_in
    gripper_in =msg.AIN0
    pass



############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############
#loc is tower: 0,1,2
# height is block (highest) 0(above tower), 1,2,3 (lowest)
def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    error = 0
    success = 1
    start_dest = Q[start_loc][start_height]
    end_dest=  Q[end_loc][end_height]

    #move above start tower
    move_arm(pub_cmd, loop_rate, Q[start_loc][0], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, start_dest, 4.0, 4.0)
    #pick up
    gripper(pub_cmd, loop_rate, suction_on)
    # Delay to make sure suction cup has grasped the block
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, Q[start_loc][0], 4.0, 4.0)
    if gripper_in < 1.95:
        gripper(pub_cmd, loop_rate, suction_off)
        return error

    #move above end tower
    move_arm(pub_cmd, loop_rate, Q[end_loc][0], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, end_dest, 4.0, 4.0)
    #drop off
    gripper(pub_cmd, loop_rate, suction_off)
    # Delay to make sure suction cup has grasped the block
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, Q[end_loc][0], 4.0, 4.0)


    return success


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)


    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 0
    ABtower = 0
    start_tower = 0
    mid_tower = 0
    end_tower = 0
    # while(not input_done):
    #     input_string = input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
    #     print("You entered " + input_string + "\n")


    #     if(int(input_string) == 1):
    #         input_done = 1
    #         loop_count = 1
    #     elif (int(input_string) == 2):
    #         input_done = 1
    #         loop_count = 2
    #     elif (int(input_string) == 3):
    #         input_done = 1
    #         loop_count = 3
    #     elif (int(input_string) == 0):
    #         print("Quitting... ")
    #         sys.exit()
    #     else:
    #         print("Please just enter the character 1 2 3 or 0 to quit \n\n")
    while(not ABtower):
        ABtower = input("Enter Starting and End Tower (1,2,3)")
        ABtower = ABtower.strip()
        start_tower = int(ABtower[0])-1
        end_tower = int(ABtower[-1])-1
        mid_tower = 3 -start_tower - end_tower




    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

        
    if (not move_block(pub_command, loop_rate, start_tower, 1, end_tower, 3)):
        print("ERROR: BLOCK NOT FOUND!")
        return 0
    if (not move_block(pub_command, loop_rate, start_tower, 2, mid_tower, 3)):
        print("ERROR: BLOCK NOT FOUND!")
        return 0
    if (not move_block(pub_command, loop_rate, end_tower, 3, mid_tower, 2)):
        print("ERROR: BLOCK NOT FOUND!")
        return 0
    if (not move_block(pub_command, loop_rate, start_tower, 3, end_tower, 3)):
        print("ERROR: BLOCK NOT FOUND!")
        return 0
    if (not move_block(pub_command, loop_rate, mid_tower, 2, start_tower, 3)):
        print("ERROR: BLOCK NOT FOUND!")
        return 0
    if (not move_block(pub_command, loop_rate, mid_tower, 3, end_tower, 2)):
        print("ERROR: BLOCK NOT FOUND!")
        return 0
    if (not move_block(pub_command, loop_rate, start_tower, 3, end_tower, 1)):
        print("ERROR: BLOCK NOT FOUND!")
        return 0

    
    # while(loop_count > 0):

    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    #     rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)

    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

    #     rospy.loginfo("Sending goal 3 ...")
    #     move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

    #     loop_count = loop_count - 1

    # gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
