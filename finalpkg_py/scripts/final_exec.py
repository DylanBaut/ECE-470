#!/usr/bin/env python

import sys
import copy
import time
import rospy
import cv2

import numpy as np
from final_header import *
from final_func import *

from matplotlib import pyplot as plt


################ Pre-defined parameters and functions below (can change if needed) ################

# 20Hz
SPIN_RATE = 20  

# UR3 home location
home = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]  

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)  

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


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


"""
Function to control the suction cup on/off
"""
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

            rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel, move_type):
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
    driver_msg.move_type = move_type  # Move type (MoveJ or MoveL)
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

################ Pre-defined parameters and functions above (can change if needed) ################

##========= TODO: Helper Functions =========##

def find_keypoints(image):
    """Gets keypoints from the given image

    Parameters
    ----------
    image : np.ndarray
        The given image (before or after preprocessing)

    Returns
    -------
    keypoints
        a list of keypoints detected in image coordinates
  `  """
    # Convert to grayscale if needed
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()
    
    # Threshold to get black lines
    _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
    
    # Create a copy for visualization
    vis_image = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    
    # Get distance transform
    dist = cv2.distanceTransform(binary, cv2.DIST_L2, 3)
    
    # Threshold distance transform to get ridges (centerlines)
    _, ridges = cv2.threshold(dist, 1, 255, cv2.THRESH_BINARY)
    ridges = ridges.astype(np.uint8)
    
    # Find contours of the ridges
    contours, _ = cv2.findContours(ridges, cv2.RETR_LIST, 
                                 cv2.CHAIN_APPROX_NONE)
    
    keypoints = []
    min_contour_length = 30
    
    for contour in contours:
        if len(contour) < min_contour_length:
            continue
            
        # Approximate the contour to get main points
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, False)
        
        # Get points with consistent spacing
        for i in range(len(approx) - 1):
            p1 = tuple(approx[i][0])
            p2 = tuple(approx[i + 1][0])
            
            # Calculate distance between points
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            dist = np.sqrt(dx*dx + dy*dy)
            
            # Add points with fixed spacing
            if dist > 20:
                steps = int(dist / 20)
                for j in range(steps + 1):
                    t = j / steps
                    x = int(p1[0] + dx * t)
                    y = int(p1[1] + dy * t)
                    keypoints.append((x, y))
            else:
                keypoints.append(p1)
                
        # Add last point
        if len(approx) > 0:
            keypoints.append(tuple(approx[-1][0]))
    
    # Remove duplicates while maintaining order
    keypoints = list(dict.fromkeys(keypoints))
    
    # Draw debug visualizations
    debug_image = image.copy()
    if len(debug_image.shape) == 2:
        debug_image = cv2.cvtColor(debug_image, cv2.COLOR_GRAY2BGR)
    
    # Draw ridges
    ridge_image = debug_image.copy()
    cv2.drawContours(ridge_image, contours, -1, (0, 255, 0), 1)
    
    # Draw keypoints and connecting lines
    keypoint_image = debug_image.copy()
    
    # # Draw lines between consecutive points
    # for i in range(len(keypoints) - 1):
    #     pt1 = keypoints[i]
    #     pt2 = keypoints[i + 1]
    #     cv2.line(keypoint_image, pt1, pt2, (0, 255, 0), 1)
    
    # Draw keypoints
    for x, y in keypoints:
        cv2.circle(keypoint_image, (x, y), 2, (0, 0, 255), -1)
    
    # Save debug images
    cv2.imwrite('debug_1_binary.png', binary)
    cv2.imwrite('debug_2_ridges.png', ridges)
    cv2.imwrite('debug_3_centerlines.png', ridge_image)
    cv2.imwrite('debug_4_keypoints.png', keypoint_image)

    
    return keypoints

def IMG2W(row, col, image):
    """Transform image coordinates to world coordinates

    Parameters
    ----------
    row : int
        Pixel row position
    col : int
        Pixel column position
    image : np.ndarray
        The given image (before or after preprocessing)

    Returns
    -------
    x : float
        x position in the world frame
    y : float
        y position in the world frame
    """
    
    
    
    x, y = 0.0, 0.0
    return x, y

def draw_image(world_keypoints):
    """Draw the image based on detecte keypoints in world coordinates

    Parameters
    ----------
    world_keypoints:
        a list of keypoints detected in world coordinates
    """
    pass


def find_keypoints2(image):
    """Gets keypoints along center of lines with reduced density
    
    Parameters
    ----------
    image : np.ndarray
        The given image (before or after preprocessing)
    """
    # Convert to grayscale if needed
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()
    cv2.imwrite("2gray.png",gray)
    filtered = gray
    filtered = cv2.bilateralFilter(gray,5,100,9)
    # Threshold to ensure clean binary image
    # _, binary = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
    binary = cv2.adaptiveThreshold(filtered,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C ,cv2.THRESH_BINARY,11,-4)
    
    # Fill in the lines to get solid regions
    kernel = np.ones((3,3), np.uint8)
    filled = cv2.morphologyEx(binary, cv2.MORPH_CLOSE,kernel)
    
    # Get the distance transform
    dist = cv2.distanceTransform(filled, cv2.DIST_L2, 5)
    
    # Threshold the distance transform to get center line
    _, center = cv2.threshold(dist, 1, 255, cv2.THRESH_BINARY_INV)
    center = center.astype(np.uint8)
    
    # Find contours of the center line
    contours, _ = cv2.findContours(binary, cv2.RETR_LIST, 
                                 cv2.CHAIN_APPROX_TC89_L1)
    
    # Process contours to get keypoints
    keypoints = []
    min_length = 10  # Increased minimum length
    spacing = 20     # Increased spacing between points
    
    for contour in contours:
        if len(contour) < min_length:
            continue
            
        # Use contour approximation to reduce points
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, False)
        
        # Sample points with larger spacing
        for i in range(0, len(approx), spacing):
            point = approx[i][0]
            keypoints.append(tuple(point))
    ridge_image = image.copy()
    cv2.drawContours(ridge_image, contours, -1, (0, 255, 0), 1)

    # Create debug images
    debug_image = image.copy()
    if len(debug_image.shape) == 2:
        debug_image = cv2.cvtColor(debug_image, cv2.COLOR_GRAY2BGR)
    
    # Draw keypoints
    keypoint_image = debug_image.copy()
    for x, y in keypoints:
        cv2.circle(keypoint_image, (x, y), 3, (0, 0, 255), -1)
        
    # Save debug images
    cv2.imwrite('2debug_1_binary.png', binary)
    cv2.imwrite('2debug_1_bilateral.png', filtered)
    cv2.imwrite('2debug_2_contours.png', ridge_image)
    cv2.imwrite('2debug_2_filled.png', filled)
    cv2.imwrite('2debug_3_center.png', center)
    cv2.imwrite('2debug_4_keypoints.png', keypoint_image)
    return keypoints
"""
Program run from here
"""
def main():
    global home
    # global variable1
    # global variable2

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    # Velocity and acceleration of the UR3 arm
    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, home, vel, accel, 'J')  # Move to the home position

    ##========= TODO: Read and draw a given image =========##
     


    move_arm(pub_command, loop_rate, home, vel, accel, 'J')  # Return to the home position
    turkey = cv2.imread("images/turkey.png")
    zigzag = cv2.imread("images/zigzag.jpg")
    status= cv2.imread("images/status.png")
    find_keypoints(status)
    find_keypoints2(status)
    # draw_image()
    
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()
if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
