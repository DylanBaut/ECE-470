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
def sort_points_for_drawing(points):
    """Sort points efficiently using NumPy vectorization."""
    
    
    points = np.asarray(points)
    n_points = len(points)
    
    sorted_indices = np.zeros(n_points, dtype=np.int32)
    mask = np.ones(n_points, dtype=bool)
    
    start_idx = np.argmin(points[:, 0])
    current_idx = start_idx
    sorted_indices[0] = start_idx
    mask[start_idx] = False
    
    for i in range(1, n_points):
        current = points[current_idx]
        
        distances = np.sum((points[mask] - current) ** 2, axis=1)
        
        next_idx = np.where(mask)[0][np.argmin(distances)]        
        sorted_indices[i] = next_idx
        mask[next_idx] = False
        current_idx = next_idx
    
    # Return sorted points as tuples
    return [tuple(map(int, p)) for p in points[sorted_indices]]

def find_keypoints2(image):
    """Gets keypoints along center of lines with reduced density
    
    Parameters
    ----------
    image : np.ndarray
        The given image (before or after preprocessing)
    """
    # Convert to grayscale if needed
    resized_image = cv2.resize(image, (319*2,239*2), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

    # Threshold to ensure clean binary image
    # _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, -5)
    
    # Threshold the distance transform to get center line
    _, center = cv2.threshold(binary, 1, 255, cv2.THRESH_BINARY)
    center = center.astype(np.uint8)
    canny = cv2.Canny(center,1,255)
    crosskernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(5,5))
    fill = cv2.dilate(canny,crosskernel,iterations = 3)
    # Find contours of the center line
    contours, _ = cv2.findContours(canny, cv2.RETR_LIST, 
                                 cv2.CHAIN_APPROX_TC89_KCOS)
    height, width = center.shape[:2]
    
    # Process contours to get keypoints
    keypoints = []
    keypoints2 = np.array([])
    min_length = 0  # Increased minimum length
    spacing = 60  # Increased spacing between points
    for contour in contours:
        if len(contour) < min_length:
            continue
        epsilon = 0.0001 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)     
        # keypoints2.append(approx.reshape(-1,2))   
        print(len(approx))
        for i in range(0, len(approx), spacing):
            # point = approx[i][0]
            point = contour[i][0]
            # keypoints.append(tuple(point))
            keypoints.append(tuple(point))
    print(len(keypoints))
    # Create debug images
    height, width = resized_image.shape[:2]
    recreated_image = np.ones((height, width, 3), dtype=np.uint8) * 255  # White background
    # Draw lines connecting keypoints
    if len(keypoints) > 1:
        for i in range(0, len(keypoints) - 1):  # Changed to -1 to avoid going past end
            start_point = keypoints[i]
            end_point = keypoints[i + 1]
            cv2.line(recreated_image, start_point, end_point, (0, 0, 255), 1)
    debug_image = image.copy()
    if len(debug_image.shape) == 2:
        debug_image = cv2.cvtColor(debug_image, cv2.COLOR_GRAY2BGR)
    
    # Draw keypoints
    keypoint_image = debug_image.copy()
    finalkeypoints = sort_points_for_drawing(keypoints)
    
    for x, y in finalkeypoints:
        cv2.circle(keypoint_image, (x, y), 3, (0, 0, 255), -1)
    # Save debug images
    cv2.drawContours(recreated_image, contours, -1, (0,255,0), 3)
    cv2.imwrite('2debug_6_fill.png',fill)
    cv2.imwrite('2debug_5_cany.png',canny)
    cv2.imwrite('2debug_4_draw.png', recreated_image)
    cv2.imwrite('2debug_1_binary.png', binary)
    cv2.imwrite('2debug_2_center.png',center)
    cv2.imwrite('2debug_3_keypoints.png', keypoint_image)
    cv2.imwrite('2debug_7_keypoints.png', resized_image)
    
    return finalkeypoints

def IMG2W(col, row):
    # Beta = 760
    # theta = 1.508*np.pi/180
    # Ty =.08
    # Tx =.300

    Or= 239
    Oc = 319
    xc = (row)/(Or*2)*.21+0.17
    yc=  (col)/(Oc*2)*.28+0.025
    # print("XC,YC: ",xc,yc, "\n \n \n")
    # xw = (xc+Tx)*np.cos(theta)-(yc+Ty)*np.sin(theta)
    # yw = (xc+Tx)*np.sin(theta)+(yc+Ty)*np.cos(theta)
    # # print("World Coordinates:",xw,",",yw)
    return(xc,yc)

def draw_image(world_keypoints):
    """Draw the image based on detecte keypoints in world coordinates

    Parameters
    ----------
    world_keypoints:
        a list of keypoints detected in world coordinates
    """
    draw_keypoints = [IMG2W(i[0],i[1]) for i in world_keypoints]
    t = len(draw_keypoints)
    for point  in draw_keypoints:
        t -= 1
        angles = lab_invk(point[0],point[1],0.03,90)
        move_arm(pub_command, loop_rate, angles, 3.0, 2.0, 'L')
        print(t," Points left")
    return


def main():
    global home
    global pub_command
    global loop_rate

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
     


    # move_arm(pub_command, loop_rate, home, vel, accel, 'J')  # Return to the home position
    turkey = cv2.imread("images/turkey.png")
    zigzag = cv2.imread("images/zigzag.jpg")
    rob= cv2.imread("images/rob.png")
    print("get data")
    keypoints = find_keypoints2(rob)
    # find_keypoints2(status)
    draw_image(keypoints)
    
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()
if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
