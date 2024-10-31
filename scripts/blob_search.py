#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.0
beta = 0.0
tx = 0.0
ty = 0.0

# Function that converts image coord to world coord
def IMG2W(col, row):
    pass

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = True

    # Filter by Area.
    params.filterByArea = True

    # Filter by Circularity
    params.filterByCircularity = True

    # Filter by Inerita
    params.filterByInertia = True

    # Filter by Convexity
    params.filterByConvexity = True

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    lower = np.array([5,120,150])     # blue lower
    upper = np.array([13,140,180])   # blue upper
    normlower = lower/255
    Cmax = max(normlower)
    Cmin = min(normlower)
    delta = Cmax-Cmin
    if(Cmax == normlower[0]): #Blue max
       H_l = 30*((normlower[2]-normlower[1])/delta+4)
    elif(Cmax == normlower[1]):
       H_l = 30*((normlower[0]-normlower[1])/delta+2)
    else:
       H_l = 30*(((normlower[1]-normlower[0])/delta)%6)
    if Cmax != 0:
        S_l = delta/Cmax*100
    else:
        S_l = 0
    V_l = Cmax*100
    

    normupper = upper/255
    Cmax = max(normupper)
    Cmin = min(normupper)
    delta = Cmax-Cmin
    if(Cmax == normupper[0]): #Blue max
       H_u = 30*((normupper[2]-normupper[1])/delta+4)
    elif(Cmax == normupper[1]):
       H_u = 30*((normupper[0]-normupper[1])/delta+2)
    else:
       H_u = 30*(((normupper[1]-normupper[0])/delta)%6)
    if Cmax != 0:
        S_u = delta/Cmax*100
    else:
        S_u = 0
    V_u = Cmax*100


    lowerhsl =(45,80,80)#(H_l,S_l,V_l) #cv2.cvtColor(lower,cv2.COLOR_BGR2HSV)
    upperhsl =(60,230,230)#(H_u,S_u,V_u) #cv2.cvtColor(upper,cv2.COLOR_BGR2HSV)
    print(lowerhsl,"\n", upperhsl)
    # lowerhsl = (25,90,90)#cv2.cvtColor(lower,cv2.COLOR_BGR2HSV)
    # upperhsl = (35,255,150)#cv2.cvtColor(upper,cv2.COLOR_BGR2HSV)
    
    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lowerhsl, upperhsl)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints ,np.array([]))# (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)  

    return xw_yw
