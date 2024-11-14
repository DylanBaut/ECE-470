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
    Beta = 760
    theta = 1.508*np.pi/180
    Tx =.09
    Ty =.215

    Or= 180
    Oc = 324
    xc = (row-Oc)/Beta
    yc=  (col-Or)/Beta
    xw = (xc+Tx)*np.cos(theta)
    yw = (yc+Ty)*np.sin(theta)
    print("Camera Coordinates:",xc,",",yc)
    return(xw,yw)




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
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    # ========================= Student's code starts here =========================
    lower = np.uint8([0,60,30])     # blue lower
    upper = np.uint8([10,140,60])   # blue upper
    normlower = lower
    Cmax = max(normlower)
    Cmin = min(normlower)
    delta = Cmax-Cmin
    if(Cmax == normlower[0]): #Blue max
       H_l = 240+60*((normlower[2]-normlower[1])/delta)
    elif(Cmax == normlower[1]):
       H_l = 120+60*((normlower[0]-normlower[2])/delta)
    else:
       H_l = 60*(((normlower[1]-normlower[0])/delta))
    if Cmax != 0:
        S_l = delta/Cmax
    else:
        S_l = 0
    V_l = Cmax
    

    normupper = upper
    Cmax = max(normupper)/360
    Cmin = min(normupper)/360
    delta = Cmax-Cmin
    if(Cmax == normupper[0]): #Blue max
       H_u = 240+60*((normupper[2]-normupper[1])/delta)
    elif(Cmax == normupper[1]):
       H_u = 120+60*((normupper[0]-normupper[2])/delta)
    else:
       H_u = 60*(((normupper[1]-normupper[0])/delta))
    if Cmax != 0:
        S_u = (delta)/Cmax
    else:
        S_u = 0
    V_u = Cmax


    hsv_lower= (H_l/2,S_l*254,V_l) #cv2.cvtColor(lower,cv2.COLOR_BGR2HSV)
    hsv_upper= (H_u/2,S_u*254,V_u) #cv2.cvtColor(upper,cv2.COLOR_BGR2HSV)
    # print(H_l,S_l,V_l)
    # print(hsv_upper,"\n",hsv_lower)
    lowerhsl =(50,100,75.75)#(H_l,S_l,V_l) #cv2.cvtColor(lower,cv2.COLOR_BGR2HSV)
    upperhsl =(64,196,130)#cv2.cvtColor(upper,cv2.COLOR_BGR2HSV)


    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lowerhsl,upperhsl)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    # print("KEYPOINTS", keypoints)
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints ,np.array([]))# (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        # print("No block found!")
        pass
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
