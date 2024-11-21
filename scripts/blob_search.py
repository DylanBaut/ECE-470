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
    Ty =.09
    Tx =.300

    Or= 239
    Oc = 319
    xc = (row-Or)/Beta 
    yc=  (col-Oc)/Beta
    # print("XC,YC: ",xc,yc, "\n \n \n")
    xw = (xc+Tx)*np.cos(theta)-(yc+Ty)*np.sin(theta)
    yw = (xc+Tx)*np.sin(theta)+(yc+Ty)*np.cos(theta)
    # print("World Coordinates:",xw,",",yw)
    return(xw,yw)




# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 500
    params.maxArea = 3000
    # Filter by Circularity
    params.filterByCircularity = False

    
    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.9

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    # ========================= Student's code starts here =========================
    

    # print(H_l,S_l,V_l)
    # print(hsv_upper,"\n",hsv_lower)
    # lowerhsl =(50,100,75.75)#(H_l,S_l,V_l) #cv2.cvtColor(lower,cv2.COLOR_BGR2HSV)
    # upperhsl =(64,196,130)#cv2.cvtColor(upper,cv2.COLOR_BGR2HSV)


    if(color == 'green'):
        lowerhsl =(40,70,70)
        upperhsl =(60,255,255)
    elif(color == 'yellow'):
        lowerhsl =(20,102,102)
        upperhsl =(30,255,255)
    

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
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints ,np.array([]),flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)# (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

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
