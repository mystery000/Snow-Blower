#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = False
# Better handling for occlusions:
lr_check = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)

xout.setStreamName("disparity")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(lr_check)
depth.setExtendedDisparity(extended_disparity)
depth.setSubpixel(subpixel)

# Linking
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.disparity.link(xout.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queue will be used to get the disparity frames from the outputs defined above
    q = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)

    while True:
        inDisparity = q.get()  # blocking call, will wait until a new data has arrived
        frame = inDisparity.getFrame()
        # Normalization for better visualization
        frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)

        # cv2.imshow("disparity", frame)
        
        depth_thresh = 100.0 # Threshold for SAFE distance (in cm)
 
        # Mask to segment regions with depth less than threshold
        mask = cv2.inRange(frame, 10, depth_thresh)
        
        # Check if a significantly large obstacle is present and filter out smaller noisy regions
        if np.sum(mask) / 255.0 > 0.01 * mask.shape[0] * mask.shape[1]:
        
            # Contour detection 
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnts = sorted(contours, key=cv2.contourArea, reverse=True)
            
            # Check if detected contour is significantly large (to avoid multiple tiny regions)
            if cv2.contourArea(cnts[0]) > 0.01 * mask.shape[0] * mask.shape[1]:
            
                x,y,w,h = cv2.boundingRect(cnts[0])
            
                # finding average depth of region represented by the largest contour 
                mask2 = np.zeros_like(mask)
                cv2.drawContours(mask2, cnts, 0, (255), -1)
            
                # Calculating the average depth of the object closer than the safe distance
                depth_mean, _ = cv2.meanStdDev(frame, mask=mask2)
                
                # Display warning text
                cv2.putText(frame, "WARNING !", (x + 15, y), 1, 2, (0, 0, 255), 2, 2)
                cv2.putText(frame, "Object at", (x + 15, y + 40), 1, 2, (100, 10, 25), 2, 2)
                cv2.putText(frame, "%.2f cm"%depth_mean, (x + 15, y + 80), 1, 2, (100, 10, 25), 2, 2)
        
        else:
            cv2.putText(frame, "SAFE!", (100, 100), 1, 3, (0, 255, 0), 2, 3)
        
        cv2.imshow('output_canvas', frame)


        # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        cv2.imshow("disparity_color", frame)

        if cv2.waitKey(1) == ord('q'):
            break