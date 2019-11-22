#!/usr/bin/env python

import cv2
import numpy as np
import sys

import argparse
from math import atan2, cos, sin, sqrt, pi
import time
import argparse
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError




def drawAxis(img, p_, q_, colour, scale):
    p = list(p_)
    q = list(q_)


    angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * cos(angle)
    q[1] = p[1] - scale * hypotenuse * sin(angle)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)

def getCenter(pts, img):
    
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i,0] = pts[i,0,0]
        data_pts[i,1] = pts[i,0,1]
    # Perform PCA analysis
    mean = np.empty((0))
    mean, eigenvectors = cv2.PCACompute(data_pts, mean)
    # eigenvalues, eigenvectors = np.linalg.eig(eigenvectors)
    #eigenvalues = cv.eigen(eigenvectors)
    #np.linalg.eig
    cntr = (int(mean[0,0]), int(mean[0,1]))

    cv2.circle(img, cntr, 3, (255, 0, 255), 2)
    p1 = (cntr[0] + 0.02 * eigenvectors[0,0] , cntr[1] + 0.02 * eigenvectors[0,1] )
    p2 = (cntr[0] - 0.02 * eigenvectors[1,0] , cntr[1] - 0.02 * eigenvectors[1,1])
    drawAxis(img, cntr, p1, (0, 255, 0), 1)
    drawAxis(img, cntr, p2, (255, 255, 0), 5)
    angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians


    # Store the center of the object
    
        
    return cntr, angle

posepub = rospy.Publisher('fabric_pose', Float32MultiArray, queue_size=10)
picpub = rospy.Publisher("Camera_bw",Image,queue_size=10)
colorpub = rospy.Publisher("Camera_color",Image,queue_size=10)
bridge = CvBridge()

rospy.init_node('fabric_detecter')

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,4208)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,3120)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

print('Width: ' + str(cap.get(3)));
print('Height: ' + str(cap.get(4)));
ret, frame = cap.read()





if(cap.isOpened()):
	while(True):
		Width = 0
		ret, frame = cap.read()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		notgray = 255 - gray
		##pbw = cv2.GaussianBlur(notgray,(3,3),cv2.BORDER_DEFAULT)
		##bw = cv2.adaptiveThreshold(notgray,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
  		#cv2.THRESH_BINARY,15,5)
  		notgray = cv2.GaussianBlur(notgray,(5,5),0)
		_ , bw = cv2.threshold(notgray, 185, 255, cv2.THRESH_BINARY) ## Will try and correct for lighting:  + cv.THRESH_OTSU
		_ , bw2 = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY) ## Will try and correct for lighting:  + cv.THRESH_OTSU

		_, contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
		_, contours2, _ = cv2.findContours(bw2, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
		center = 0  
		
		for i, c in enumerate(contours):
			# Calculate the area of each contour
			area = cv2.contourArea(c);
			# Ignore contours that are too small or too large

			# Project
			if area < 40000:
			 	continue
			#print(area)
			# Draw each contour only for visualisation purposes

			# Find the orientation of each shape
			center, angle = getCenter(c, frame)    

			if(center[1] < 400 or center[1] > 800 or center[0] < 400 or center[0] > 800):
				continue
			# print(center)
			cv2.drawContours(frame, contours, i, (0, 0, 255), 2);

			max_x = 0
			max_y = 0
			min_x = 10000
			min_y = 10000

			
			for k in c:
				j = k[0]
				if j[0] > max_x:
					max_x = j[0]
				if j[0] < min_x:
					min_x = j[0]
				if j[1] > max_y:
					max_y = j[1]
				if j[1] < min_y:
					min_y = j[1]

			print "Height: ", max_y - min_y
			print "Width: ", max_x - min_x
			Width = max_y - min_y
			position = Float32MultiArray()
			if center != 0:
				position.data = [0,0,Width,0,0,0]
				posepub.publish(position)
			# if center:
			    # bw,angle2,newx,newy = getN(bw,center, angle)

			#bw[center[1],center[0]] = 100
			# position = Float32MultiArray()
			# if center != 0:
			#     position.data = [center[0],center[1],angle,angle2,newx,newy]
			#     posepub.publish(position)
		for i, c in enumerate(contours2):
			# Calculate the area of each contour
			area = cv2.contourArea(c);
			# Ignore contours that are too small or too large

			# Project
			if area < 5000 or area > 20000:
			 	continue
			# Draw each contour only for visualisation purposes

			# Find the orientation of each shape
			center, angle = getCenter(c, frame)    

			if(center[1] < 400 or center[1] > 800 or center[0] < 400 or center[0] > 800):
				continue
			# print(center)
			cv2.drawContours(frame, contours2, i, (0, 0, 255), 2);

			max_x = 0
			max_y = 0
			min_x = 10000
			min_y = 10000

			
			for k in c:
				j = k[0]
				if j[0] > max_x:
					max_x = j[0]
				if j[0] < min_x:
					min_x = j[0]
				if j[1] > max_y:
					max_y = j[1]
				if j[1] < min_y:
					min_y = j[1]

			print "Little Height: ", max_y - min_y
			print "Little Width: ", max_x - min_x

			# if center:
			    # bw,angle2,newx,newy = getN(bw,center, angle)

			#bw[center[1],center[0]] = 100
			position = Float32MultiArray()
			if center != 0:
				position.data = [center[0],center[1],Width,angle,0,0]
				posepub.publish(position)

		cv2.imshow('contour',frame)
		cv2.imshow('blackwhite',bw2)
		cv2.imshow('invert',bw)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()