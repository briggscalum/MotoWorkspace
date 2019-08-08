#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import cv2 as cv
import numpy as np
import argparse
import rospy
from math import atan2, cos, sin, sqrt, pi
from std_msgs.msg import Float32MultiArray
import time

def drawAxis(img, p_, q_, colour, scale):
    p = list(p_)
    q = list(q_)


    angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * cos(angle)
    q[1] = p[1] - scale * hypotenuse * sin(angle)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
    
def getOrientation(pts, img):
    
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i,0] = pts[i,0,0]
        data_pts[i,1] = pts[i,0,1]
    # Perform PCA analysis
    mean = np.empty((0))
    


    mean, eigenvectors = cv.PCACompute(data_pts, mean)
    
    # eigenvalues, eigenvectors = np.linalg.eig(eigenvectors)

    #eigenvalues = cv.eigen(eigenvectors)
    #np.linalg.eig

    # Store the center of the object
    cntr = (int(mean[0,0]), int(mean[0,1]))
    
    
    cv.circle(img, cntr, 3, (255, 0, 255), 2)
    p1 = (cntr[0] + 0.02 * eigenvectors[0,0] , cntr[1] + 0.02 * eigenvectors[0,1] )
    p2 = (cntr[0] - 0.02 * eigenvectors[1,0] , cntr[1] - 0.02 * eigenvectors[1,1])
    drawAxis(img, cntr, p1, (0, 255, 0), 1)
    drawAxis(img, cntr, p2, (255, 255, 0), 5)
    angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
    
    return angle, cntr

def getN(img,target, angle):

	imgN = img
	center = target

	if(target[1] < 200 or target[1] > 600 or target[0] < 200 or target[0] > 800):
	    return imgN, 0.0, 0.0 ,0.0

	center = target

	angle1 = 1.5 - angle
	angle2 = 2.95 - angle
	angle3 = 0.17 - angle

	point2 = [int(round(center[0]-190*cos(angle1))), int(round(center[1]+190*sin(angle1)))]
	point3 = [int(round(center[0]-290*cos(angle2))), int(round(center[1]+290*sin(angle2)))]
	point4 = [int(round(center[0]-300*cos(angle3))), int(round(center[1]+300*sin(angle3)))]
	img[point2[1],point2[0]-1] = 50;
	img[point3[1],point3[0]-1] = 50;
	img[point4[1],point4[0]] = 50;


	bottom_left_hor = 0;
	bottom_left_ver = 0;
	bottom_right_ver = 0;

	bottom_right_ver_2 = 0;
	bottom_left_ver_2 = 0;
	bottom_right_hor = 0;

	angle2 = 0;
	newy = 0;
	newx = 0;

	if (angle > 0):
		state = 0
		center = point2
		while center[0] > 2:
			if state == 0:
				if img[center[1],center[0]-1] == 0:
					state = 1
			elif state == 1:
				if img[center[1],center[0]-1] >= 240:
					state = 2
			elif state == 2:
				if img[center[1],center[0]-1] == 0:
					break
				img[center[1],center[0]-1] = 99;
				bottom_left_hor = bottom_left_hor + 1;
			center = [center[0]-1,center[1]]

		state = 0
		center = point2

		while center[1] < 947:
			if state == 0:
				if img[center[1]+1,center[0]] == 0:
					state = 1
			elif state == 1:
				if img[center[1]+1,center[0]] >= 240:
					state = 2
			elif state == 2:
				if img[center[1]+1,center[0]] == 0:
					break
				img[center[1]+1,center[0]] = 99;
				bottom_left_ver = bottom_left_ver + 1;

			center = [center[0],center[1]+1]

		state = 0
		center = point3

		while center[1] < 947:
			if state == 0:
				if img[center[1]+1,center[0]] == 0:
					state = 1
			elif state == 1:
				if img[center[1]+1,center[0]] >= 240:
					state = 2
			elif state == 2:
				if img[center[1]+1,center[0]] == 0:
					break
				img[center[1]+1,center[0]] = 99;
				bottom_right_ver = bottom_right_ver + 1;

			center = [center[0],center[1]+1]
		angle2 = (np.arcsin((bottom_right_ver-bottom_left_ver)*0.085/30))
		newy = ((bottom_left_ver + bottom_right_ver) / 2)
		newx = (bottom_left_hor - (newy)*0.4)

	else:
		state = 0
		center = point2
		while center[0] < 1200:
			if state == 0:
				if img[center[1],center[0]+1] == 0:
					state = 1
			elif state == 1:
				if img[center[1],center[0]+1] >= 240:
					state = 2
			elif state == 2:
				if img[center[1],center[0]+1] == 0:
					break
				img[center[1],center[0]+1] = 99;
				bottom_right_hor = bottom_right_hor + 1;
			center = [center[0]+1,center[1]]

		state = 0
		center = point2

		while center[1] < 947:
			if state == 0:
				if img[center[1]+1,center[0]] == 0:
					state = 1
			elif state == 1:
				if img[center[1]+1,center[0]] >= 240:
					state = 2
			elif state == 2:
				if img[center[1]+1,center[0]] == 0:
					break
				img[center[1]+1,center[0]] = 99;
				bottom_right_ver_2 = bottom_right_ver_2 + 1;
			center = [center[0],center[1]+1]

		state = 0
		center = point4

		while center[1] < 947:
			if state == 0:
				if img[center[1]+1,center[0]] == 0:
					state = 1
			elif state == 1:
				if img[center[1]+1,center[0]] >= 240:
					state = 2
			elif state == 2:
				if img[center[1]+1,center[0]] == 0:
					break
				img[center[1]+1,center[0]] = 99;
				bottom_left_ver_2 = bottom_left_ver_2 + 1;

			center = [center[0],center[1]+1]
		angle2 = (np.arcsin((bottom_left_ver_2-bottom_right_ver_2)*0.085/30))
		newy = ((bottom_left_ver_2 + bottom_right_ver_2) / 2)
		newx = (bottom_right_hor - (newy)*0.4)

	print("Angle, y, x")
	print(angle2)
	print(newy)
	print(newx)
	
	imgN = img
	return imgN,angle2, newx, newy	

parser = argparse.ArgumentParser(description='Code for Introduction to Principal Component Analysis (PCA) tutorial.\
                                              This program demonstrates how to use OpenCV PCA to extract the orientation of an object.')
parser.add_argument('--input', help='Path to input image.', default='../data/pca_test1.jpg')
args = parser.parse_args()

cap = cv.VideoCapture(0)

cap.set(cv.CAP_PROP_FRAME_WIDTH,4208)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,3120)

cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))

posepub = rospy.Publisher('fabric_pose', Float32MultiArray, queue_size=10)

rospy.init_node('fabric_detecter')

bwbuffer = [[],[],[],[],[]]

while(True):

	ret, src = cap.read()

	src = cv.resize(src,(1280,949))

	if cv.waitKey(1) & 0xFF == ord('q'):
	    break

	# Check if image is loaded successfully
	if src is None:
	    print('Could not open or find the image: ', args.input)
	    exit(0)

	# Convert image to grayscale
	gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)

	average = int(np.mean(gray))
	adjustment = 1 - (115 / average)
	#print(average)
	#print(adjustment)

	# Corrects for Brightness
	fixed = cv.add(gray,np.array(adjustment))


	# print(np.shape(gray))
	# print(np.shape(fixed))

	# Convert image to binary



	_ , obw = cv.threshold(fixed, 60, 255, cv.THRESH_BINARY) ## Will try and correct for lighting:  + cv.THRESH_OTSU

	bwbuffer[4] = bwbuffer[3]
	bwbuffer[3] = bwbuffer[2]
	bwbuffer[2] = bwbuffer[1]
	bwbuffer[1] = bwbuffer[0]
	bwbuffer[0] = obw

	bw = obw

	if(np.any(bwbuffer[4])):
	    bw = bwbuffer[0] + bwbuffer[1] + bwbuffer[2] + bwbuffer[3] + bwbuffer[4]

	_, contours , _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
	center = 0    

	for i, c in enumerate(contours):
	    # Calculate the area of each contour
	    area = cv.contourArea(c);
	    # Ignore contours that are too small or too large
	    
	    # Project
	    #if area > 60000 and area < 100000 :
	        #print(area)
	    if area < 70000 or 110000 < area:
	        continue

	    # Draw each contour only for visualisation purposes
	    
	    # Find the orientation of each shape
	    angle, center = getOrientation(c, src)    

	    if(center[1] < 200 or center[1] > 800 or center[0] < 200 or center[0] > 800):
	        continue

	    #print(center)
	    cv.drawContours(src, contours, i, (0, 0, 255), 2);

	    if center:
	        bw,angle2,newx,newy = getN(bw,center, angle)
	    
	    #bw[center[1],center[0]] = 100
	    position = Float32MultiArray()
	    if center != 0:
	        position.data = [center[0],center[1],angle,angle2,newx,newy]
	        posepub.publish(position)


	cv.imshow('Original',src)
	if(np.any(bw)):
	    cv.imshow('Black and White', bw)

cap.release()
cv.destroyAllWindows()


