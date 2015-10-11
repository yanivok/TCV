#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from cuprec.srv import *
from cuprec.msg import *

br = CvBridge()

def handle_image(req):
	global br
	# convert input img to cv2
	img = br.imgmsg_to_cv2(req.inimage)	

	# init classifier 
	cass = cv2.CascadeClassifier("/home/komodo/catkin_ws/src/cuprec/cascade_27.xml")

	# convert img to gray
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	# Detect cups
	cups = cass.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=10, minSize=(10, 10), flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

	count = 0
	l = []
	
	#Make a list of the rectangles coordinates around the detected cups
	for (x, y, w, h) in cups:
		coordinates = Coordinates();
		count += 1
		coordinates.arr = [x, y, w, h]
		l.append(coordinates);

	return FindCupsResponse(count,l)

def detector():
	
	rospy.init_node('detector')
	s = rospy.Service('cup_detect' ,FindCups, handle_image)
	print "Cup detector running"
	rospy.spin()
if __name__ == "__main__":

	detector()
	


