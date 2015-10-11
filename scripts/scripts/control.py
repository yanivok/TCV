#!/usr/bin/env python
import rospy
import sys
import cv2
import math
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cuprec.srv import *
from cuprec.msg import *
from geometry_msgs.msg import Twist
from threading import Lock

br = CvBridge()
num = 0;
depth = None
deltaX = 0.0
CENTER_OF_IMAGE_X = 320.0
CENTER_OF_IMAGE_Y = 240.0
UPDATE_FRAME = 15
SAVE_IMAGES = 1

def rgb_callback (data):
	global location_pub, num, depth, lock, deltaX, CENTER_OF_IMAGE_X, frameCount
	lock.acquire()

	#Preform the cup detection only every UPDATE_FRAME frames
	frameCount = frameCount +1
	if(frameCount>UPDATE_FRAME):
		frameCount = 0

		#Wait until the 'cup_detect' service is running
		rospy.wait_for_service('cup_detect')
		try:

			#Call the service on the image
			detect = rospy.ServiceProxy('cup_detect', FindCups)
			resp = detect(data)

			#If cups are detected calculate their position and send it to the drive node
			if resp.numcups > 0:
				num=num+1
				c = resp.cups[0]
				x = c.arr[0]
				y = c.arr[1]
				w = c.arr[2]
				h = c.arr[3]
				centerX = int(x + w/2)
				centerY = int(y + h/2)
				deltaX = CENTER_OF_IMAGE_X-centerX
				deltaY = CENTER_OF_IMAGE_Y-centerY
				print(str(resp.numcups) + " cups detected. Cup "+ str(num)+".")
				position = Position()
				position.x = deltaX/CENTER_OF_IMAGE_X
				position.y = deltaY/CENTER_OF_IMAGE_Y
				position.z = 0.0

				# We set z to be the average depth within the center of the detected area
				if(depth!=None):
					avg_z = 0.0
					count = 0
					for i in range(-4,5):
						for j in range(-7,8):
							if depth[centerY+j][centerX+i] != 0.0:
								avg_z += depth[centerY+j][centerX+i]
								count += 1
					if count>0:
						position.z = avg_z/count
				print(str(position.x) + " " + str(position.y) + " " + str(position.z))
				location_pub.publish(position)
				
				#If SAVE_IMAGES is set to true, we save the RGB and depth images to disk.
				if(SAVE_IMAGES and depth!=None):
					img = br.imgmsg_to_cv2(data)
					npdepth = np.array(depth)
					npdepth = npdepth / 13
					# color the rectangle
					cv2.rectangle(npdepth, (x, y), (x+w, y+h), (255, 255, 255), 2)
					cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 255), 2)
					# filenames
					depth_filename = "depth.jpg"
					rgb_filename = "rgb.jpg"
					# save to disk
					cv2.imwrite(depth_filename,npdepth)
					cv2.imwrite(rgb_filename,img)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		finally:
			lock.release()
	else: 
		lock.release()

def depth_callback (data):
	global depth, lock
	lock.acquire()
	depth = br.imgmsg_to_cv2(data)
	lock.release()
	
def control():
	global location_pub, depth, lock, frameCount
	lock = Lock()
	print "control"
	rospy.Subscriber("/komodo_1/komodo_1_Asus_Camera/rgb/image_raw", Image, rgb_callback)
	rospy.Subscriber("/komodo_1/komodo_1_Asus_Camera/depth/image_raw", Image, depth_callback)
	location_pub = rospy.Publisher("cup_location", Position, queue_size=1)

	frameCount = 0

	rospy.init_node('control', anonymous=True)
	rospy.spin()

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass
