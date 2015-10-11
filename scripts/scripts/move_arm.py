#!/usr/bin/env python

import rospy
import roslib
import numpy as np
import math
from cuprec.msg import *
from std_msgs.msg import Float64

X_FOV = 1.4			# Horizontal field of view of the camera
Y_FOV = 1.05		# Vertical field of view of the camera
SH = 0.5 			# Shoulder to elbow length in meters
EL = 0.33 			# Elbow to wrist length in meters
H_ARM_TO_CAM = 0.5 	# Height of camera from the arm base


def move_arm_initialize():
	global pub_base_rotation, pub_shoulder, pub_elbow1, pub_elbow2, pub_wrist, pub_left_finger, pub_right_finger
	
	print("Moving arm to initial state")	

	br = Float64(0.0)
	sh = Float64(0.0)
	e1 = Float64(-1.57)
	e2 = Float64(-1.57)	
	wr = Float64(-1.57) 
	lf = Float64(0.0) 	
	rf = Float64(0.0)

	pub_base_rotation.publish(br)
	pub_shoulder.publish(sh)
	pub_elbow1.publish(e1)
	pub_elbow2.publish(e2)
	pub_wrist.publish(wr)
	pub_left_finger.publish(lf)
	pub_right_finger.publish(rf)
	rospy.sleep(5.0)

	lf = Float64(-0.8) 	
	rf = Float64(0.8)
	pub_left_finger.publish(lf)
	pub_right_finger.publish(rf)

def move_arm_zero():
	global pub_base_rotation, pub_shoulder, pub_elbow1, pub_elbow2, pub_wrist, pub_left_finger, pub_right_finger
	
	print("Moving arm to grabbing state")	
	
	br = Float64(0.0)
	sh = Float64(0.0)
	e1 = Float64(-1.0)
	e2 = Float64(0.0)	
	wr = Float64(0.0) 
	lf = Float64(0.0) 	
	rf = Float64(0.0)

	pub_base_rotation.publish(br)
	pub_shoulder.publish(sh)
	pub_elbow1.publish(e1)
	pub_elbow2.publish(e2)
	pub_wrist.publish(wr)
	pub_left_finger.publish(lf)
	pub_right_finger.publish(rf)

	rospy.sleep(1.0)

	br = Float64(0.0)
	sh = Float64(0.0)
	e1 = Float64(0.0)
	e2 = Float64(1.57)
	wr = Float64(0.0)
	lf = Float64(-0.8)
	rf = Float64(0.8)

	pub_base_rotation.publish(br)
	pub_shoulder.publish(sh)
	pub_elbow1.publish(e1)
	pub_elbow2.publish(e2)
	pub_wrist.publish(wr)
	pub_left_finger.publish(lf)
	pub_right_finger.publish(rf)

def pick_and_throw():
	global pub_base_rotation, pub_shoulder, pub_elbow1, pub_elbow2, pub_wrist, pub_left_finger, pub_right_finger
	
	print("Picking cup")	

	lf = Float64(0.2)
	rf = Float64(0.0)

	pub_left_finger.publish(lf)
	pub_right_finger.publish(rf)
	rospy.sleep(4.0)

	br = Float64(0.0)
	sh = Float64(0.0)
	e1 = Float64(0.0)
	e2 = Float64(0.0)
	wr = Float64(0.0)
	lf = Float64(0.2)
	rf = Float64(0.0)

	pub_base_rotation.publish(br)
	pub_shoulder.publish(sh)
	pub_elbow1.publish(e1)
	pub_elbow2.publish(e2)
	pub_wrist.publish(wr)
	pub_left_finger.publish(lf)
	pub_right_finger.publish(rf)
	

def grab_callback(data):
	global pub_base_rotation, pub_shoulder, pub_elbow1, pub_elbow2, pub_wrist, pub_left_finger, pub_right_finger
	
	x = data.x
	y = data.y - 0.40
	z = data.z/1000.0 -0.04 + (950 - data.z)*0.0001 #- 0.05*x
	
	
	move_arm_zero()
	rospy.sleep(4.0)

	# Calculating desired joint angles in order to reach the cup

	x_a = x*X_FOV/2.0
	y_a = y*Y_FOV/2.0
	
	dx = np.cos(y_a) * z
	dy = np.sin(y_a) * z
	H = H_ARM_TO_CAM + dy
	
	L_squared = np.square(dx) + np.square(H)
	L = np.sqrt(L_squared)
	
	A = (L_squared + np.square(SH) - np.square(EL))/ (2*SH*L)
	B = (np.square(EL) + np.square(SH) - L_squared)/ (2*SH*EL)
	
	alpha = np.arccos(A)
	beta = np.arccos(B)
	gamma = np.arctan2(H,dx)

	br = x_a + 0.12
	sh = 0.5*np.pi - (alpha+gamma)
	e2 = np.pi - beta
	
	print("br: " +str(br) + " sh: " +str(sh) + " e2: " +str(e2))
	
	if(math.isnan(sh) or math.isnan(e2)):
		print("Target is out of reach")
	else:
		pub_base_rotation.publish(br)
		pub_shoulder.publish(sh)
		pub_elbow2.publish(e2)
	
	rospy.sleep(5.0)
	pick_and_throw()
	rospy.sleep(1.0)
	move_arm_initialize()
	
def move_arm():
	global pub_base_rotation, pub_shoulder, pub_elbow1, pub_elbow2, pub_wrist, pub_left_finger, pub_right_finger
	
	pub_base_rotation=rospy.Publisher('/komodo_1/base_rotation_controller/command', Float64, queue_size=10)
	pub_shoulder = rospy.Publisher('/komodo_1/shoulder_controller/command', Float64, queue_size=10)
	pub_elbow1 = rospy.Publisher('/komodo_1/elbow1_controller/command', Float64, queue_size=10)	
	pub_elbow2 = rospy.Publisher('/komodo_1/elbow2_controller/command', Float64, queue_size=10)	
	pub_wrist = rospy.Publisher('/komodo_1/wrist_controller/command', Float64, queue_size=10)
	pub_left_finger = rospy.Publisher('/komodo_1/left_finger_controller/command', Float64, queue_size=10)
	pub_right_finger = rospy.Publisher('/komodo_1/right_finger_controller/command', Float64, queue_size=10)
	rospy.Subscriber("grab_cup", Position, grab_callback)

	rospy.init_node('move_arm')

	move_arm_initialize()

	
	rospy.spin()
	
 
if __name__ == '__main__':
	try:
		move_arm()
	except rospy.ROSInterruptException:
		pass
