#!/usr/bin/env python
import rospy
import sys
import cv2
import math
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from facerec.srv import *
from cuprec.msg import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 
from numpy import arctan, pi, sign
from threading import Lock

MIN_DISTANCE = 0.35		# Stop distance from obstacle
MAX_SPEED = 0.25		# Maximum forward velocity
TURN_VEL = 0.7			# Turn velocity
FORWARD_ANGLE = 0.3		# Forward zone 
FORWARD_DRIVE_RATE = 1	# Rate of forawrd drive messages
TIME_STOP = 7.0			# Minimum time between move
MIN_REACH = 500.0		# Minimum distance to object to initiate grabbing
MAX_REACH = 950.0		# Maximum distance to object to initiate grabbing
SCAN_TIME = 10.0		# Time to start scanning if a cup isn't detected
SCAN_ANGLE = 0.3		# Turn angle during scan

def func(x):
	return x+0.05

#Turning using odometry feedback
def turnByAngle(angle,turnVelocity,errorWidth):
	startAngle = currentAngle
	drive_msg = Twist()
	drive_msg.linear.x = 0.0
	print("Turning " + str(angle) + " rad")
	while not rospy.is_shutdown() and not (scanningMode and lastTime !=0.0) and abs(currentAngle-startAngle-angle) > errorWidth:
		drive_msg.angular.z = -1*turnVelocity*sign(currentAngle-startAngle-angle)
		drive_pub.publish(drive_msg)
		rospy.sleep(0.1)
	drive_msg.angular.z = 0.0
	drive_pub.publish(drive_msg)
	if not (scanningMode and lastTime !=0.0):
		print("Yap")

def move_callback(data):
	global drive_pub, grab_pub, lock, canMove, speed, lastTime, obstacleCount, grabMode, driveForward, MIN_DISTANCE, criticalDistance, scanningMode
	lock.acquire()
	thisTime = time.time()
	timePassed = thisTime-lastTime
	print timePassed
	drive_msg = Twist()
	print scanningMode
	# Stop if cup is within reach
	if(canMove and data.z>MIN_REACH and data.z<MAX_REACH):
		canMove = 0
		driveForward = 0
		print("Cup in reach.")

	# We wait TIME_STOP seconds before responding to new position data because of the delay in input data
	if(timePassed>TIME_STOP and grabMode==0 and scanningMode==0):
		deltaX = data.x
		driveForward = 0
		
		if(canMove):
			# Move forward if the cup is centered
			if(abs(deltaX)<FORWARD_ANGLE):
				print("Forward moving mode")
				driveForward = 1

			# Turn towards the cup if it's not centered
			else:
				print("Turning mode")
				turnByAngle(deltaX*0.15,TURN_VEL,0.04)
			obstacleCount = 0
			
		# !canMove
		else:
			if speed == 0.0:
				print("Too close to obstacle")
			obstacleCount = obstacleCount + 1
			print("z distance from target: " +str(data.z))
			
			# if the cup is within reach and we stopped moving, initiate the cup grabbing
			if(obstacleCount>2 and data.z>MIN_REACH and data.z<MAX_REACH):
				print("Grabbing mode")
				grabMode = 1
				grab_pub.publish(data)

			# if the cup is almost within reach, move a little bit further.
			elif(obstacleCount>3 and data.z>MIN_REACH and data.z<(MAX_REACH+MIN_DISTANCE*1000)):
				if MIN_DISTANCE>criticalDistance:
					MIN_DISTANCE = max(MIN_DISTANCE/2.0,criticalDistance)
					print("Almost in reach. Moving a bit forward.")
				else:
					print("Almost in reach, but can't move any further.")

		lastTime=thisTime
	if scanningMode:
		lastTime=thisTime
	lock.release()
		
def scan_callback(data):
	global lock, canMove, speed, MIN_DISTANCE
	lock.acquire()
	closestObstacle = min(data.ranges)
	if(closestObstacle<MIN_DISTANCE):
		speed = 0.0
		canMove = 0
	else:
		speed = MAX_SPEED*func(closestObstacle-MIN_DISTANCE)
		speed = min(speed,MAX_SPEED)
		if(math.isnan(speed)):
			speed=0.0
		canMove = 1
	lock.release()

def odom_callback(data):
	global currentAngle
	currentAngle = data.pose.pose.orientation.z
	
def drive():
	global drive_pub, grab_pub, lock, canMove, speed, lastTime, obstacleCount, grabMode, driveForward, criticalDistance, currentAngle, scanningMode
	lock = Lock()
	print "Drive node running"
	criticalDistance = 0.1
	speed = 0.0
	initTime = time.time()
	lastTime = 0.0
	driveForward = 0
	forwardCount = 0
	canMove = 0
	grabMode = 0
	obstacleCount = 0
	scanStage = 0
	scanningMode = 0
	startAngle = 0.0
	drive_msg = Twist()
	drive_msg_zero = Twist()
	drive_msg_zero.linear.x = 0.0
	drive_msg_zero.angular.z = 0.0
	rospy.init_node('drive')

	rospy.Subscriber("cup_location", Position, move_callback)
	rospy.Subscriber("/komodo_1/odom_pub", Odometry, odom_callback)
	rospy.Subscriber("/komodo_1/scan", LaserScan, scan_callback)
	drive_pub = rospy.Publisher("/komodo_1/cmd_vel" ,Twist, queue_size=1)
	grab_pub = rospy.Publisher("grab_cup", Position, queue_size=1)
	
	rate = rospy.Rate(FORWARD_DRIVE_RATE)
	

	#Scanning from side to side and moving forward until a cup is detected
	while not rospy.is_shutdown() and lastTime == 0.0:
		thisTime = time.time()
		timePassed = thisTime-initTime
		if(timePassed> SCAN_TIME):
			scanningMode=1
			print("Didn't detect cup. Scanning.")
			if canMove:
				if scanStage == 0:
					print("Checking left.")
					startAngle = currentAngle
					targetAngle = SCAN_ANGLE
					turnByAngle(targetAngle,TURN_VEL,0.07)
					scanStage = 1
				elif scanStage == 1:
					print("Back to center.")
					targetAngle = startAngle-currentAngle
					turnByAngle(targetAngle,TURN_VEL,0.07)
					scanStage = 2
				elif scanStage == 2:
					print("Checking right.")
					targetAngle = -1*SCAN_ANGLE
					turnByAngle(targetAngle,TURN_VEL,0.07)
					scanStage = 3
				elif scanStage == 3:
					print("Back to center.")
					targetAngle = startAngle-currentAngle
					turnByAngle(targetAngle*0.65,TURN_VEL,0.07)
					scanStage = 4
				elif scanStage == 4:
					print("Driving forward.")
					for i in range(5):
						drive_msg.linear.x = speed
						drive_msg.angular.z = 0.0
						drive_pub.publish(drive_msg)
						rospy.sleep(1.8)
					scanStage = 0
					rospy.sleep(5.0)
				rospy.sleep(1.0)
			else:
				print("Too close to obstacle. Can't move for scanning.")
			

	scanningMode = 0

	while not rospy.is_shutdown():
		if(driveForward and canMove and forwardCount<5):
			forwardCount = forwardCount + 1
			drive_msg.linear.x = speed
			drive_msg.angular.z = 0.0
			drive_pub.publish(drive_msg)
			print("Moving forward with speed: "+str(speed))
		else:
			forwardCount = 0
			drive_pub.publish(drive_msg_zero)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	try:
		drive()
	except rospy.ROSInterruptException:
		pass
