#! /usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
#cv_bridge converts between ROS Image messages and OpenCV images.

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import random 

class Follower:
	def __init__(self):
		print('init Follower')
		self.bridge=cv_bridge.CvBridge()
		cv2.namedWindow('Window',cv2.WINDOW_NORMAL)
		cv2.resizeWindow('Window',30,30)
		self.image_sub=rospy.Subscriber('/camera/rgb/image_raw',Image,self.image_callback)
		#self.image_sub=rospy.Subscriber('/camera/image',Image,self.image_callback)
		self.cmd_vel_pub=rospy.Publisher('cmd_vel', Twist, queue_size=1)
		#self.twist=Twist()

	def image_callback(self,msg):
		print('Callback')
		image=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		#Red
		low_red = np.array([161, 155, 84])
		high_red = np.array([179, 255, 255])

		red_mask = cv2.inRange(hsv, low_red, high_red)
		red = cv2.bitwise_and(image, image, mask= red_mask)
		
		cv2.imshow('window', image)
		cv2.imshow('red', red)
		cv2.imshow('mask', red_mask)
		cv2.waitKey(10)
		
		rospy.loginfo('ciao2')
		#printer()

		if np.any(red) == True:
			self.twist.linear.x=0.2
			self.cmd_vel_pub.publish(self.twist)
		
		
		#Green
'''
		low_green = np.array([25, 52, 72])
    high_green = np.array([102, 255, 255])
    green_mask = cv2.inRange(hsv, low_green, high_green)
    green = cv2.bitwise_and(image, image, mask=green_mask)
'''

def printer():
	rospy.init_node('printer_node')
	print('a')
	

def controller():
	print('controller')	
	rospy.init_node('follower')
	print('init node')	
	follower=Follower()
	print('rospy')
	rospy.spin()
	
	
if __name__ =='__main__':
	try:	
		print('try')
		controller()
	except rospy.ROSInterruptException: 
		pass
