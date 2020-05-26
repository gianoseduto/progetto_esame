#! /usr/bin/env python

import rospy, cv2, cv_bridge, numpy
#cv_bridge converts between ROS Image messages and OpenCV images.

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import random 

class Follower:
	def __init__(self):
		self.bridge=cv_bridge.CvBridge()
		cv2.namedWindow('Window',cv2.WINDOW_NORMAL)
		cv2.resizeWindow('Window',30,30)
		#self.image_sub=rospy.Subscriber('/camera/rgb/image_raw',Image,self.image_callback)
		self.image_sub=rospy.Subscriber('/camera/image',Image,self.image_callback)

	def image_callback(self,msg):
		image=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		cv2.imshow('window',image)
		cv2.waitKey(10)

def controller():
	
	rospy.init_node('follower')
	follower=Follower()
	rospy.spin()
	
if __name__ =='__main__':
	try:
		controller()
	except rospy.ROSInterruptException: 
		pass
