#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
#cv_bridge converts between ROS Image messages and OpenCV images.

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import random 

PI = 3.1415926535897

def scan_callback(msg):
	global g_range_ahead	
	global g_range_ahead_right
	global g_range_ahead_left
	global g_range_right
	global g_range_left
	global g_range_behind
	g_range_left=min(msg.ranges[45:134]) 
	g_range_ahead=min(min(msg.ranges[0:15]),min(msg.ranges[345:359]))
	g_range_right=min(msg.ranges[225:315])
	g_range_behind=min(msg.ranges[135:224])
	g_range_ahead_right=min(msg.ranges[315:344])
	g_range_ahead_left=min(msg.ranges[16:44])	

g_range_ahead=1
g_range_right=1
g_range_left=1
g_range_behind=1
g_range_ahead_right=1
g_range_ahead_left=1
g_color = str("none")
temp_color = str("none")

class Follower:
	
	def __init__(self):
		#print('init Follower')
		#controlla se stiamo leggendo l'immagine del colore o no
		#cosi da poter virare per non andare a sbattere al muro solo se non sto girando
		#per cambiare direzione a seconda del colore
		self.arrived = False
		self.temp_color = str("none")
		self.g_color = str("none")	
		self.img = False
		self.direction = 1 #random.randint(0,1)
		print("direction: ", self.direction)
		self.bridge=cv_bridge.CvBridge()
		#cv2.namedWindow('Window',cv2.WINDOW_NORMAL)
		self.image_sub=rospy.Subscriber('/camera/rgb/image_raw',Image,self.image_callback)
		self.scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
		
		#self.image_sub=rospy.Subscriber('/camera/image',Image,self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		#self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.twist_callback)
		self.twist=Twist()
		
			
	
	def twist_fun(self):

		if not self.img:
			#print('no self image', self.temp_color)
			if (g_range_ahead <= 0.5 or g_range_ahead_right <= 0.3 or g_range_ahead_left <=0.3):
				self.twist.linear.x = 0
				if not(self.temp_color == "none"):
					#print('not none')
					if(self.temp_color == "green"):
						#print('no image green')
						self.twist.angular.z =  + 0.3
					elif(self.temp_color == "blue"):
						#print('no image blue')
						self.twist.angular.z =  - 0.3
					elif(self.temp_color == "purple"):
						if self.direction==1:
							self.twist.angular.z = 0.0
						else:	
							self.twist.angular.z = - 0.3
				
					
				elif(abs(g_range_ahead_right - g_range_ahead_left)>0.1):
					if  g_range_ahead_right < g_range_ahead_left:
						#print('stop-left')
						self.twist.angular.z =  + 0.3
						
					else:
						#print('stop-right')
						self.twist.angular.z = - 0.3
				
				#corridoio
				elif (abs(g_range_left - g_range_right )> 0.3 and (g_range_ahead <= 0.5) ):
					if (g_range_ahead_left> g_range_ahead_right):
						self.twist.angular.z =  + 45*PI/180
					else:
						self.twist.angular.z =  - 45*PI/180
				else:
					
					if  g_range_right < g_range_left:
						#print('stop-left')
						self.twist.angular.z =  + 0.3
						
					else:
						#print('stop-right')
						self.twist.angular.z = - 0.3
		
			else:	
				if (g_range_ahead > 0.6 and g_range_ahead_right > 0.4 and g_range_ahead_left > 0.4):
					self.temp_color = "none"
				
				self.twist.linear.x = 0.3
				self.twist.angular.z = 0
		else:	
			self.twist.linear.x = 0
			self.temp_color = self.g_color
			
			if(self.temp_color == "green"):
				#print('no image green')
				self.twist.angular.z =  + 0.3
			elif(self.temp_color == "blue"):
				#print('no image blue')
				self.twist.angular.z =  - 0.3
			elif(self.temp_color == "purple"):
				if self.direction==1:
					self.twist.angular.z = 0.0
				else:	
					self.twist.angular.z = - 45*PI/180
			elif(self.temp_color == "yellow"):
					while(True):
						self.twist.angular.z =  - 0.3
						self.twist.linear.x = 0
					
			
		self.cmd_vel_pub.publish(self.twist)
		rospy.Rate(10).sleep()
			

	def image_callback(self,msg):
		image=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		#Blue
		low_blue= np.array([100,150,0])
		high_blue = np.array([140,255,255])

		blue_mask = cv2.inRange(hsv, low_blue, high_blue)
		blue = cv2.bitwise_and(image, image, mask= blue_mask)
		
		#Green					
		low_green = np.array([25, 52, 72])
		high_green = np.array([102, 255, 255])
		green_mask = cv2.inRange(hsv, low_green, high_green)
		green = cv2.bitwise_and(image, image, mask=green_mask)
		
		#White. Entrance of maze
		sensitivity = 15
		low_white = np.array([0,0,255-sensitivity], dtype=np.uint8)
		high_white = np.array([255,sensitivity,255], dtype=np.uint8)
		white_mask = cv2.inRange(hsv, low_white, high_white)
		white = cv2.bitwise_and(image, image, mask=white_mask)
		
		#Yellow. Exit of maze
		low_yellow=np.array([20,100,100])
		high_yellow=np.array([30,255,255])
		yellow_mask=cv2.inRange(hsv,low_yellow,high_yellow)
		yellow=cv2.bitwise_and(image,image,mask=yellow_mask)

		#Purple
		low_purple = np.array([145,100,20], dtype=np.uint8)
		high_purple = np.array([160,255,255], dtype=np.uint8)
		purple_mask = cv2.inRange(hsv, low_purple, high_purple)
		purple = cv2.bitwise_and(image, image, mask=purple_mask)

		#cv2.imshow('window', image)
		#cv2.imshow('blue', blue)
		#cv2.imshow('mask_b', blue_mask)
		#cv2.imshow('white', white)
		#cv2.imshow('mask_w', white_mask)
		cv2.waitKey(10)
		
		if (np.count_nonzero(white)>210000):
			#print('white')
			self.img=True

		else:
			if (np.count_nonzero(blue)>500000):
				self.img=True
				self.g_color="blue"
				#print(g_color)
				
			elif (np.count_nonzero(green)>500000):
				self.g_color="green"
				#print(g_color)
				self.img=True
			
			elif (np.count_nonzero(yellow)>350000):
				self.img=True
				self.g_color="yellow"
				print('arrived')
				self.arrived=True
			elif (np.count_nonzero(purple)>500000):
				self.img=True
				self.g_color="purple"	
			else: 
				
				self.img=False
		
		if not self.arrived:
			self.twist_fun()
		else:
			self.twist.angular.z =  - 0.3
			self.twist.linear.x = 0
			self.cmd_vel_pub.publish(self.twist)
			rospy.Rate(10).sleep()
	

def controller():
	#print('controller')	
	rospy.init_node('follower')
	#print('init node')	
	follower=Follower()
	#print('rospy')
	rospy.spin()
	
	
if __name__ =='__main__':
	try:	
		#print('try')
		controller()
	except rospy.ROSInterruptException: 
		pass
