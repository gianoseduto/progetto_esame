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

class Follower:
	
	def __init__(self):
		print('init Follower')
		#controlla se stiamo leggendo l'immagine del colore o no
		#cosi da poter virare per non andare a sbattere al muro solo se non sto girando
		#per cambiare direzione a seconda del colore
			
		self.img = False
		self.bridge=cv_bridge.CvBridge()
		cv2.namedWindow('Window',cv2.WINDOW_NORMAL)
		self.image_sub=rospy.Subscriber('/camera/rgb/image_raw',Image,self.image_callback)
		self.scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
		
		#self.image_sub=rospy.Subscriber('/camera/image',Image,self.image_callback)
		self.cmd_vel_pub=rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.twist_callback)
		self.twist=Twist()
			
	#in teoria legge la velocita corrente dal canale sub e aggiunge 0.1 
	#in direzione negativa o positiva per correggere il tiro
	#invece sovrascrive la velocita angolare con -0.1
	def twist_callback(self,vel):
		#print('angular z= ',vel.angular.z)
		if (g_range_ahead < 0.1):
			while(g_range_ahead<0.5):
				self.twist.linear.x = -0.1
		else:
			if not self.img:
				if (g_range_ahead <= 0.5 ):
					if (g_range_ahead <= 0.4):
						if (g_range_right>0.20 and g_range_left < 0.20):
							self.twist.linear.x = -0.2
							self.twist.angular.z = vel.angular.z + random.randint(-0.3, 0.3)
							print('random angle')
						elif  g_range_right < g_range_left:
							print('stop-left', g_range_right)
							self.twist.angular.z = vel.angular.z + 0.3
							self.twist.linear.x = -0.2
						else:
							print('stop-right', g_range_left)
							self.twist.angular.z = vel.angular.z - 0.3
							self.twist.linear.x = -0.2
						
					else:	
						if  g_range_right < g_range_left:
							print('left ', vel.angular.z)
							self.twist.angular.z = vel.angular.z + 0.3
							self.twist.linear.x = 0.1
						else:	
							print('right')
							self.twist.angular.z = vel.angular.z - 0.3
							self.twist.linear.x = 0.1
				elif(g_range_ahead_right <0.2):
					self.twist.angular.z = vel.angular.z + 0.3
					print('too much right vel.ang: ', vel.angular.z)
				elif(g_range_ahead_left <0.1):
					self.twist.angular.z = vel.angular.z - 0.3
					print('too much left vel.ang', vel.angular.z)
					
				else:						
					self.twist.angular.z= 0.0
					self.twist.linear.x = 0.2
				

				self.cmd_vel_pub.publish(self.twist)
				rospy.Rate(10).sleep()
				#print('changed z ',self.twist.angular.z)
			else:
				print('reading img')
				if (g_range_ahead <= 0.5):
					print('stop')
					self.twist.linear.x = -0.1
					if (vel.angular.z >0):
						self.twist.angular.z =0.2
					else: 
						self.twist.angular.z =-0.2
					
				else: 
					print('i\'m free')
					self.twist.linear.x = 0.1

		self.cmd_vel_pub.publish(self.twist)
		rospy.Rate(10).sleep()
			

	def image_callback(self,msg):
		
		#scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
		image=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		#Blue
		low_blue= np.array([100,150,0])
		high_blue = np.array([140,255,255])

		blue_mask = cv2.inRange(hsv, low_blue, high_blue)
		blue = cv2.bitwise_and(image, image, mask= blue_mask)
		
		#conto i pixel della maschera
		#print('blue pixels: ',np.count_nonzero(blue))
		low_green = np.array([25, 52, 72])
		high_green = np.array([102, 255, 255])
		green_mask = cv2.inRange(hsv, low_green, high_green)
		green = cv2.bitwise_and(image, image, mask=green_mask)
		
		#White. Entrance of maze
		low_white = np.array([0,0,0], dtype=np.uint8)
    		high_white = np.array([0,0,255], dtype=np.uint8)
		white_mask = cv2.inRange(hsv, low_white, high_white)
		white = cv2.bitwise_and(image, image, mask=white_mask)
		
		#Yellow. Exit of maze
		low_yellow=np.array([20,100,100])
		high_yellow=np.array([30,255,255])
		yellow_mask=cv2.inRange(hsv,low_yellow,high_yellow)
		yellow=cv2.bitwise_and(image,image,mask=yellow_mask)

		cv2.imshow('window', image)
		#cv2.imshow('blue', blue)
		#cv2.imshow('mask_b', blue_mask)
		#cv2.imshow('white', white)
		#cv2.imshow('mask_w', white_mask)
		cv2.waitKey(10)
		
		if (np.count_nonzero(blue)>210000):
			self.img=True
			#print('blue-right')
			self.twist.linear.x=0.2
			if (g_range_ahead <= 0.3):
				self.twist.linear.x=0
			self.twist.angular.z=-0.3
		elif (np.count_nonzero(green)>250000):
			#print('green')
			self.img=True
			self.twist.linear.x=0.2
			if (g_range_ahead <= 0.3):
				self.twist.linear.x=0
			self.twist.angular.z=0.3
		elif (np.size(image)-np.count_nonzero(white)>250000):
			print(np.count_nonzero(white), 'mask ', np.size(image))
			self.img=True
			self.twist.linear.x=0
			self.twist.angular.z= 2*PI
		elif (np.count_nonzero(yellow)>250000):
			self.twist.angular.z=0.0
			self.twist.linear.x=0.0
		else: 
			self.img=False
			#print('nothing')
			self.twist.linear.x=0.2
			self.twist.angular.z=0.0
		
		self.cmd_vel_pub.publish(self.twist)

	

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
