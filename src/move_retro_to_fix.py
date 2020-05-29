#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
#cv_bridge converts between ROS Image messages and OpenCV images.

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import random 

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
		global retro
		self.retro=False		
		self.img = False
		self.bridge=cv_bridge.CvBridge()
		cv2.namedWindow('Window',cv2.WINDOW_NORMAL)
		cv2.resizeWindow('Window',30,30)
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
				
		if not self.img:
			#print('not img')
			if (g_range_ahead <= 0.4 ):
				if(g_range_behind<= 0.15):
					self.retro=False
				#se il robot si avvicina troppo ad un ostacolo attiviamo il flag retro per avviare una manovra di ri-posizionamento				
				if (g_range_ahead <= 0.15):
					print('retro')
					self.retro=True
					if  g_range_right < g_range_left:
						self.twist.angular.z = vel.angular.z - 0.3
						self.twist.linear.x = -0.1
					else:
						self.twist.angular.z = vel.angular.z + 0.3
						self.twist.linear.x = -0.1
				
				elif not (self.retro):	
					
					if  g_range_right < g_range_left:
						print('left')
						self.twist.angular.z = vel.angular.z + 0.3
						self.twist.linear.x = 0.1
					else:	
						print('right')
						self.twist.angular.z = vel.angular.z - 0.3
						self.twist.linear.x = 0.1
				elif (self.retro):
					print('retro')					
					if  g_range_right < g_range_left:
						self.twist.angular.z = vel.angular.z - 0.3
						self.twist.linear.x = -0.1
					else:
						self.twist.angular.z = vel.angular.z + 0.3
						self.twist.linear.x = -0.1	
			else:	
				if (g_range_ahead >= 0.8 or g_range_behind <= 0.15 ):
					self.retro=False
				self.twist.angular.z=0
				self.twist.linear.x = 0.2

			self.cmd_vel_pub.publish(self.twist)
			rospy.Rate(10).sleep()
			#print('changed z ',self.twist.angular.z)
		else:
			print('yes img')
			

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

		cv2.imshow('window', image)
		cv2.imshow('blue', blue)
		cv2.imshow('mask_b', blue_mask)
		cv2.imshow('green', green)
		cv2.imshow('mask_g', green_mask)
		cv2.waitKey(10)
		
		if (np.count_nonzero(blue)>120000):
			self.img=True
			#print('blue-right')
			self.twist.linear.x=0.2
			self.twist.angular.z=-0.3
			self.cmd_vel_pub.publish(self.twist)
		elif (np.any(green) and g_range_ahead < 1):
			#print('green')
			self.img=True
			self.twist.linear.x=0.2
			self.twist.angular.z=0.3
			self.cmd_vel_pub.publish(self.twist)
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
