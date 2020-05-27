#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


def scan_callback(msg):
	global g_range_1
	global g_range_2
	global g_range_3
	global g_range_4
	
	global g_range_ahead
	global g_range_right
	global g_range_left
	global g_range_behind

	g_range_1=min(msg.ranges[0:89]) 
	g_range_2=min(msg.ranges[90:179])
	g_range_3=min(msg.ranges[180:269])
	g_range_4=min(msg.ranges[270:360])
		
	
	print('min scan-1', g_range_1)
	print('min scan-2', g_range_2)
	print('min scan-3', g_range_3)
	print('min scan-4', g_range_4)
	
	g_range_left=min(msg.ranges[45:134]) 
	g_range_ahead=min(min(msg.ranges[0:44]),min(msg.ranges[315:359]))
	g_range_right=min(msg.ranges[225:314])
	g_range_behind=min(msg.ranges[135:224])

	print('g_range_left', g_range_left)
	print('g_range_ahead', g_range_ahead)
	print('g_range_right', g_range_right)
	print('g_range_behind', g_range_behind)
	
rospy.init_node('scan_values')
sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()
