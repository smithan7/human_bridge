#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix

# my functions
from custom_messages.msg import DJI_Bridge_Status_MSG, DJI_Bridge_Travel_Path_MSG, DJI_Bridge_Travel_Speed_MSG
from utils import *
from navigation import Navigation


import time
import sys
import math
import numpy as np

class human:
	
	# everything you should need for position, lat lon and local map
	current_global_location = Global_Loc(0.0, 0.0) # lat, lon
	goal_global_location = Global_Loc(0.0, 0.0) # lat, lon
	published_goal_global_location = Global_Loc(0.0, 0.0) # lat, lon
	
	current_local_location = Local_Loc(0.0, 0.0, 0.0, 0.0) # x,y,alt,heading
	goal_local_location = Local_Loc(0.0, 0.0, 0.0, 0.0) # x,y,alt,heading
	published_local_location = Local_Loc(0.0, 0.0, 0.0, 0.0) # x,y,alt,heading

	status_report_time = 0.00 # this prints
	status_report_interval = 3.0

	status_publisher_time = 0.00 # publish status to other nodes
	status_publisher_interval = 0.1

	heartbeat_time = 0.0 # how long since I heard from planner
	max_heartbeat_interval = 2.0

	get_dji_control_time = 0.0 # how often do I attempt to take control of the quad
	get_dji_control_interval = 5.0

	travel_speed = 0.0
	gps_up_to_date = False

	state = "waiting for travel path"

	status_publisher = rospy.Publisher('/dji_bridge_status', DJI_Bridge_Status_MSG, queue_size=10)
	odom_publisher = rospy.Publisher('/dji_bridge_odom', Odometry, queue_size=10)

	def __init__(self, nw, se):
		nw_corner = Global_Loc(nw[1], nw[0])
		se_corner = Global_Loc(se[1], se[0])
		self.navigation = Navigation( nw_corner, se_corner )

		# get true odom from the GPS
		rospy.Subscriber("/ublox_gps_node/fix", NavSatFix, self.callback_gps)

		self.RAGS_query_time = rospy.get_time()
		print("Human_Bridge::Human Initialized")

	def publish_status_msg( self ):
		if rospy.get_time() - self.status_publisher_time > self.status_publisher_interval:
			self.status_publisher_time = rospy.get_time()
			report = DJI_Bridge_Status_MSG()
			
			report.longitude = self.current_global_location.longitude # x loc
			report.latitude = self.current_global_location.latitude # y loc
			report.altitude = self.current_local_location.altitude # z loc
			report.heading = self.current_local_location.heading # heading
			report.goal_longitude = self.goal_global_location.longitude # goal location ~ travel_path[-1]
			report.goal_latitude = self.goal_global_location.latitude # goal location
			report.local_x = self.current_local_location.local_x
			report.local_y = self.current_local_location.local_y
			report.local_goal_x = self.goal_local_location.local_x
			report.local_goal_y = self.goal_local_location.local_y

			if self.state == "waiting for travel path":
				report.waiting = True
				report.travelling = False
			elif self.state == "travelling":
				report.waiting = False
				report.travelling = True

			if self.state == "in emergency stop":
				report.emergency_stopped = True
			else:
				report.emergency_stopped = False

			self.status_publisher.publish( report )


	def print_status_report( self ):
		if( rospy.get_time() - self.status_report_time > self.status_report_interval ):
			self.status_report_time = rospy.get_time()
			print("Human_Bridge::in action server at state: ", self.state , ", " , self.current_global_location.longitude, " / ", self.current_global_location.latitude, ", ", self.current_local_location.local_x, " / ", self.current_local_location.local_y)
			if self.state == "travelling":
				print("dist to goal: ", abs(self.goal_local_location.local_x - self.current_local_location.local_x), " , ", abs(self.goal_local_location.local_y - self.current_local_location.local_y), " (m)")

	def callback_gps(self, data):

		# update location in navigation
		[self.current_local_location.local_x, self.current_local_location.local_y] = self.navigation.update_location(data)
		self.current_global_location.longitude = data.longitude
		self.current_global_location.latitude = data.latitude

		# print a status report
		self.print_status_report()
		# publish status report
		self.publish_status_msg()

