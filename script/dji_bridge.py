#!/usr/bin/env python

from utils import *
from quad import quad

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg 

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
from dji_sdk.msg import GlobalPosition
from dji_sdk.msg import TransparentTransmissionData
from dji_sdk.msg import RCChannels

import time
import sys
import math

if __name__ == "__main__":

	nw_corner = rospy.get_param( '~nw_corner' )
	se_corner = rospy.get_param( '~se_corner' )
	drone = DJIDrone()
	Q = quad( drone, nw_corner, se_corner )
		
	rospy.spin()
