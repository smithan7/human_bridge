#!/usr/bin/env python

import rospy
from human import human

if __name__ == "__main__":
	


	rospy.init_node("Human_Bridge")
	environment_number = rospy.get_param('~test_environment_number') 
	nw_corner = []
	se_corner = []

	# hardware test environment
	if environment_number == 3:
		nw_corner = [44.539847, -123.251004]
		se_corner = [44.538552, -123.247446]

	# practice field
	if environment_number == 0:
		nw_corner = [44.565683, -123.272974]
		se_corner = [44.564965, -123.270456]

	# bigger test environment
	if environment_number == 4:
		nw_corner = [44.539457, -123.250866]
		se_corner = [44.538162, -123.247776]

		# bigger test environment
	if environment_number == 5:
		nw_corner = [44.539457, -123.250866]
		se_corner = [44.538162, -123.247776]

	if environment_number == 5:	
		nw_corner = [44.539580, -123.250876]
		se_corner = [44.538725, -123.247601]

	h = human( nw_corner, se_corner )	
	rospy.spin()
