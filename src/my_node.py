#!/usr/bin/env python
"""
    my_node.py

    A ROS node that repeats the map and odometry topic to the correct ecte477 
    namespace topics for map and path.

    Subscribed: map/, odom/
    Publishes: ecte477/map/, ecte477/e_path/, ecte477/r_path/
    Services: explore/explore_service
    Created: 2021/04/08
    Author: Brendan Halloran
    Updated 12/04/2022 by Jeff Moscrop
"""

import rospy
import time
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import SetBool

class my_node:
    def __init__(self):
        # Initialise subs, pubs, service calls, path object
		
    def callback_map(self, data):
        # Do something with map
        
    def callback_odom(self, data):
        # Do something with odometry
        
    def callback_frontiers(self, frontiers):
        # Do something with frontiers array
	
	
	
# Main function
if __name__ == '__main__':
    rospy.init_node('my_node', anonymous=True)
    rospy.loginfo("Starting My Node!")
    mn = my_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down My Node!")
