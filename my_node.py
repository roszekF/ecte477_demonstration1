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
        self.explore_srv = rospy.ServiceProxy('/explore/explore_service', SetBool)
        
        self.maze_pub = rospy.Publisher('/ecte477/maze', OccupancyGrid, queue_size=10)
        self.move_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.e_pub = rospy.Publisher('ecte477/e_path', Path, queue_size=10)
        self.r_pub = rospy.Publisher('ecte477/r_path', Path, queue_size=10)

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.frontiers_sub = rospy.Subscriber('explore/frontiers', MarkerArray, self.callback_frontiers)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)

        # constructiong goal message
        self.goal = PoseStamped()
        self.goal.header.stamp = rospy.Time.now()
        self.goal.header.frame_id = 'map'
        self.goal.pose.position.x = 0
        self.goal.pose.position.y = 0
        self.goal.pose.orientation.w = 1

        # constructing explorer path message
        self.e_path = Path()
        self.e_path.header.frame_id = 'odom'

        # constructing return path message
        self.r_path = Path()
        self.r_path.header.frame_id = 'odom'

        self.is_exploration_finished = False


        rospy.sleep(8) # sleep for 8 seconds
        print "Starting exploration!"
        try:
            self.explore_srv(True)
        except rospy.ServiceException, e:
            print "Service call failed: {}".format(e)


    def callback_map(self, data):
        # republish the data
        self.maze_pub.publish(data)
        

    def callback_odom(self, data):
        # construct a new pose and assign the odom data
        new_pose = PoseStamped()
        new_pose.header = data.header
        new_pose.pose = data.pose.pose

        if not self.is_exploration_finished:
            self.e_path.poses.append(new_pose)
            self.e_pub.publish(self.e_path)
        else:
            self.r_path.poses.append(new_pose)
            self.r_pub.publish(self.r_path)
        
        
    def callback_frontiers(self, frontiers):
        # check if the array is empty
        if len(frontiers.markers) == 0:
            self.is_exploration_finished = True

            print "Exploration has finished!"
            rospy.sleep(5)

            print "Going to the starting point!"
            self.move_pub.publish(self.goal)


# Main function
if __name__ == '__main__':
    rospy.init_node('my_node', anonymous=True)
    rospy.loginfo("Starting My Node!")
    mn = my_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down My Node!")

