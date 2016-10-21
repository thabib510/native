#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import message_filters



def callback(odom, goal):

	print "\todom: %d\n\tgoal: %d\n\n" % (odom.header.stamp.nsecs, goal.header.stamp.nsecs)

def node():
	rospy.init_node('Synchronize_Subscriptions')
	pose_sub = message_filters.Subscriber('/RosAria/pose_approx', Odometry)
  	goal_sub = message_filters.Subscriber('/goal_approx', Odometry)

  	ts = message_filters.TimeSynchronizer([pose_sub, goal_sub], 10) # Synchronizes the time stamps and checks if they're ever equal
  	ts.registerCallback(callback)

  	rospy.spin()



if __name__=="__main__":
 	node()
