#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry




def goal_approximation(goal):
  goal.header.stamp.nsecs = (int(goal.header.stamp.nsecs*1e-8)+1)*1e8
  pub = rospy.Publisher('/goal_approx', Odometry, queue_size=10)
  odom = Odometry()
  odom.header.seq            = goal.header.seq
  odom.header.stamp.secs     = goal.header.stamp.secs
  odom.header.stamp.nsecs    = goal.header.stamp.nsecs
  odom.header.frame_id       = goal.header.frame_id
  #odom.child_frame_id        = "base_link"
  odom.pose.pose.position    = goal.pose.position
  odom.pose.pose.orientation = goal.pose.orientation
  pub.publish(odom)

def node():
  rospy.init_node('goal_approx_node')
  rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_approximation)

  rospy.spin()



if __name__ == "__main__":
  try:
    node()
  except rospy.ROSInterruptException:
    pass
