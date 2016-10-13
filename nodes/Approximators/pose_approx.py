#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry





def pose_approximation(odom):
  odom.header.stamp.nsecs = int(odom.header.stamp.nsecs*1e-8)*1e8
  pub = rospy.Publisher('/RosAria/pose_approx', Odometry, queue_size=10)
  pub.publish(odom)

def node():
  rospy.init_node('pose_approx_node')
  rospy.Subscriber('/RosAria/pose_map', Odometry, pose_approximation)

  rospy.spin()



if __name__ == "__main__":
  try:
    node()
  except rospy.ROSInterruptException:
    pass
