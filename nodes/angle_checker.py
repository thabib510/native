#!/usr/bin/env python

# This ROS node will take in a set of goals from Astar.py and publish velocity commands that the robot will use to get to the final destination
import math
import numpy
import tf.transformations
import rospy
import re, time
from nav_msgs.msg import Odometry









def mover(odom):
    Pose = odom.pose.pose
    euler = tf.transformations.euler_from_quaternion((Pose.orientation.x,Pose.orientation.y, Pose.orientation.z, Pose.orientation.w))
    theta_p3dx = euler[2]*180/math.pi
    if 180 > theta_p3dx > 0:
        theta_p3dx = theta_p3dx
    else:
        theta_p3dx = 360 + theta_p3dx

    print("angle:\t%.4f\n" % (theta_p3dx))
    return

        



# This function will repeatedly check /pose topic to see where the robot is located now until the robot has met its goal
def Get_Angle():

    
    rospy.init_node('Angle_Measurer')
    my_spin = rospy.Rate(30) # Gives our motor_controller a frequency of 10Hz
    while not rospy.is_shutdown():
        pose_sub = rospy.Subscriber('/RosAria/pose_bl', Odometry, mover)
        my_spin.sleep() # sleep for 0.1 sec
        pose_sub.unregister()

    





if __name__ == "__main__":
    try:
        Get_Angle()
    except rospy.ROSInterruptException:
        pass

