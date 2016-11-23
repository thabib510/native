#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from native.msg import path
from time import sleep

i = 1
goalAchieved = False
threadUsingMover = False
if_mover_is_going_to_a_goal = False

def callback2(goal):
  global goalAchieved, i 
  i = 1
  print 'goalAchieved set to False by callback2'
  goalAchieved = False
  return


def callback1(pose):
  global i, goalAchieved, if_mover_is_going_to_a_goal 

  if not if_mover_is_going_to_a_goal:
    if_mover_is_going_to_a_goal = True
    if (i % 8) != 0 and if_mover_is_going_to_a_goal:
      print "%d" % i
      i = i + 1
      sleep(1)
    else:
      print 'TRUE %d' % i
      i = i + 1
      sleep(1)
      goalAchieved = True
    if_mover_is_going_to_a_goal = False
  return
  
def node():
  rospy.init_node('tester');
  goal_sub = rospy.Subscriber('/Path', path, callback2)
  pose_sub = rospy.Subscriber('/RosAria/pose_bl', Odometry, callback1)
  rospy.spin()

if __name__ == '__main__':
  try:
    node()
  except rospy.ROSInterruptException:
    pass