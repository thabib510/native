#!/bin/bash/env python

import rospy
from sensor_msgs.msg import PointCloud
import math
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

#define variables
threshold = 0.45 # distance from center of robot to the obstacle
obstacle = False #obstacle detected flag
dyn_count = 0
stat_count = 0
tempa = False
handling = False
new = True
cmd_vel_topic = '/RosAria/cmd_vel'
dynamic_obstacle = False

def nothing():
  return True

sonar_sub = rospy.Subscriber('None', Bool, nothing)

def callback(data):
  global obstacle, tempa, threshold, new, sonar_sub, stat_count, handling
  if not tempa:
    tempa = True
    behind = False # checks behind the robot before reversing
    obstacle = False
    for i in range(len(data.points)):
      point = data.points[i]
      if(i >= 0 and i <= 7):
        hyp = math.sqrt(point.x ** 2 + point.y ** 2)
        #print '\tindex:\t%d\thyp:%.2f\n---' % (i, hyp)
        if(hyp <= threshold):
          obstacle = True
          print('\tStatic Obstacle Detected!!!\tindex:\t%d\thyp:%.2f\n---' % (i, hyp))
        else:
          print('No Obstacle Detected')
      if(i > 7): #check behind the robot
        hyp = math.sqrt(point.x ** 2 + point.y ** 2)
        if(hyp <= .45):
          behind = True
    sonar_sub = rospy.Subscriber('None', Bool, nothing)
    if (obstacle and new) or handling:
      handling = True
      pub = rospy.Publisher('OAH',Bool, queue_size = 1)
      pub.publish(True)
      pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
      twist = Twist()
      if(behind): #object behind stop moving and mark obstacle as handled
        twist.linear.x = 0.0
        twist.angular.z= 0.0 
        pub.publish(twist)
        obstacle = False
        obstacleFlg = False
        handling = False
        pub = rospy.Publisher('OAH',Bool, queue_size = 1)
        pub.publish(False)
      elif(stat_count < 3):
        twist.linear.x = -0.2
        twist.angular.z = 0.0
        pub.publish(twist)
        stat_count = stat_count + 1
        time.sleep(1)
      else:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        pub = rospy.Publisher('OAH',Bool, queue_size = 1)
        pub.publish(False)
        stat_count = 0
        obstacle = False
        obstacleFlg = False
        handling = False
        print 'Static Obstacle Handled'
    else:
      pub = rospy.Publisher('OAH',Bool, queue_size = 1)
      pub.publish(False)
      obstacle = False
      obstacleFlg = False
      handling = False
    
    tempa = False 

def dynamic(detect):
  global dynamic_obstacle, dyn_count
  dynamic_obstacle = detect.data
  print(dynamic_obstacle)
  
  if(dynamic_obstacle and dyn_count < 10):
    pub = rospy.Publisher('OAH',Bool, queue_size = 1)
    pub.publish(True)
    pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    time.sleep(1) #check to see if obstacle is still there every one second.
    dyn_count = dyn_count + 1
    if(dyn_count >= 10):
      print 'Dynamic Obstacle Timed Out'
      dynamic_obstacle = False
      pub = rospy.Publisher('OAH',Bool, queue_size = 1)
      pub.publish(False)
    
  elif(not dynamic_obstacle):
    dyn_count = 0
    print 'no obstacle'
    dynamic_obstacle = False
    pub = rospy.Publisher('OAH',Bool, queue_size = 1)
    pub.publish(False)


def main():
  global new, sonar_sub
  rospy.init_node('OAH')
  sonar_sub = rospy.Subscriber('/RosAria/sonar', PointCloud, callback, queue_size = 1)
  dyn_sub = rospy.Subscriber('/Dynamic_OA/face_found', Bool, dynamic, queue_size = 1)
  new = True
  rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
print "Everything shutted off..."
