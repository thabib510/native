#!/bin/bash/env python

import rospy
from sensor_msgs.msg import PointCloud
import math
import time
from std_msgs.msg import Bool

#define variables
threshold = 0.4 # distance from center of robot to the obstacle
obstacle = False #obstacle detected flag
tempa = False

def callback(data):
  global obstacle, tempa, threshold
  temp = 0
  pub = rospy.Publisher('static_OA', Bool, queue_size = 1)
  if not tempa:
    tempa = True

    obstacle = False
    for i in range(len(data.points)):
      point = data.points[i]
      if(i >= 0 and i <= 7):
        hyp = math.sqrt(point.x ** 2 + point.y ** 2)
        #print '\tindex:\t%d\thyp:%.2f\n---' % (i, hyp)
        if(hyp <= threshold):
          temp = temp + 1
          print('\tStatic Obstacle Detected!!!\tindex:\t%d\thyp:%.2f\n---' % (i, hyp))
        else:
          print('No Obstacle Detected')

    if temp > 0 and not obstacle:
      obstacle = True
      pub.publish(True)
    else:
      obstacle = False
      pub.publish(False)
    
    tempa = False 

def main():
  rospy.init_node('static_OA')
  sonar_sub = rospy.Subscriber('/RosAria/sonar', PointCloud, callback, queue_size = 1)
  rospy.spin()


if __name__ == '__main__':
  main()
