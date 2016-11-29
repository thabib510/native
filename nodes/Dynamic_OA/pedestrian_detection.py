#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import math
from std_msgs.msg import Bool

#face_count = 0
#body_count = 0

#define Variables
count = 0 #steady frame counter
obstacle = False #publisher flag

def callback_face(data):
	global count, obstacle 
	obstacle = False
	if(data.data!= 0): 
		count = count+1
		if (count>=5):
			obstacle = True
			print '\tPedestrian detected!!!\n'

	else:
		#obstacle =False
		count = 0
#		print 'No pedestrian detected...'
	face_pub = rospy.Publisher('pedestrian_found', Bool, queue_size = 1)
	face_pub.publish(obstacle)

def det_pedestrian():
  rospy.init_node('pedestrian_found')
  face_sub = rospy.Subscriber("/person_detection/nb_pedestrians", Int32, callback_face, queue_size = 1)
  rospy.spin()


if __name__ == "__main__":
  det_pedestrian()
else:
  pass
