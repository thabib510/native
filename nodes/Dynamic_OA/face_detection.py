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
		if (count==20): # at a rate of 10 frames per second
			obstacle = True
			count = 0
			print '\t Face detected !!!\n'
	else:
		count = 0
#		print 'No face detected...'
	face_pub = rospy.Publisher('face_found', Bool, queue_size = 1);
	face_pub.publish(obstacle)

def det_face():
  rospy.init_node('face_found')
  face_sub = rospy.Subscriber("/person_detection/nb_faces", Int32, callback_face, queue_size=1)
  rospy.spin()


if __name__ == "__main__":
  try:
    det_face()
  except rospy.ROSInterruptException:
    pass
