import math
import numpy
import rospy
import re
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import time


Sonar_topic   = 'static_OA'
cmd_vel_topic = '/RosAria/cmd_vel'

#Static Obstacle Flag
obstacle = False
temp = False

def setVariable(obstacleFlg):
    global obstacle, temp
    if(temp == False):
      temp = True
      obstacle = obstacleFlg.data
      print(obstacle)
      pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
      twist = Twist()
      if(obstacle):
          twist.linear.x = 0.0
          twist.angular.z= 0.0 
          pub.publish(twist)
          #time.sleep(2)
          twist.linear.x = -0.2
          twist.angular.z = 0.0
          pub.publish(twist)
          time.sleep(3)
          twist = Twist()
          twist.linear.x = 0.0
          twist.angular.z = 0.0
          pub.publish(twist)
          obstacle = False
          obstacleFlg = False
          print 'Obstacle Handled'
      temp = False

def OA_Handler():
    print "In OA_Handler"
    rospy.init_node('OA_Handler')
    sonar_sub = rospy.Subscriber(Sonar_topic, Bool, setVariable)
    rospy.spin() # To keep this node alive





if __name__ == "__main__":
    try:
        OA_Handler()
    except rospy.ROSInterruptException:
        pass
print "Everything shutted off..."
