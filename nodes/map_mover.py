#!/usr/bin/env python

'''This node will publish ROS Twist messages to a desired topic. These messages can help a user control a robot. 

For our purposes, we will be using this node in congruence with the ARC to control a P3-DX (interfaced with RosAria).

This node was originally done in a ROS package called "p3dx_mover". Their goal was to control a P3-DX using the keyboard arrow keys. We, Team NATIVE, modified that code to apply the same commands to our ARC's Joystick Keys using a UDP server. 

The ARC's arrow keys will be used as a mapping utility. For example, the UP (@) arrow will save a map that's generated in RTAB-Map by copying the existing temporary database and saving it.

'''

# All of the relevant libraries
import rospy
from socket import *
import re
from geometry_msgs.msg import Twist
import time
import subprocess
import os
from std_srvs.srv import Empty






def save_map():

  map_path = "/home/abdul/catkin_ws/src/native-master/native/maps"

  # Put rtabmap in localization mode so it does not continue to update map after we save it
  rtabmap_localization_mode = rospy.ServiceProxy('rtabmap/set_mode_localization',Empty())
  rtabmap_localization_mode()
        
  # create timestamp for map file
  time_stamp = "%s" % time.ctime().replace(":","_").replace(" ","_")
  map_file = "native_2Dmap_%s" % time_stamp
  db_file = "native_3Dmap_%s.db" % time_stamp

  # save map file using map_server
  print "Saving 2D map to file '%s'" % map_file
  sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (map_path, map_file), shell=True)

  # Save a default copy just named "map" also
  if sts == 0:
    sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (map_path, "map"), shell=True)

  # make backup of the rtabmap DB
  dbPath = os.environ["HOME"]
  if sts == 0:
    sts = subprocess.call('cp "%s/.ros/rtabmap.db" "%s/.ros/%s"' % (dbPath, dbPath, db_file), shell=True)

  print "\n\tSave Map returned sts %d" % sts

  # let user know what happened
  if sts==0:
    print "\n\n\t\t*** Map Saved ok"
  else:
    print "\n\n\t\t*** Save map failed. Status = %d" % sts






def mover(topic_name):
  # Variables responsible for the controller's maximum speeds
  zTwist = 0.0000
  xTwist = 0.0000
  forward = 0.0
  left = 0.0
  Max_Forward = 0.8
  Max_Side = 0.9
  
  # This segment contains some preliminary items needed so that the socket can listen to anyone who would access its port
  host = "" # So that IP doesn't matter
  port = 2362 # Remember, this is the port that you MUST connect to on ARC
  buf = 1024 # Buffer size
  addr = (host, port)
  UDPSock = socket(AF_INET, SOCK_DGRAM)
  UDPSock.bind(addr)

  print "Waiting to receive messages..." # This printout tells us that our server is ready 


  # Initializing Publisher node
  pub = rospy.Publisher(topic_name, Twist, queue_size=10)
  rospy.init_node('p3dx_mover')

  while True: # Our server will continue receiving messages forever until the SELECT key on the ARC is pressed

    try:
      twist = Twist() # This instantiated object will hold the relative speed of the robot and the direction of that speed
      (data, addr) = UDPSock.recvfrom(buf) # Socket waits for a command to be sent from the ARC client
      buttonPressed=str(data) # Save that data into a string (if it already isn't)
      
      # If the user presses SELECT key, then the controller must be closed
      if buttonPressed == "button select 1":  
        print "button select 1\n\n\t*** Closing Controller Connection...\n"
        break # Get out of the while loop

      # If "Arrow" Keys are pressed
      elif (buttonPressed == "button @ 1") and (forward <= Max_Forward):
        save_map()
        continue
      elif (buttonPressed == "button % 1") and (left    <= Max_Side):
        left    += 0.1 # Go right
      elif (buttonPressed == "button & 1") and (forward >= -1*Max_Forward):
        forward -= 0.1 # Go backward
      elif (buttonPressed == "button # 1") and (left    >= -1*Max_Side):
        left    -= 0.1 # Go left
      
      else: # However, if the joysticks are pressed
        zTwist = 0.0
        joyxy = re.findall(r"[-+]?\d*\.\d+|\d+", str(data)) # Determine if the message is indeed resembeling a Joystick command
        rightOrLeft = str(data).split(' ') # Split the message using the space as a delimiter

        if rightOrLeft[1] == "right":	#changes x twist i.e. determines the forward backward (F/B) speed of the robot
          rl_msg = "Right Joystick"
        
          temp1 = float(joyxy[1]) 
          if temp1 > 0.2:
            xTwist = (temp1 - 0.2) / 0.8
          elif temp1 < -0.2:
            xTwist = (temp1 + 0.2) / 0.8
          else:
            xTwist = 0 # If the joystick is close to origin +-0.2, then it is considered to be zero

        if rightOrLeft[1] == "left":	#changes z twist i.r. determines the left right (L/R) speed of the robot
          rl_msg = "Left Joystick"
          temp2 = float(joyxy[0])
          if temp2 > 0.2:
            zTwist = (temp2 - 0.2) / 0.8
          elif temp2 < -0.2:
            zTwist = (temp2 + 0.2) / 0.8
          else:
            zTwist = 0 # same as xTwist
      
        left = zTwist * Max_Side # Determine speed L/R
        forward = xTwist * Max_Forward # Determine speed F/B
      
      
      print("%s\nForward: %+.4f    Left/Right: %+.4f\n" %(rl_msg, forward, left))
      twist.linear.x = forward
      twist.angular.z = -1 * left
      pub.publish(twist)


    except error as e: # Catch a socket.error that may occur when this node is unexpectedly closed via a KeyboardInterrupt
      print "\n\n\t*** Encountered socket.error\n\t\tNode will now unexpectedly close..."
      break  
  
  
  
  # This segment ensures that the robot stops when the user presses SELECT by sending a Twist() object of linear and angular speeds of zero (initialized state) 
  twist = Twist() # Make an empty twist 
  pub.publish(twist) # Publish that empty twist to stop the P3-DX's motor movements
  
  UDPSock.close() # Close the UDP socket
  print "\t*** mover.py node is now closed."
  exit()





# Get the cmd_vel_topic parameter from the launch file. It will publish its Twist msgs to "/RosAria/cmd_vel" topic by default
topic = rospy.get_param("cmd_vel_topic","/cmd_vel")

print "Publishing to the cmd_vel topic: %s" % topic


if __name__ == "__main__":
  try:
    mover(topic)
  except rospy.ROSInterruptException:
    pass

