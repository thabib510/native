#!/usr/bin/env python
import rospy
import subprocess
from nav_msgs.msg import OccupancyGrid





def my_map_saver(blah):
  # save the map from /map in .pgm file and .yaml
  map_path = "/home/abdul/catkin_ws/src/native-master/native"
  map_filename = "native_map"
  sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (map_path, map_filename), shell=True)





def native_map_saver():
  rospy.init_node('native_map_saver')
  sub = rospy.Subscriber('/map', OccupancyGrid, my_map_saver)
  rospy.spin()





if __name__ == "__main__":
  try:
    native_map_saver()
  except rospy.ROSInterruptException:
    pass





