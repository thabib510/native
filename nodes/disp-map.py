#!/usr/bin/env python
import subprocess
import rospy
import re
import time
import numpy
from matplotlib import pyplot
from nav_msgs.msg import OccupancyGrid
from matplotlib import animation



# This function reads in a pgm file and saves it in a list-of-lists format
def read_pgm(filename, byteorder='>'): 
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
        
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))




def updatefig(*args):
  global image, total_map_path, im, map_path, map_file
  sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (map_path, map_file), shell=True)
  image = read_pgm(total_map_path, byteorder='<') # read pgm file and save it as a matrix (list-of-lists)
  im.set_array(image)
  return im,





def disp_map():
  global image, total_map_path, im
  
  # Save the map
  map_path = "/home/abdul/catkin_ws/src/native-master/native/"
  map_file = "test_2D_map"
  sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (map_path, map_file), shell=True)
  
  # Display the map
  total_map_path = "%s/%s.pgm" % (map_path, map_file)
  image = read_pgm(total_map_path, byteorder='<') # read pgm file and save it as a matrix (list-of-lists)
  fig = pyplot.figure()
  im = pyplot.imshow(image, pyplot.cm.gray, animated=True) # plots everything in a grayscale format
  
  ani = animation.FuncAnimation(fig, updatefig, interval=5000, blit=True)
  pyplot.show() # show that plot and wait till its existed to close the program




image = 0
total_map_path = 0;
im = 0;
map_path = "/home/abdul/catkin_ws/src/native-master/native/"
map_file = "test_2D_map"
if __name__ == "__main__":
  try:
    disp_map()
  except rospy.ROSInterruptException:
    pass






