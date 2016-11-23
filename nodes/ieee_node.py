#!/usr/bin/env python

import rospy
import tf
import time
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import subprocess
import numpy





map_data = None





def saveMap(maps):
  global map_data
  map_data = maps
  rospy.loginfo("Saved map")





# Need to figure out how to publish service publish_map
#   msg.global = True; msg.optimized = True; msg.graphOnly = True
#   rtabmap_publish_map = rospy.ServiceProxy('rtabmap/publish_map', msg)
#
#   Read this to see how to publish this: http://answers.ros.org/question/215257/publish-rtab-point-cloud-from-rviz/
def callback(complete_bool):

  global map_data
  map2d = map_data
  map_path = "/home/abdul/catkin_ws/src/native-master/native/maps" # Need to get rosparam

  # Put rtabmap in localization mode so it does not continue to update map after we save it
  # rtabmap_localization_mode = rospy.ServiceProxy('rtabmap/set_mode_localization',Empty())
  # rtabmap_localization_mode()

  # create timestamp for map file
  time_stamp = time.strftime('%Y-%m-%d_%H:%M:%S')
  map_2d_filename = "map_pgm_%s"   % time_stamp
  map_3d_filename = "map_3D_%s.db" % time_stamp

  # save map file using map_server
  print "Saving 2D map to file '%s'" % map_2d_filename
  sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (map_path, map_2d_filename), shell=True)
  if sts == 0: # Save a default copy just named "map" just in case
    sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (map_path, "map"), shell=True)

  # # make backup of the rtabmap DB
  # home_path = os.environ["HOME"]
  # if sts == 0:
  #   sts = subprocess.call('cp "%s/.ros/rtabmap.db" "%s/%s"' % (home_path, map_path, map_3d_filename), shell=True)

  print "\n\tSave Map returned sts %d" % sts



  # Now save the same map in IEEE Std 1873-2015
  fp = open('%s/map_ieee_%s.xml' % (map_path, time_stamp),'wb')
  xml_dateTtime = time_stamp.replace('_','T')
  yaw = tf.transformations.euler_from_quaternion((map_data.info.origin.orientation.x, map_data.info.origin.orientation.y, map_data.info.origin.orientation.z, map_data.info.origin.orientation.w) )
  yaw = float(yaw[2])
  xml_top = '''<?xml version='1.0' encoding='UTF-8'?>



<grid_map id='GridMap' 
          map_type='1' 
          num_cells_x='%d' 
          num_cells_y='%d' 
          mdr_version='IEEE Std 1873-2015' 
          resolution='%f'>


  <metadata>

    <authors>
        <author>Dr. Daniel M. Lofaro</author>
        <author>Dr. Jill K. Nelson</author>
        <author>Abdulwahaab Arif</author>
        <author>Boris Reinosa</author>
        <author>Brendan Kittinger</author>
        <author>Matthew Schell</author>
        <author>Tariq Habib</author>
        <author>William McGovern</author>
    </authors>

    <license>
      Not sure what to put there...
    </license>

    <copyright_owner>
      Team Native of LofaroLabs at George Mason University (2016)
    </copyright_owner>

    <description>
        This file contains a 2D grid of the 3rd Floor of the Nguyen
      Engineering Building at George Mason University. This file was 
      formatted using the IEEE Std. 1873-2015 of a MapArray data 
      type. 
    </description>

    <map_location>
      3rd Floor of Nguyen Engineering Building of George Mason Univeristy
    </map_location>

    <!-- 2014-08-01T21:10:50 -->
    <creation_date> %s </creation_date>
    <last_modified> %s </last_modified>

  </metadata>


  <offset offset_x="%f" offset_y="%f" theta="%f"/>


  <!--<coordinate_system />-->


  <palette_elements>
    <palette value_start='0' value_end='100' meaning='Occupancy is represented as an integer in the range [0,100], with 0 meaning completely free and 100 meaning completely occupied. This value, 0...100, corresponds to probability, 0.0...1.0. The cells that are definitely occupied have a minimum probability of 0.65, while the cells that are definitely empty have a maximum probability of 0.196.'/>
    <palette value_start='-1' meaning='This special value -1 corresponds to completely unknown cells. In other words, cells with the value of -1 are cells that we are not sure if it is occupied or not.'>
  </palette_elements>


  <cells> 
  
''' %   (map2d.info.width, 
         map2d.info.height, 
         map2d.info.resolution, 
         xml_dateTtime, 
         xml_dateTtime, 
         map2d.info.origin.position.x, 
         map2d.info.origin.position.y, 
         yaw)
  
  xml_bottom = """
  </cells>



</grid_map>\n\n"""


  fp.write(xml_top)
  
  width, height = map2d.info.width, map2d.info.height
  
  
  for i in range(height):
    start = i*width
    n = (i+1)*width
    maparray = list(map2d.data[start:n])
    for j in range(width):
      fp.write("    <cell x='%d' y='%d' value='%d'/>\n" % ( j, i, maparray[j] ))

  fp.write(xml_bottom)



  # let user know what happened
  if sts==0:
    print "\n\n\t\t*** Map Saved ok"
  else:
    print "\n\n\t\t*** Save map failed. Status = %d" % sts
  
  
  rospy.signal_shutdown('\n\n\n\tProcess Complete. Shutting Down ROS...\n\n\n')





def save_maps_node():
  rospy.init_node('Map_saver');
  rospy.Subscriber('/map', OccupancyGrid, saveMap)
  sub = rospy.Subscriber('/complete', String, callback)
  rospy.spin()





if __name__=='__main__':
  try:
    save_maps_node()
  except rospy.ROSInterruptException:
    pass


