#!/usr/bin/env python





from native.msg import path # custom message of an array of geometry_msgs/pose objects
import rospy
from nav_msgs.msg import Odometry
import time






# The topics that I'm currently using to test this node
pose_topic    = '/RosAria/pose_bl'

cur_index = 0 # index of the path object 
final_index_plus_one = 0 # Final index of the path object
goalAchieved = False # Checks if we've travesed through the list of /pose objects successfully

threadUsingMover = False # To make sure that during multi-threading, only the recent thread will access it









def mover(odom, goal): # For every odometry message and list of goals, do the following stuff
    global goalAchieved, cur_index, final_index_plus_one, threadUsingMover
    
    if not rospy.is_shutdown() and not threadUsingMover and not goalAchieved: # While rospy has not shut down this node
    #if not goalAchieved: 
        threadUsingMover = True

        if cur_index < final_index_plus_one: # if we haven't completely traversed the list of goals
            
            # Do some stuff

            cur_index = cur_index + 1 # when done, increment the index
            print cur_index
            time.sleep(0.5)
            


        else:
            print '\n* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *'
            print   '* * * * * Current goal met... waiting for next goal * * * *'
            print   '* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n'            
            goalAchieved = True
        
        threadUsingMover = False # Let the next thread use this callback    
            
    else:
        print '-------------------------------------------------------------'



    return






def Get_Pose(the_goal): # Once it receives the list of goals, keep subscribing to odometry_topic until we reach the final goal
    global goalAchieved, pose_global, final_index_plus_one
    final_index_plus_one = len(the_goal.poses) 

    rate = rospy.Rate(15) # The rate of the odometry topic is 10Hz
    while not goalAchieved: # while we haven't reached our goal, keep subscribing in a controlled manner
        pose_global = rospy.Subscriber(pose_topic, Odometry, mover, the_goal)
        rate.sleep()

    goalAchieved = False
    pose_global.unregister() # Exit this function completely and close the thread 
                             #   and wait for the next message to be published to /Path topic

    





def motion_controller(): # this is where we get a list of paths to traverse
    global goalAchieved
    goalAchieved = False # Make sure that goalAchieved is FALSE
    print "motion_controller"

    rospy.init_node('Motion_Controller')
    rospy.Subscriber("Path", path, Get_Pose)
    
    rospy.spin() 




if __name__ == "__main__":
    try:
        print "in main()"
        motion_controller()
    except rospy.ROSInterruptException:
        pass

