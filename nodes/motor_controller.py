#!/usr/bin/env python

# This ROS node will take in a set of goals from Astar.py and publish velocity commands that the robot will use to get to the final destination
import math
import numpy
import tf.transformations
import rospy
import re, time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from native.msg import Astar_Goal # ************** Needs to be made first


cmd_vel_topic = '/RosAria/cmd_vel'

# set up a sample goal path
Goal = []
goal = Odometry()
goal.pose.pose.position.x = 0.5
goal.pose.pose.position.y = 0.0
Goal.append(goal)
#goal2 = Odometry()
#goal2.pose.pose.position.y = 0.5
#Goal.append(goal2)

# Helps mover() understand whether we're at the final goal of the path or we're still in the middle of the path
cur_index = 0
final_index = len(Goal) - 1 
goalAchieved = False

# These two variables help mover() understand whether the robot has twisted/advanced far enough to reach the goal
done_twisting = False
done_linear = False

i = 0 # Just a way to figure out how many times you reach mover() function
pose_global = None # A global variable to hold the rospy.subscriber() object


# Variables used to record the deisred angle to twist and desired distance to travel
recalculate_distance = True
distance_to_travel = 0.0
recalculate_angle = True
angle_to_twist = 0.0
cur_pose = Odometry();






def turn(pub, cur_goal_odom, current_odom):
    global recalculate_angle, angle_to_twist
    Pose = current_odom.pose.pose
    

    if recalculate_angle:
        hypo = (((cur_goal_odom.pose.pose.position.x - Pose.position.x) ** 2) + ((cur_goal_odom.pose.pose.position.y - Pose.position.y) ** 2)) ** 0.5
        y = cur_goal_odom.pose.pose.position.y - Pose.position.y
        x = cur_goal_odom.pose.pose.position.x - Pose.position.x
        angle_to_twist = math.degrees(math.asin(y/hypo))
        
        if(x >= 0 and y >= 0): #Quadrant I
            angle_to_twist = angle_to_twist
        elif(x<=0 and y >= 0): #Quadrant II
            angle_to_twist = 180 - angle_to_twist
        elif(x<=0 and y<=0): #Quadrant III
            angle_to_twist = 180 + angle_to_twist
        elif(x>=0 and y<=0): #Quadrant IV
            angle_to_twist = 270 + angle_to_twist

        recalculate_angle = False
    


    # Get angles of robot in (roll, pitch, yaw) in rad
    robot_angles = tf.transformations.euler_from_quaternion((Pose.orientation.x,Pose.orientation.y, Pose.orientation.z, Pose.orientation.w))
    
    yaw_angle = robot_angles[2]*180/math.pi # Convert to degrees
    
    if 180 > yaw_angle > 0:
        yaw_angle = yaw_angle
    else:
        yaw_angle = 360 + yaw_angle
    
    yaw_angle = yaw_angle % 360
    angle_to_twist = angle_to_twist % 360

    if (angle_to_twist - 3.6) < yaw_angle < (angle_to_twist + 3.6) :  
        pub.publish(Twist())
        pub.publish(Twist())

        print "cur_pose:\n\tangle: %.4f\ngoal_pose:\n\tangle: %.4f\n" % (yaw_angle, angle_to_twist)
        print """
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE
                 DOOOOOOOOOOOOOOOOO0OOOOOOOOOOOOOOOOOONE"""
        
        return True
        
    else:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 1.1
        pub.publish(twist)

        recalculate_angle = False
        
        print "cur_pose:\n\tangle: %.4f\ngoal_pose:\n\tangle: %.4f\n" % (yaw_angle, angle_to_twist)
        
        return False





def forward(pub, cur_goal_odom, current_odom):
    global recalculate_distance, distance_to_travel, cur_pose
    twist = Twist() 
    
    Pose = current_odom.pose.pose
    Goal = cur_goal_odom.pose.pose
    if recalculate_distance:
        cur_pose.pose.pose = Pose
        distance_to_travel = math.sqrt(((Goal.position.x - Pose.position.x) ** 2) + ((Goal.position.y - Pose.position.y) ** 2))
        recalculate_distance = False

    distanceRemaining = math.sqrt(((cur_pose.pose.pose.position.x - Pose.position.x) ** 2) + ((cur_pose.pose.pose.position.y - Pose.position.y) ** 2)) # goalLocation - poseLocation
    print("\tDistance to goal: %.4f\n" % (distanceRemaining))

    if distanceRemaining < distance_to_travel: #if the P3-DX has not travelled the distance between its starting point and desired point, 
        twist.linear.x = 0.2
        twist.angular.z = 0
        pub.publish(twist) # then keep moving forward
        return False
    
    else: # once you've travelled far enough,
        print "\tReached goal"
        pub.publish(Twist()) # stop the P3-DX's motion by sending an empty Twist
        return True





def mover(odom, goal):
    # Make sure this fxn can access all of the global variables
    global goalAchieved, cur_index, final_index, done_twisting, done_linear, i
    
    # Print out the mover's status
    i = i + 1
    print "%d In mover function," % i
    
    # Determine the mover()'s action
    if not rospy.is_shutdown():
        pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
        
        # Make sure that the P3-DX has twisted to the direction of the current goal
        if not done_twisting and not done_linear: 
            done_twisting = turn(pub, goal[cur_index], odom)
        if done_twisting and not done_linear:
            done_linear = forward(pub, goal[cur_index], odom)



        # This boolean will let us know if we have reached our current goal
        complete = done_twisting and done_linear
    
        
        if complete and cur_index != final_index: # If we've reached our goal, but did not reach the final goal
            cur_index = cur_index + 1 # increment the index
            
            # Reset the booleans
            done_twisting, done_linear, recalculate_distance = False, False, True, 0.0
            print "cur_index = %d" % cur_index

        elif complete and cur_index == final_index: # If we've reached our goal and it was our final goal
            print "\n\n\nFinal Goal Reached. Leaving mover() function..."
            pub.publish(Twist())
            done_twisting, done_linear, goalAchieved, recalculate_distance = False, False, True, 0.0 # Reset global variables and set goalAchieved to True
            cur_index = 0 # Reset the current index ************** [Should be in motor_controller()]    
    

    else:
        print 'rospy.is_shutdown is on...'
        pub.publish(Twist())

    return

        



# This function will repeatedly check /pose topic to see where the robot is located now until the robot has met its goal
def Get_Pose(the_goal):
    global goalAchieved, final_index, pose_global, Goal
    #final_index = len(Goal)
    # Goal = the_goal;
    rospy.init_node('Motor_Controller')
    pose_global = rospy.Subscriber('/RosAria/pose', Odometry, mover, Goal)
    rospy.spin() # sleep for 0.1 sec    





def shutdown_callback():
    print "At shutting down callback"
    pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    pub.publish(Twist())





# This function will initialize the node and subscribe this node to astar_path
def motor_controller():
    print "motor_controller"
    rospy.init_node('Motor_Controller')
    rospy.Subscriber("Astar_Path", Astar_Goal, Get_Pose)
    
    rospy.spin() # To keep this node alive





if __name__ == "__main__":
    try:
        print "in main()"
        Get_Pose("arg doesn't matter")
    except rospy.ROSInterruptException:
        pass
print "Everything shut off correctly..."

