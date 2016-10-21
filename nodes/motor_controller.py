#!/usr/bin/env python

# motor_controller is a ROS node that operates with Astar_node to move the robot to the desired destination dictated by Astar_node.




from native.msg import path 
import math
import numpy
import tf.transformations
import rospy
import re
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry






# ROS Topics used in this node
cmd_vel_topic = '/RosAria/cmd_vel'
pose_topic    = '/RosAria/pose_bl'

# setting up a simple goal path for motor_controller to follow
#goal = Odometry()
#goal.pose.pose.position.x = 0.5
#goal.pose.pose.position.y = 0.0
#Goal.append(goal)
#goal2 = Odometry()
#goal2.pose.pose.position.x = 0.0
#goal2.pose.pose.position.y = 0.0
#Goal.append(goal2)


# Helps mover() understand whether we're at the final goal of the path or we're at an interim goal of the path
cur_index = 0
final_index = 0
goalAchieved = False

# These two variables help mover() understand whether the robot has twisted/advanced far enough to reach the current goal
done_twisting = False
done_linear = False

i = 0 # Just a way to figure out how many times you access the mover() function. It's used with a print statement
pose_global = None # A global variable to hold the rospy.subscriber() object
threadUsingMover = False


# Variables used to record the deisred angle to twist and desired distance to travel
recalculate_distance = True
distance_to_travel = 0.0
cur_pose = Odometry(); # holds the current pose of the robot before it advances
recalculate_angle = True
angle_to_twist = 0.0
angle_tolerance = 1.5










# This function first determines the desired angle that the robot needs to have and then determines the turning direction of the robot to turn to that desired angle.
#
# NEEDS:   pub           -> Publishing object to the cmd_vel topic
#          cur_goal_odom -> Current Goal's Odometry() object
#          current_odom  -> Robot's current pose as an Odometry() object
#
# RETURNS: True/False to flag whether we've turned enough or need more turning, respectively
def turn(pub, cur_goal_odom, current_odom):
    global recalculate_angle, angle_to_twist
    Pose = current_odom.pose.pose
    twist = Twist()
    
    # If this is the first time in this turn() function, then figure out the desired angle (counterclockwise from positive x-axis) and save that to angle_to_twist
    if recalculate_angle:
        hypo = ((abs(cur_goal_odom.position.x - Pose.position.x) ** 2) + (abs(cur_goal_odom.position.y - Pose.position.y) ** 2)) ** 0.5
        y = cur_goal_odom.position.y - Pose.position.y
        x = cur_goal_odom.position.x - Pose.position.x
        angle_to_twist = math.degrees(math.asin(y/hypo))
        print angle_to_twist
        print x
        print y
        if(x >= 0 and y >= 0): #Quadrant I correct
            angle_to_twist = angle_to_twist
        elif(x<=0 and y >= 0): #Quadrant II correct
            angle_to_twist = 180 - angle_to_twist
        elif(x<=0 and y<=0): #Quadrant III correct
            angle_to_twist = 180 + abs(angle_to_twist)
        elif(x>=0 and y<=0): #Quadrant IV
            angle_to_twist = 360 + angle_to_twist
        
        recalculate_angle = False
        print angle_to_twist
    
    
    # Now, get angles (in rad) of robot in the format (roll, pitch, yaw)
    robot_angles = tf.transformations.euler_from_quaternion((Pose.orientation.x,Pose.orientation.y, Pose.orientation.z, Pose.orientation.w))
    
    yaw_angle = robot_angles[2]*180/math.pi # Convert to degrees
    
    if yaw_angle < 0:
        yaw_angle = yaw_angle + 360

    yaw_angle = yaw_angle % 360 # To make sure that it's a number between 0 and 360
    
    temp_desired_angle = angle_to_twist
    temp_yaw_angle = yaw_angle
    # *************** PROBLEM: What if our angle_of_twist is 1.0 degrees? Then (1 - 3.6) = -2.6 degrees. So, I've added 90 degrees to both angle variables in order to ensure that the range defined in the next if-else statement will involve positive numbers in both extremes
    desired_angle = angle_to_twist
    if (desired_angle - angle_tolerance) < 0:
        temp_desired_angle = (desired_angle + 90) % 360
        temp_yaw_angle     = (yaw_angle     + 90) % 360

    if (temp_desired_angle - angle_tolerance) < temp_yaw_angle < (temp_desired_angle + angle_tolerance) : 
        # Publish an empty Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        
        # Print out status of angles and some sort of success statement 
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
        
    else: # If we're still not in range,
        if desired_angle > yaw_angle:
            if (desired_angle - yaw_angle) < 180:
                twist.linear.x = 0
                twist.angular.z = .4 #turn left
                pub.publish(twist)
            else:
                twist.linear.x = 0
                twist.angular.z = -.4 #turn right
                pub.publish(twist)
        elif (yaw_angle - desired_angle) < 180:
            twist.linear.x = 0
            twist.angular.z = -.4 #turn right
            pub.publish(twist)
        else:
            twist.linear.x = 0
            twist.angular.z = .4 #turn left
            pub.publish(twist)
        
        # Print the status of the angles
        #####print "cur_pose:\n\tangle: %.4f\ngoal_pose:\n\tangle: %.4f\n" % (yaw_angle, angle_to_twist)
        print "Robot Angle: %.4f \t Goal Angle: %.4f\n" % (yaw_angle, angle_to_twist)
        
        return False 
        





# This function first determines the desired distance that the robot needs to travel and then moves the robot forward until it has reached that distance.
#
# NEEDS:   pub           -> Publishing object to the cmd_vel topic
#          cur_goal_odom -> Current Goal's Odometry() object
#          current_odom  -> Robot's current pose as an Odometry() object
#
# RETURNS: True/False to flag whether the robot has travelled that distance or has not completely travelled that distance, respectively
def forward(pub, cur_goal_odom, current_odom):
    global recalculate_distance, distance_to_travel, cur_pose
    twist = Twist() 
    Pose = current_odom.pose.pose
    Goal = cur_goal_odom
    
    # If this is the first time in forward(), then determine the distance the robot needs to travel and record the starting position of the robot
    if recalculate_distance:
        print("inside")
        cur_pose = current_odom
        distance_to_travel = math.sqrt(((Goal.position.x - Pose.position.x) ** 2) + ((Goal.position.y - Pose.position.y) ** 2))
        recalculate_distance = False
    
    Curr = cur_pose.pose.pose

    distanceRemaining = math.sqrt(((Curr.position.x - Pose.position.x) ** 2) + ((Curr.position.y - Pose.position.y) ** 2)) # startingLocation - currentLocation
    print("\tDistance travelled: %.4f\n" % (distanceRemaining))
    print("\tDistance to go: %.4f\n" % (distance_to_travel))
    if distanceRemaining < distance_to_travel: #if the P3-DX has not travelled the complete distance between its starting point and desired point, 
        twist.linear.x  = 0.2
        twist.angular.z = 0.0
        pub.publish(twist) # then keep moving forward
        #print "\tx: %.4f\n\tz: %.4f\n" % (twist.linear.x, twist.angular.z)        
        return False
    
    else: # once the robot has travelled far enough,
        print "\tReached goal"
        twist.linear.x  = 0.0
        twist.angular.z = 0.0
        pub.publish(twist) # stop the robot's motion by sending an empty Twist
        print "\tx: %.4f\n\tz: %.4f\n" % (twist.linear.x, twist.angular.z)
        return True





# This function determines whether the robot needs to turn or move forward. After that, it determines whether the robot has reached its final goal or an intermittent goal
#
# NEEDS:   odom -> Current odometry of the robot
#          goal -> Goals list of Odometry() objects
#
# RETURNS: Nothing
def mover(odom, goal): 
    print goal
    print goal.poses[0].position.x
    global goalAchieved, cur_index, final_index, i, done_twisting, done_linear, recalculate_distance, recalculate_angle, threadUsingMover
    print cur_index
    #if not rospy.is_shutdown() and not threadUsingMover and not goalAchieved: # While rospy has not shut down this node
    if not goalAchieved: # While rospy has not shut down this node
        threadUsingMover = True        

        
        
        
        # This flag tells us if we've reached the current goal
        
        
        if cur_index != final_index: # If we've reached the current goal, but did not reach the final goal in the path
            
            # Print out the mover's status
            i = i + 1
            ###print "%d In mover function" % i 
            pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
            # Determine whether we need to turn, move forward, or completely stop
            if not done_twisting and not done_linear:
                done_twisting = turn(pub, goal.poses[cur_index], odom)
            elif done_twisting and not done_linear:
                done_linear = forward(pub, goal.poses[cur_index], odom)
            else:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub.publish(twist)
            
            complete = done_twisting and done_linear

            if complete:
                cur_index = cur_index + 1 # increment the index
            
                # Reset the flags
                done_twisting, done_linear = False, False 
                recalculate_distance, recalculate_angle = True, True 

                print "cur_index = %d\n\n" % cur_index

        elif cur_index == final_index: # If we've reached our goal and it was our final goal
            goalAchieved = True
            
            # Reset the flags and indexes
            done_twisting, done_linear = False, False 
            recalculate_distance, recalculate_angle = True, True
            cur_index, i = 0, 0 # Reset the current index 
            
            print "\n\n\nFinal Goal Reached. Leaving mover() function..."

        threadUsingMover = False # Let the next thread use this callback    
            
    else:
        print '\n* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *'
        print   '* * * * * * * * Current goal met... waiting for next goal * * * * * * * * * * * *'
        print   '* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n'
    return





# This function runs the ROS node subscriber model
#
# NEEDS:   goal -> Goals list of Odometry() objects
#
# RETURNS: Nothing
def Get_Pose(the_goal):
    global goalAchieved, pose_global, final_index
    final_index = len(the_goal.poses)
    print the_goal
    #rospy.init_node('Motor_Controller')
    pose_global = rospy.Subscriber(pose_topic, Odometry, mover, the_goal)
    rospy.spin()

    #my_spin = rospy.Rate(1) # Gives our motor_controller a frequency of 1Hz
    #while not goalAchieved and not rospy.is_shutdown():
    #    pose_global = rospy.Subscriber(pose_topic, Odometry, mover, Goal)
    #    my_spin.sleep() # sleep for 1 sec
    #print "goal achieved is %s" % str(goalAchieved)
    





# ********************** IGNORE FUNCTION FOR NOW ******************
def motor_controller():
    global goalAchieved
    goalAchieved = False
    print "motor_controller"
    rospy.init_node('Motor_Controller')
    rospy.Subscriber("Path", path, Get_Pose)
    
    rospy.spin() # To keep this node alive
# *****************************************************************




if __name__ == "__main__":
    try:
        print "in main()"
        motor_controller()
    except rospy.ROSInterruptException:
        pass
print "Everything shutted off..."

