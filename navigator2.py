#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    print 'what... are your ending coordinates?'
    print "x: "
    x = float(raw_input())
    print "y: "
    y = float(raw_input())

    rospy.init_node('homework4_navigator')
    rate = rospy.Rate(10)

    velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    motPub = rospy.Publisher("/cmd_motor_state", MotorState, queue_size=10)
    #goalPub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    motor = MotorState()
    motor.state = 1

    print "make sure you start the motors!"
    #for i in xrange(20):
    #    motPub.publish(motor)
    #    rate.sleep()

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #Waits until the action server has started up and started listening for goals
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "move_base"
    goal.target_pose.header.stamp = rospy.get_rostime()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    print "Sending goal!"
    client.send_goal(goal)

    print "Result:", client.get_result()
