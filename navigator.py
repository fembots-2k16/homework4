#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import PoseStamped
    

if __name__ == "__main__":
    print "What... are your ending coordinates?"
    print "x:",
    x = float(raw_input())
    print "y:",
    y = float(raw_input())

    rospy.init_node('homework4_controller')
    rate = rospy.Rate(10)

    velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    motPub = rospy.Publisher("/cmd_motor_state", MotorState, queue_size=10)
    goalPub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    motor = MotorState()
    motor.state = 1

    print "starting motors..."
    for i in xrange(20):
        motPub.publish(motor)
        rate.sleep()

    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    goal = PoseStamped()

    #we'll send a goal to the robot to move 1 meter forward
    goal.header.frame_id = "base_link"
    goal.header.stamp = rospy.get_rostime()

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0

    print "SENDING GOAL"
    while not rospy.is_shutdown():
        goalPub.publish(goal)
