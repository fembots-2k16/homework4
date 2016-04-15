#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray

point = None
status = None
goalz = [[] for x in xrange(3)]
curGoal = 0

def callback(data):
    global status
    status = data.GoalStatus
    print "status: ", status
    print data.MoveBaseResult

def subFn(data):
    global point
    print "heyyy"
#    print data
    point = data.point

def aprilFn(array):
    if (len(array.poses) == 0):
        print "unseen"

def evalStatus(stat):
    if (stat == 5):
        # rejected! need to set new goal set new goal
        print "goal rejected"
        

def initGoalz():
    global goalz
    goalz[0] = [12.0797977448, 1.10401678085]
    goalz[0] = [10.6293430328, -0.075917750597] 
    goalz[1] = [6.31805467606, 1.28301894665]

def main():
    global point
    global status
    print 'what... are your ending coordinates?'
    rospy.init_node('homework4_navigator')
    rate = rospy.Rate(10)
    sub = rospy.Subscriber('/clicked_point', PointStamped, subFn)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
    rospy.Subsciber("/tag_detections_pose", PoseArray, aprilFn)

    while (point == None):
       rate.sleep()
    print "x: ", point.x
    x = point.x 
    print "y: ", point.y
    y = point.y


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
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.get_rostime()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    print "Sending goal!"
    client.send_goal(goal)
    while (status != 3):
        rate.sleep()
    
    print "Result:", client.get_result()

    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
