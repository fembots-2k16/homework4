#!/usr/bin/env python

import rospy
from p2os_msgs.msg import MotorState

if __name__ == "__main__":
    rospy.init_node("motorer")
    rate = rospy.Rate(10)
    motPub = rospy.Publisher("/cmd_motor_state", MotorState, queue_size=10)

    motor = MotorState()
    motor.state = 1



    print "starting motors..."
    for i in xrange(20):
        motPub.publish(motor)
        rate.sleep()


    print "bye :)!"
