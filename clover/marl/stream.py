#!/usr/bin/env python3
# simulate a stream of input trajectorys

import rospy
from geometry_msgs.msg import Twist
import random
import math
from clover import srv

navigate = rospy.ServiceProxy('navigate', srv.Navigate)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

def stream():
    pub = rospy.Publisher('trajectory_stream', Twist, queue_size=10)
    rospy.init_node('test_publisher', anonymous=True)
    rate = rospy.Rate(10)  # set publish rate to 10 Hz

    speed, direction = 1, 0
    vect = Twist()
    while not rospy.is_shutdown():
        vect.linear.x = speed  # forward speed
        vect.angular.z = direction  # direction in radians

        pub.publish(vect)
        print(f"sending: {speed}, {direction}")
        direction = (direction + 0.2) % (2*math.pi)
        rate.sleep()


def square():
    pub = rospy.Publisher('trajectory_stream', Twist, queue_size=10)
    rospy.init_node('test_publisher', anonymous=True)
    rate = rospy.Rate(10)  # set publish rate to 10 Hz

    speed, direction = 1, 0
    vect = Twist()
    counter = 0
    while not rospy.is_shutdown():
        vect.linear.x = speed  # forward speed
        vect.angular.z = direction  # direction in radians

        pub.publish(vect)
        print(f"sending: {speed}, {direction}, {vect.linear.z}")
        if counter == 50:
            direction = (direction + (math.pi/2)) % (2*math.pi)
            counter = 0
            vect.linear.z = 1 if vect.linear.z == 2 else 2
        counter += 1
        rate.sleep()


if __name__ == "__main__":
    stream()
