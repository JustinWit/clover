#!/usr/bin/env python3

from evdev import InputDevice
import rospy
from geometry_msgs.msg import Twist
import math

#SET THIS TO YOUR DEVICE
device = InputDevice('/dev/input/event7')
# print(device.capabilities(verbose=True))  # use this to view devices

# ros publisher
pub = rospy.Publisher('trajectory_stream', Twist, queue_size=1)
rospy.init_node('test_publisher', anonymous=True)

        
touch = False
start_x = None
start_y = None
vect = Twist()

while not rospy.is_shutdown():
    event = device.read_one()
    if event is not None:
        # wait for first touch
        if not touch and event.code == 330:
            touch = True

        # release start coords
        elif touch and event.code == 330:
            touch = False
            start_x = None
            start_y = None

        # grab x and y on initial touch
        elif touch and event.code == 53 and start_y is None:
            start_y = event.value
            rel_y = 0

        elif touch and event.code == 54 and start_x is None:
            start_x = event.value
            rel_x = 0

        # send coords relative to start
        elif touch and event.code == 53 and start_y is not None:
            rel_y = event.value - start_y
            speed = min(5, math.sqrt(rel_x**2 + rel_y**2)/500)
            direction = math.atan2(rel_x, rel_y)

            vect.linear.x = speed
            vect.angular.z = direction
            pub.publish(vect)
        
        elif touch and event.code == 54 and start_x is not None:
            rel_x = start_x - event.value
            speed = min(5, math.sqrt(rel_x**2 + rel_y**2)/500)
            direction = math.atan2(rel_x, rel_y)
            
            vect.linear.x = speed
            vect.angular.z = direction
            pub.publish(vect)

        # TODO: height control with scroll gesture 
    