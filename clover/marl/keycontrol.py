#!/usr/bin/env python3

from pynput import keyboard
import rospy
from geometry_msgs.msg import Twist
import math

pub = rospy.Publisher('trajectory_stream', Twist, queue_size=10)
rospy.init_node('test_publisher', anonymous=True)


def on_press(key):
    vect = Twist()
    if key == keyboard.Key.up:
        vect.linear.x = 1  # forward speed
        vect.angular.z = math.pi/2  # direction in radians

    elif key == keyboard.Key.down:
        vect.linear.x = 1  # forward speed
        vect.angular.z = 3*math.pi/2  # direction in radians

    elif key == keyboard.Key.left:
        vect.linear.x = 1  # forward speed
        vect.angular.z = math.pi  # direction in radians

    elif key == keyboard.Key.right:
        vect.linear.x = 1  # forward speed
        vect.angular.z = 0  # direction in radians

    else:
        vect.linear.x = 0  # forward speed
        vect.angular.z = 0  # direction in radians

    # send message
    pub.publish(vect)

def on_release(key):
    vect = Twist()
    vect.linear.x = 0  # forward speed
    vect.angular.z = 0  # direction in radians
    pub.publish(vect)


    if key == keyboard.Key.esc:
        # Stop listener
        return False


# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

rospy.spin()