#!/usr/bin/env python3

import rospy
from clover.msg import SendCoords
import random

def grid_pattern():
    pub = rospy.Publisher('set_pos', SendCoords, queue_size=10)
    rospy.init_node('test_publisher', anonymous=True)

    x = [0, 1, 2, 3, 4]
    y = [4, 3, 2, 1, 0]

    pub.publish(0, 0)
    rospy.sleep(5)

    for i in x:
        y.reverse()
        for j in y:
            print(f"Moving to {i}, {j}")
            pub.publish(i, j)
            rospy.sleep(5)

if __name__ == "__main__":
    grid_pattern()
