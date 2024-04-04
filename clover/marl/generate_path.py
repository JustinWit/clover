#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import math

rospy.init_node('send_path', anonymous=True)
pub = rospy.Publisher('waypoints', Path, queue_size=10)

while pub.get_num_connections() < 1:
    rospy.sleep(.1)

radius = 1
num_points = 20

msg_head = Header(stamp=rospy.Time.now(), frame_id="map")
poses = []


for i in range(num_points):
    theta = ((2 * math.pi) / num_points) * i
    x = (radius * math.cos(theta)) - radius
    y = radius * math.sin(theta)
    z = 1


    # generate message
    header = Header(seq=i+1)
    pose = Pose(position=Point(x=x, y=y, z=z))

    poses.append(PoseStamped(header=header, pose=pose))

path = Path(header=msg_head, poses=poses)

pub.publish(path)

