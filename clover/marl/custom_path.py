#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import math

rospy.init_node('send_path', anonymous=True)
pub = rospy.Publisher('waypoints', Path, queue_size=10)

while pub.get_num_connections() < 2:
    rospy.sleep(.1)




msg_head = Header(stamp=rospy.Time.now(), frame_id="map")
poses = []

ptsx = [0,   1, 2,   3, 4,   5, 6,   7, 8,   9, 10,  10.5, 10,  9,   8,    7,  6,   5, 4,   3, 2 , 1, 0]
ptsy = [0.5, 1, 1.5, 1, 0.5, 0, 0.5, 1, 1.5, 1, 0.5, 0,   -0.5, -1, -1.5, -1, -0.5, 0, -0.5, -1, -1.5, -1, 0]
z = 1
time_step = 1000


for i, (x, y) in enumerate(zip(ptsx, ptsy)):
    # generate message
    header = Header(seq=int(time_step * (i+1)))
    pose = Pose(position=Point(x=x, y=y, z=z))

    poses.append(PoseStamped(header=header, pose=pose))

path = Path(header=msg_head, poses=poses)
pub.publish(path)

