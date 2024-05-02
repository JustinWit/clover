#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import math

rospy.init_node('send_path', anonymous=True)
pub = rospy.Publisher('waypoints', Path, queue_size=1)

while pub.get_num_connections() < 1:
    rospy.sleep(.1)

radius = 2
num_points = 24
completion_time = 45000
time_step = completion_time / num_points

msg_head = Header(stamp=rospy.Time.now(), frame_id="aruco_113")
poses = []


for i in range(num_points):
    theta = ((2 * math.pi) / num_points) * i
    x = (radius * math.cos(theta)) - radius
    y = radius * math.sin(theta)
    z = 2


    # generate message
    header = Header(seq=int(time_step * (i+1)))
    pose = Pose(position=Point(x=x, y=y, z=z))

    poses.append(PoseStamped(header=header, pose=pose))

# # generate test message
# header = Header(seq=int(5000))
# pose = Pose(position=Point(x=1, y=1, z=1))
# poses.append(PoseStamped(header=header, pose=pose))

# header = Header(seq=int(10000))
# pose = Pose(position=Point(x=0, y=0, z=1))
# poses.append(PoseStamped(header=header, pose=pose))


header = Header(seq=int(time_step * (num_points+1)))
pose = Pose(position=Point(x=0, y=0, z=z))
poses.append(PoseStamped(header=header, pose=pose))

path = Path(header=msg_head, poses=poses)

pub.publish(path)

