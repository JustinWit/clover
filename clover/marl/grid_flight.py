#!/usr/bin/env python3

import rospy
from clover import srv
from clover.msg import SendCoords
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import math
TAKEOFF_FRAME = 'map'

# Initialize Clover Services
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
land = rospy.ServiceProxy('land', Trigger)

# Set these according to grid size
SQUARE_SIZE = 0.573  # length of single cell in the grid in meters
MARKER_SIZE = 0.293  # total length of corner markers
OFFSET = SQUARE_SIZE/2 + MARKER_SIZE/2  # calculated to offset from the center of marker to center of grid cell

NAV_HGT = 2  # set the hovering height, need to be high enough to see markers while moving through the grid

def callback_setpos(msg):
    # adjust coords to reflect actual grid
    # so (0,0) would be the first square rather than over the marker
    gridx = (SQUARE_SIZE * msg.x) + OFFSET
    gridy = (SQUARE_SIZE * msg.y) + OFFSET
    out = set_position(x=gridx, y=gridy, z=NAV_HGT, frame_id=f'aruco_0')
    rospy.loginfo(f"Recieved x: {msg.x}\ty: {msg.y}")

def call_land():
    rospy.loginfo("Landing on exit")
    land()

def takeoff(height):
    input('Press <ENTER> to take off')
    print("Takeoff.. wait for ready")
    # takeoff
    navigate(x=0, y=0, z=height.data, frame_id='body', auto_arm=True)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id=TAKEOFF_FRAME)

        # reached our goal
        if abs(telem.z - height.data) < 0.2:
            break
        rospy.sleep(0.2)
    
    # navigate to (0, 0)
        navigate(x=0, y=0, z=height.data, frame_id=TAKEOFF_FRAME)
        while not rospy.is_shutdown():
            telem = get_telemetry(frame_id=TAKEOFF_FRAME)

            # reached our goal
            if math.sqrt((telem.x)**2 + (telem.y)**2) < 0.2:
                break
            rospy.sleep(0.2)
    print('Ready')

def flight():

    rospy.init_node('flight')

    # Subscribers
    rospy.Subscriber("set_pos", SendCoords, callback_setpos)
    rospy.Subscriber("takeoff", Int32, takeoff, queue_size=1)

    # land on shutdown
    rospy.on_shutdown(call_land)

    # spin ros node
    rospy.spin()

if __name__ == "__main__":
    flight()
    
