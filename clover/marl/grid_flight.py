#!/usr/bin/env python3

import rospy
from clover import srv
from clover.msg import SendCoords
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

# Initialize Clover Services
# get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
# navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
# set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
# set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
# set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def callback_setpos(msg):
    # adjust coords to reflect actual space
    # so (0,0) would be the first square rather than over the marker
    gridx = (0.573 * msg.x) + 0.4335
    gridy = (0.573 * msg.y) + 0.4335
    out = set_position(x=gridx, y=gridy, z=2, frame_id=f'aruco_0')
    print(out)

def call_land():
    print("Landing on exit")
    land()


def flight():

    rospy.init_node('flight')
    # on start takeoff and hover 2m
    
    rospy.loginfo("Takeoff")
    navigate(x=0, y=0, z=2, frame_id='body', auto_arm=True)
    rospy.sleep(5)
    rospy.loginfo(set_position(x=0.4335, y=0.4335, z=2, frame_id='aruco_0'))


    # Subscribers
    rospy.Subscriber("set_pos", SendCoords, callback_setpos)

    # land on shutdown
    rospy.on_shutdown(call_land)

    # spin ros node
    rospy.spin()

if __name__ == "__main__":
    flight()
    
