#!/usr/bin/env python3

import rospy
import threading
from clover import srv
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from sensor_msgs.msg import Range
from std_msgs.msg import Int32
import math

# Initialize Clover Services
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
# navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
# set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
# set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

class Drone():
    def __init__(self, name):
        # init node
        rospy.init_node(name)

        # class variables
        self.rng = 0  # drone current height
        self.lock = threading.Lock()
        self.path = None
        self.frame_id = None
        self.speed = 1
        self.tolerance = 0.1

    def set_range(self, rng):
        self.rng = rng

    def follow_path(self):
        with self.lock:
            for pt in self.path:
                rospy.sleep(1)
                t = pt.header.seq
                pose = pt.pose.position
                print(t, pose)

                # navigate(x=pose.x, y=pose.y, z=pose.z, speed=self.speed, frame_id=self.frame_id)
                set_position(x=pose.x, y=pose.y, z=pose.z, frame_id=self.frame_id)

                while not rospy.is_shutdown():
                    telem = get_telemetry(frame_id='navigate_target')
                    if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < self.tolerance:
                        break
                    rospy.sleep(0.01)

            self.path = None
            self.frame_id = None

    def takeoff(self, height):
        print("Takeoff.. wait for ready")
        navigate(x=0, y=0, z=height.data, frame_id='body', auto_arm=True)
        rospy.sleep(10)
        print('Ready')


def callback_path(path, drone):
    if drone.path == None:
        drone.path = path.poses
        drone.frame_id = path.header.frame_id

        drone.follow_path()

    else:
        pass

    
def call_land():
    print("Landing on exit")
    land()


def timeout():
    # hold current position if not receiving coordinates
    set_velocity(vx=0, vy=0, vz=0, frame_id='body')
    rospy.loginfo(f"Lost communication")


def flight():
    # initialize drone
    clover = Drone('clover')

    # Subscribers
    rospy.Subscriber("waypoints", Path, callback_path, callback_args=clover, queue_size=1)
    rospy.Subscriber("takeoff", Int32, clover.takeoff, queue_size=1)
    # rospy.Subscriber("rangefinder/range", Range, callback_range, callback_args=clover, queue_size=1)

    # land on shutdown
    rospy.on_shutdown(call_land)

    # spin ros node
    rospy.spin()

if __name__ == "__main__":
    flight()
    
