#!/usr/bin/env python3

import rospy
import threading
from clover import srv
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import math

# Initialize Clover Services
# get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
# navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
# set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
# set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

class Drone():
    def __init__(self, name, timer):
        # init node
        rospy.init_node(name)
        self.timer = timer

        # class variables
        self.rng = 0  # drone current height
        self.altitude = 2  # desired flight height

    def set_range(self, rng):
        self.rng = rng

    def set_altitude(self, altitude):
        self.altitude = altitude


def callback_position(vector, args):
    # get args
    drone = args[0]
    lock = args[1]

    # reset timer
    drone.timer.cancel()
    drone.timer = threading.Timer(.2, timeout)

    # extract message
    x, z = vector.linear.x, vector.angular.z

    if vector.linear.z != 0:
        drone.set_altitude(vector.linear.z)

    # calculate new position
    x_pos = x * math.cos(z)
    y_pos = x * math.sin(z)

    # avoid deadlocking
    with lock:
        z_pos = drone.altitude - drone.rng

    # navigate based on position so we can adjust height
    set_position(x=x_pos, y=y_pos, z=z_pos, frame_id='body')
    rospy.loginfo(f"\nx: {x_pos}\ny: {y_pos}\nz: {z_pos}\n")

    # start next timer
    drone.timer.start()


def callback_range(msg, args):
    # parse args
    drone = args[0]
    lock = args[1]

    # avoid deadlocking
    with lock:
        drone.set_range(msg.range)
    # TODO: add safety for flying above some height

def call_land():
    print("Landing on exit")
    land()


def timeout():
    # hold current position if not receiving coordinates
    set_velocity(vx=0, vy=0, vz=0, frame_id='body')
    rospy.loginfo(f"Lost communication")


def flight():
    # initialize drone
    timer = threading.Timer(.5, timeout)
    clover = Drone('clover', timer)

    # thread lock
    lock = threading.Lock()

    # Subscribers
    rospy.Subscriber("trajectory_stream", Twist, callback_position, callback_args=(clover, lock), queue_size=10)
    rospy.Subscriber("rangefinder/range", Range, callback_range, callback_args=(clover, lock), queue_size=1)

    # land on shutdown
    rospy.on_shutdown(call_land)

    # spin ros node
    rospy.spin()

if __name__ == "__main__":
    flight()
    
