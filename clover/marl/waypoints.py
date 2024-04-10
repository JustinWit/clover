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
        self.speed = .5
        self.tolerance = 0.3
        self.rate = rospy.Rate(15)

    def set_range(self, rng):
        self.rng = rng

    def follow_path(self):
        prev_t = 0
        start_t = rospy.get_time()
        for pt in self.path:
            t = pt.header.seq
            telem = get_telemetry(frame_id='map')

            # true navigation point
            abs_x = pt.pose.position.x
            abs_y = pt.pose.position.y
            abs_z = pt.pose.position.z

            # projected navigation point
            proj_x = abs_x + (abs_x - telem.x) * 2
            proj_y = ((abs_y - telem.y) / (abs_x - telem.x)) * (proj_x - telem.x) + telem.y

            # speed
            dist = math.sqrt((telem.x - abs_x)**2 + (telem.y - abs_y)**2)
            self.speed = dist / ((t - prev_t) / 1000)
            
            print(f't: {t}\nv: {self.speed}\npose:{(abs_x, abs_y)}')
            # print(f"Locl x: {telem.x:.4f}\tReal x: {abs_x:.4f}\t Proj x:{proj_x:.4f}")
            # print(f"Locl y: {telem.y:.4f}\tReal y: {abs_y:.4f}\t Proj y:{proj_y:.4f}")

            # navigate(x=proj_x, y=proj_y, z=abs_z, speed=self.speed, frame_id=self.frame_id)
            # set_position(x=pose.position.x, y=pose.position.y, z=pose.position.z, frame_id=self.frame_id)

            
            while not rospy.is_shutdown():
                navigate(x=proj_x, y=proj_y, z=abs_z, speed=self.speed, frame_id=self.frame_id)
                telem = get_telemetry(frame_id='map')

                # reached our goal
                if math.sqrt((telem.x - abs_x)**2 + (telem.y - abs_y)**2) < self.tolerance:
                    print(f"T actual: {rospy.get_time() - start_t}\n")
                    break
                # need to keep navigating so update proj_x and proj_y
                else:
                    proj_x = abs_x + (abs_x - telem.x) * 2
                    proj_y = ((abs_y - telem.y) / (abs_x - telem.x)) * (proj_x - telem.x) + telem.y

                    # print(f"Locl x: {telem.x:.4f}\tReal x: {abs_x:.4f}\t Proj x:{proj_x:.4f}")
                    # print(f"Locl y: {telem.y:.4f}\tReal y: {abs_y:.4f}\t Proj y:{proj_y:.4f}")

                rospy.sleep(.1)
            
            prev_t = t

        self.path = None
        self.frame_id = None

    def takeoff(self, height):
        print("Takeoff.. wait for ready")
        navigate(x=0, y=0, z=height.data, frame_id='map', auto_arm=True)
        rospy.sleep(10)
        print('Ready')


def callback_path(path, drone):
    print("Got path")
    if drone.path == None:
        drone.path = path.poses
        drone.frame_id = path.header.frame_id

        drone.follow_path()

    else:
        print("Already Running")
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
    
