#!/usr/bin/env python3

import rospy
import threading
from clover import srv
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from sensor_msgs.msg import Range
from std_msgs.msg import Int32
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import tf

import math

# Initialize Clover Services
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
land = rospy.ServiceProxy('land', Trigger)
head = Header(frame_id='map')

class Drone():
    def __init__(self, name):
        # init node
        rospy.init_node(name)

        # class variables
        self.path = None
        self.frame_id = None
        self.speed = 0
        self.tolerance = 0.3
        self.pub = None


    def follow_path(self):
        prev_t = 0
        start_t = rospy.get_time()

        telem = get_telemetry(frame_id='map')
        prev_x = telem.x
        prev_y = telem.y

        for pt in self.path:
            # time step
            t = pt.header.seq

            # goal position
            abs_x = pt.pose.position.x
            abs_y = pt.pose.position.y
            abs_z = pt.pose.position.z
            
            # speed
            dist = math.sqrt((prev_x - abs_x)**2 + (prev_y - abs_y)**2)
            self.speed = dist / ((t - prev_t) / 1000)

            # lookahead distances
            c_dist = dist / 4  # fix the distance we are going to navigate to
            proj_dist = c_dist * 2  # fix how far we are projecting that point
            
            # loop til in tolerance of goal updating lookahead point as we go
            while not rospy.is_shutdown():
                telem = get_telemetry(frame_id='map')
                dist = math.sqrt((telem.x - abs_x)**2 + (telem.y - abs_y)**2)

                # reached our goal
                if dist < self.tolerance:
                    break
                # update proj_x and proj_y
                else:
                    # dist_num = abs((abs_x - prev_x) * (telem.y - prev_y) - (telem.x - prev_x) * (abs_y - prev_y))  # from wikipedia distance from point to line
                    # p_error = dist_num / dist  # orthogonal distance from drone to real path
                    # dist_p = math.sqrt((telem.x - prev_x)**2 + (telem.y - prev_y)**2)  # drone distance from start point


                    # dist_line = math.sqrt(abs(dist_p**2 - p_error**2)) + math.sqrt(abs(c_dist**2 - p_error**2))

                    linept = abs_x, abs_y

                    proj_x = telem.x + ((dist + proj_dist) / dist) * (linept[0] - telem.x)
                    proj_y = telem.y + ((dist + proj_dist) / dist) * (linept[1] - telem.y)                

                # call clover navigate with out projection points
                navigate(x=proj_x, y=proj_y, z=abs_z, speed=self.speed, frame_id=self.frame_id)
                self.publish_proj(telem, proj_x, proj_y, abs_z)  #rviz display of projection points
                rospy.sleep(.01)
            
            # track necessary previous values for next trajectory point
            prev_t = t
            prev_x = abs_x
            prev_y = abs_y

        # end at abs_pose
        navigate(x=abs_x, y=abs_y, z=abs_z, speed=self.speed, frame_id=self.frame_id)

        print(f"Total time: {rospy.get_time() - start_t}")

        # reset class variables
        self.path = None
        self.frame_id = None


    def publish_proj(self, telem, x, y, z):
        yaw = math.atan2(y - telem.y, x - telem.x)
        orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))


        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = 0
        marker.id = 0
        marker.scale.x = math.sqrt((x - telem.x)**2 + (y - telem.y)**2)
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = telem.x
        marker.pose.position.y = telem.y
        marker.pose.position.z = z
        marker.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))

        self.pub.publish(marker)


    def takeoff(self, height):
        input('Press <ENTER> to take off')
        print("Takeoff.. wait for ready")
        # takeoff
        navigate(x=0, y=0, z=height.data, frame_id='body', auto_arm=True)
        while not rospy.is_shutdown():
            telem = get_telemetry(frame_id='map')

            # reached our goal
            if abs(telem.z - height.data) < 0.2:
                break
            rospy.sleep(0.2)
                
        # navigate to (0, 0)
        navigate(x=0, y=0, z=height.data, frame_id='map')
        while not rospy.is_shutdown():
            telem = get_telemetry(frame_id='map')

            # reached our goal
            if math.sqrt((telem.x)**2 + (telem.y)**2) < 0.2:
                break
            rospy.sleep(0.2)
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
    set_velocity(vx=0, vy=0, vz=0, frame_id='body')
    rospy.sleep(.5)
    print("Landing on exit")
    land()


def flight():
    # initialize drone
    clover = Drone('clover')

    # Subscribers
    rospy.Subscriber("waypoints", Path, callback_path, callback_args=clover, queue_size=1)
    rospy.Subscriber("takeoff", Int32, clover.takeoff, queue_size=1)

    # publisher for rviz
    pub = rospy.Publisher("carrot", Marker, queue_size=10)
    clover.pub = pub

    # land on shutdown
    rospy.on_shutdown(call_land)

    # spin ros node
    rospy.spin()

if __name__ == "__main__":
    flight()
    
