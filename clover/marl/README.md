# File Details and Instruction
There are 3 different ways to send instructions to the drone. 

* grid_flight.py - Accepts grid coordinates to control drones movement over a grid.
* trajectory.py - Takes in a constant stream of direction and velocity for continuous control over the drone.
* waypoints - Takes in a set of points for the drone to navigate through.

### Launch
Launch any of the three files with rosrun. 
```
rosrun clover {file_name.py}
```

They all subscribe to the 'takeoff' topic which is a std_msgs/Int32 message, that is used to set the height for takeoff. I've just been publishing to this topic from the command line, but it could also be published to from another file. 

```
$ rostopic pub takeoff std_msgs/Int32 1
```
You'll be promted from the terminal running the main node to press enter to verfiy launch, then the drone will takeoff and hover at the specified height. Each file has a TAKEOFF_FRAME variable that can be set to change the frame the drone navigates to on takeoff. In the grid_flight and waypoints files, after launch it will also navigate to (0, 0) of the selected frame. The terminal will show 'ready' when the drone has finished the takeoff sequence.

### Takeoff 

Interpting the node with ctrl-c at anytime should land the drone immediately. If this fails you can directly call the land service from the terminal.
```
$ rosservice call land
```

Once the main node is running you can run the file that will be publishing to the expected topic.

### Environment
The node running the main files should be ran on Pi, so you can ssh to the Pi and use rosrun from there. Then you can publish to topics from your machine with the correct ROS network set up, make sure to source ros noetic and your local workspace as well.

```
$ export ROS_IP=192.168.11.183 && export ROS_MASTER_URI=http://192.168.11.1:11311 && export ROS_HOSTNAME=192.168.11.183
```


## grid_flight.py
This node uses an [aruco map](https://clover.coex.tech/en/aruco_map.html) for position estimation. Maps are defined in the `aruco_pose/maps` directory. Currently the simulator is set up to use 2 markers for navigation: one outside of the corner of the (0, 0) position, and one outside the (n, n) position. The code navigates based on the marker in the (0, 0) position, but because Clover has access to the map, that marker doesn't need to be in view for the drone to be able to estimate it's position, it just needs to see one of the markers in the map. If you change the marker_ids being used you need to change the frame_id value in the set_position call, this happens twice, it should be near lines 26 and 42. 

You can update the markers in the simulator following [Clovers docs](https://clover.coex.tech/en/simulation_usage.html#changing-the-map-of-aruco-markers-in-the-simulator), you'll just need to change the map file you pass in.

You should set the global variables in this file to match the environment.
* SQUARE_SIZE to the length of single cell in the grid in meters.
* MARKER_SIZE to the total length of corner markers.
* NAV_HGT to set the hovering height. This needs to be high enough to see markers while moving through the grid.

This node subscirbes to the 'set_pos' topic and uses a custom defined message 'SendCoords'. SendCoords is defined in clover/msg. There are just two arguements: x and y that should be set to the grid position being navigated to.

`test_grid.py` has an example of how to send points to the drone.


## trajectory.py
This node expects a constant stream of instructions for direction (radians) and speed (m/s). There is a timeout function so if the drone doesn't recieve a message for a set time (default 0.5 seconds) it will hold at its current location.

It subscribes to the 'trajectory_stream' topic with the message type [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html). Speed should be sent in the linear.x arguement, and direction in the angular.z.

There can be unexpected behaviors if the node recieves messages at too high of a rate. `stream.py` publishes at a rate of 10Hz and seems to give expected results. 

`stream.py` can be used to publish the trajectory of a circle. `keycontrol.py` and `mousecontrol.py` can be used to send trajectories based on keyboard or mouse inputs. `mousecontrol.py` will need to be set up for your machine and you'll have to edit some permissions to allow access. 


## waypoints
This node takes in a path expressed as waypoints in the map. There are three approaches to how the drone should actually navigate these points, they are each on a seperate file described below. 

### waypoints.py
This is the most basic approach. It treats each point as the final destination so it will stop and start at each point along the path. This gives the most accurate following of the path, but the navigation can be jerky and it tends to fall behind on the times set for navigation because its stopping so much.

### waypoints_proj.py
This attempts to solve the stopping and starting issue by projecting the point it is navigating to futher than it needs to go. This means the drone doesn't slow down as it goes through the point sent as part of the path. This creates a more seamless transition navigating from point to point, but the error in following the path is increased. And since the drone won't update the point it is navigating to until it is within a set tolerance, if it misses the point it was supposed to navigate to it will circle around to go back through the point. This mostly happens when trying to navigate through the path quickly, or if there are sharp angles in the path.

### waypoints_carrot.py
To try and decrease how often the drone misses the point it was supposed to navigate through, this method changes where the drone is navigating based on how far from the path it is. So if the drone is far from the path, rather than simply ignoring the error and navigating to the next point, it focuses on getting closer to the path first, then looking ahead to the point it needs to get to. 

### Use Cases
To send a path, the node subscribes to the 'waypoints' topic; the message type is [nav_msgs/Path](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html). There is a header for the Path message, the [frame_id](https://clover.coex.tech/en/frames.html) should be set according to the frame_id to be used to navigate. Path contains a list of PoseStamped messages, each containing a header and a pose. The seq value for each header should be set to the time step in milliseconds for each position. The (x, y, z) position for each pose should be set according to the frame_id passed from the main header.

`generate_path.py` and `custom_path.py` are ways to create and send waypoints. `generate_path.py` will generate the points for a circle, you can set the circle radius, the number of points to generate, and the total time to complete the circle. `custom_path.py` has two lists for the x and y positions that can be set to create your own path to test. You can also set the timestep between points, and the height for navigating. Both of these files will wait until two subscribers are connected to allow the path to be shown on rviz. If you aren't using rviz you should lower the number of connections set in the first while loop to 1. 

## Other notes
The `clover.rviz` file is set up with tabs for viewing the camera and aruco feeds, and is also connected to some topics for visualization. 

