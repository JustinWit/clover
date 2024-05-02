# Directions for Use
All work was done in Ubuntu 20.04

## Simulator Setup for Local Testing
Follow the [install directions](https://clover.coex.tech/en/simulation_native.html) from Clover. Rather than cloning the Clover package from clover use [this](https://github.com/JustinWit/clover/tree/master) GitHub link (First clone in the 'Clone Clover' sources section). The additional steps section is not required. This will mimic the environment on the drone, so use cases for the simulator will be the same on the drone. You'll just have to copy any code changes over to the Pi on the drone.

## Simulator Use
Source ROS noetic and the setup.bash file for the work space. Launch the simulator according to the Clover Documentation
```
$ roslaunch clover simulator.launch 
```

Refer to the README in the marl directory for directions for running code.

## Drone Use
Connect to the Pi on the drone through SSH. 

```
$ ssh pi@192.168.11.1
```

The password is: raspberry

You should set-up [rsa key authentication](https://www.digitalocean.com/community/tutorials/how-to-configure-ssh-key-based-authentication-on-a-linux-server). 

The `catkin_ws` contains the code for Clover. You should copy any of the work done on your computer to the respective directory on the Pi. 

## Connecting to QGround
Launch QGround and go to application settings, comm links. Change the type to TCP and enter the drones IP address: 192.168.11.1, the port should be set to 5760. Enter a name and press OK. You should be able to click on the created link and press connect. 

## Other Notes

`clover/launch` has the launch files that start all of the drones nodes. The `clover.launch` file is ran on startup for both the simulator and the drone so you shouldn't need to directly call it. 