#!/bin/bash

### Set IP address for ROS
#IP=`ifconfig wlan0 | grep "inet " | awk '{print $2}' | awk -F':' '{print $2}'` # automatically detect wifi address (TODO: ensure wifi is connected before)
#IP=localhost		# Locally on odroid only
#IP=192.168.1.199	# Ethernet
IP=192.168.43.129	# Wifi

### ROS Setup
export ROS_HOSTNAME=$IP
export ROS_MASTER_URI=http://$IP:11311
source /opt/ros/indigo/setup.bash
cd /root/catkin_ws
source devel/setup.bash

### Start ROS launch scripts
roslaunch manual_control manual_odroid.launch &
#roslaunch random_movement auto.launch &

