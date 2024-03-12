#!/bin/bash

# DELAY = 98
# DELAY2 = 205

# start from the beginning: kill all gazebo and rviz windows
clear

pkill -9 gzserver
pkill -9 gzclient
killall -9 rosmaster
killall -9 rosout
killall -9 roscore

sleep 10;

#Command 1 running in the background
roslaunch remaro_scenarios launch_remaro_dangerous_scenario_segment.launch&

#Command 2 running in the background: 
# put delay between first Large Vertical Tank Inspection
#, and go to other side of Subsea Infrastructure
(sleep 10; roslaunch remaro_scenarios send_dangerous_waypoints_segment.launch)&

(sleep 15; rosbag record /ground_truth_to_tf_bluerov2/pose)&

# print all process finished
# echo "all scenarios launched"


