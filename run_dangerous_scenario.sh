#!/bin/bash

# DELAY = 98
# DELAY2 = 205

# start from the beginning: kill all gazebo and rviz windows
clear

pkill -9 gzserver
pkill -9 gzclient

#Command 1 running in the background
roslaunch remaro_scenarios launch_remaro_scenario.launch&

#Command 2 running in the background: 
# put delay between first Large Vertical Tank Inspection
#, and go to other side of Subsea Infrastructure
(sleep 98; roslaunch remaro_scenarios send_dangerous_waypoints.launch)&

(sleep 205; roslaunch remaro_scenarios launch_small_vertical_tank_inspection.launch)&

# print all process finished
# echo "all scenarios launched"




