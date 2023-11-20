#!/usr/bin/env python
import numpy as np
import rospkg
from uuv_waypoints import Waypoint, WaypointSet

wps_array = np.array([
    [0, 0, -2],
    [5, 0, -2],
    [5, 5, -2],
    [-5, 5, -2],
    [-5, -5, -2],
    [5, -5, -2],
    [0, 0, -2]
])

waypoints = WaypointSet(scale=0.1, inertial_frame_id='world', max_surge_speed=.5)

for c_wp in wps_array:
    c_waypoint = Waypoint(x=c_wp[0], y=c_wp[1], z=c_wp[2], 
                          max_forward_speed=0.4, 
                          heading_offset=0, 
                          use_fixed_heading=False, 
                          inertial_frame_id='world', 
                          radius_acceptance=0.2)
    waypoints.add_waypoint(c_waypoint, add_to_beginning=False)

rospack = rospkg.RosPack()
wp_path = rospack.get_path("osl_simulator") + "/config/"
wp_file = "waypoints.yaml"
waypoints.export_to_file(wp_path, wp_file)