#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roslaunch dynamic_obstacle_avoidance my_lanefollowing.launch veh:=$VEHICLE_NAME
