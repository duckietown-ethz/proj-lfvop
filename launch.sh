#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roslaunch dynamic_obstacle_avoidance dynamic_obstacle_avoidance.launch veh:=$VEHICLE_NAME
